#include "imu.h"

#include <math.h>

#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define IMU_TIMEOUT_MS 100

#define REG_INT1_CTRL 0x0D
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL4_C 0x13
#define REG_STATUS 0x1E
#define REG_OUTX_L_G 0x22

#define WHO_AM_I_LSM6DS3 0x69
#define WHO_AM_I_LSM6DS3TRC 0x6A

#define ODR_104_HZ 0x40

#define FS_XL_4G 0x08
#define ACCEL_SCALE_4G_MG 0.122f

#define FS_G_1000DPS 0x08
#define GYRO_SCALE_1000DPS_MDPS 35.0f

#define IMU_SETTLING_SAMPLES 20

static bool write_reg(imu_t *imu, uint8_t reg, uint8_t value)
{
	uint8_t data[] = {reg, value};
	return i2c_master_transmit(imu->device, data, sizeof(data), IMU_TIMEOUT_MS) == ESP_OK;
}

static bool read_regs_pleasework(imu_t *imu, uint8_t reg, uint8_t *data, size_t length)
{
	return i2c_master_transmit_receive(imu->device, &reg, 1, data, length,
		IMU_TIMEOUT_MS) == ESP_OK;
}

static bool read_reg(imu_t *imu, uint8_t reg, uint8_t *value)
{
	return read_regs_pleasework(imu, reg, value, 1);
}

static void IRAM_ATTR drdy_interrupt_thing(void *arg)
{
	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR((SemaphoreHandle_t)arg, &higher_priority_task_woken);
	if (higher_priority_task_woken) {
		portYIELD_FROM_ISR();
	}
}

float imu_fix_angle_pleasework(float degrees)
{
	degrees = fmodf(degrees + 180.0f, 360.0f);
	if (degrees < 0.0f) {
		degrees += 360.0f;
	}
	return degrees - 180.0f;
}

void imu_reset_angle(imu_t *imu)
{
	imu->yaw_deg = 0.0f;
	imu->last_sample_us = 0;
}

bool imu_setup(imu_t *imu, i2c_master_bus_handle_t bus, uint8_t address)
{
	const uint8_t addresses[] = {address, address == LSM6DS3_ADDR_LOW ?
		LSM6DS3_ADDR_HIGH : LSM6DS3_ADDR_LOW};

	imu->gyro_scale_dps = GYRO_SCALE_1000DPS_MDPS / 1000.0f;
	imu->accel_scale_g = ACCEL_SCALE_4G_MG / 1000.0f;
	imu->yaw_bias_dps = 0.0f;
	imu->yaw_deg = 0.0f;
	imu->accel_zero_fwd_g = 0.0f;
	imu->accel_zero_lat_g = 0.0f;
	imu->last_sample_us = 0;
	imu->drdy_sem = NULL;

	uint8_t who_am_i;
	bool found = false;
	for (size_t i = 0; i < sizeof(addresses); ++i) {
		const i2c_device_config_t device_config = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = addresses[i],
			.scl_speed_hz = 100000
		};
		if (i2c_master_bus_add_device(bus, &device_config, &imu->device) != ESP_OK) {
			continue;
		}
		if (read_reg(imu, REG_WHO_AM_I, &who_am_i) &&
			(who_am_i == WHO_AM_I_LSM6DS3 || who_am_i == WHO_AM_I_LSM6DS3TRC)) {
			found = true;
			break;
		}
		i2c_master_bus_rm_device(imu->device);
	}
	if (!found) {
		return false;
	}

	if (!write_reg(imu, REG_CTRL3_C, 0x01)) {
		i2c_master_bus_rm_device(imu->device);
		return false;
	}
	for (int attempt = 0; attempt < 20; ++attempt) {
		uint8_t ctrl3;
		vTaskDelay(pdMS_TO_TICKS(5));
		if (!read_reg(imu, REG_CTRL3_C, &ctrl3)) {
			i2c_master_bus_rm_device(imu->device);
			return false;
		}
		if ((ctrl3 & 0x01) == 0) {
			break;
		}
	}

	if (!write_reg(imu, REG_CTRL3_C, 0x44) ||
		!write_reg(imu, REG_CTRL1_XL, ODR_104_HZ | FS_XL_4G) ||
		!write_reg(imu, REG_CTRL2_G, ODR_104_HZ | FS_G_1000DPS) ||
		!write_reg(imu, REG_CTRL4_C, 0x00)) {
		i2c_master_bus_rm_device(imu->device);
		return false;
	}

	imu_sample_t discard;
	for (int i = 0; i < IMU_SETTLING_SAMPLES; ++i) {
		vTaskDelay(pdMS_TO_TICKS(10));
		imu_get_stuff_pleasework(imu, &discard);
	}
	imu_reset_angle(imu);
	return true;
}

bool imu_start_interrupt_pleasework(imu_t *imu, gpio_num_t int_pin)
{
	SemaphoreHandle_t sem = xSemaphoreCreateBinary();
	if (sem == NULL) {
		return false;
	}

	const gpio_config_t int_config = {
		.pin_bit_mask = 1ULL << int_pin,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE
	};
	if (gpio_config(&int_config) != ESP_OK) {
		vSemaphoreDelete(sem);
		return false;
	}

	esp_err_t isr_service = gpio_install_isr_service(0);
	if (isr_service != ESP_OK && isr_service != ESP_ERR_INVALID_STATE) {
		vSemaphoreDelete(sem);
		return false;
	}
	if (gpio_isr_handler_add(int_pin, drdy_interrupt_thing, sem) != ESP_OK) {
		vSemaphoreDelete(sem);
		return false;
	}

	if (!write_reg(imu, REG_INT1_CTRL, 0x02)) {
		gpio_isr_handler_remove(int_pin);
		vSemaphoreDelete(sem);
		return false;
	}

	imu->drdy_sem = sem;
	return true;
}

bool imu_wait_for_data(imu_t *imu, uint32_t timeout_ms)
{
	if (imu->drdy_sem == NULL) {
		return false;
	}
	return xSemaphoreTake((SemaphoreHandle_t)imu->drdy_sem,
		pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool imu_get_stuff_pleasework(imu_t *imu, imu_sample_t *sample)
{
	uint8_t raw[12];
	if (!read_regs_pleasework(imu, REG_OUTX_L_G, raw, sizeof(raw))) {
		return false;
	}

	int16_t gyro_z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
	int16_t accel_x = (int16_t)((uint16_t)raw[7] << 8 | raw[6]);
	int16_t accel_y = (int16_t)((uint16_t)raw[9] << 8 | raw[8]);

	int64_t now_us = esp_timer_get_time();
	float dt_s = 0.0f;
	if (imu->last_sample_us != 0) {
		dt_s = (float)(now_us - imu->last_sample_us) / 1000000.0f;

		if (dt_s > 0.2f) {
			dt_s = 0.2f;
		}
	}
	imu->last_sample_us = now_us;

	float yaw_rate = IMU_YAW_SIGN * ((float)gyro_z * imu->gyro_scale_dps) - imu->yaw_bias_dps;
	imu->yaw_deg += yaw_rate * dt_s;

	sample->yaw_deg = imu->yaw_deg;
	sample->yaw_rate_dps = yaw_rate;
	sample->accel_fwd_g = IMU_FORWARD_SIGN * (float)accel_x * imu->accel_scale_g -
		imu->accel_zero_fwd_g;
	sample->accel_lat_g = IMU_LATERAL_SIGN * (float)accel_y * imu->accel_scale_g -
		imu->accel_zero_lat_g;
	sample->dt_s = dt_s;
	return true;
}

bool imu_zero_the_accel(imu_t *imu, int samples)
{
	double sum_fwd_g = 0.0;
	double sum_lat_g = 0.0;
	int collected = 0;

	imu->accel_zero_fwd_g = 0.0f;
	imu->accel_zero_lat_g = 0.0f;
	for (int i = 0; i < samples; ++i) {
		imu_sample_t sample;
		if (imu->drdy_sem != NULL) {
			if (!imu_wait_for_data(imu, 50)) {
				continue;
			}
		} else {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		if (imu_get_stuff_pleasework(imu, &sample)) {
			sum_fwd_g += sample.accel_fwd_g;
			sum_lat_g += sample.accel_lat_g;
			collected++;
		}
	}

	if (collected < samples / 2) {
		return false;
	}
	imu->accel_zero_fwd_g = (float)(sum_fwd_g / collected);
	imu->accel_zero_lat_g = (float)(sum_lat_g / collected);
	return true;
}

bool imu_fix_drift_pleasework(imu_t *imu, int samples)
{
	double sum_dps = 0.0;
	int collected = 0;
	bool use_drdy = imu->drdy_sem != NULL;

	imu->yaw_bias_dps = 0.0f;
	for (int i = 0; i < samples; ++i) {
		imu_sample_t sample;

		if (use_drdy) {
			if (!imu_wait_for_data(imu, 50)) {
				use_drdy = false;
				vTaskDelay(pdMS_TO_TICKS(10));
			}
		} else {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		if (imu_get_stuff_pleasework(imu, &sample)) {
			sum_dps += sample.yaw_rate_dps;
			collected++;
		}
	}

	if (collected < samples / 2) {
		return false;
	}
	imu->yaw_bias_dps = (float)(sum_dps / collected);
	imu_reset_angle(imu);
	return true;
}
