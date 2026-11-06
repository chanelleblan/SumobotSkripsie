#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#define LSM6DS3_ADDR_LOW 0x6A
#define LSM6DS3_ADDR_HIGH 0x6B

#define IMU_YAW_SIGN (+1.0f)
#define IMU_FORWARD_SIGN (+1.0f)
#define IMU_LATERAL_SIGN (+1.0f)

#define IMU_SAMPLE_RATE_HZ 104.0f

typedef struct {
	i2c_master_dev_handle_t device;
	float gyro_scale_dps;
	float accel_scale_g;
	float yaw_bias_dps;
	float yaw_deg;
	float accel_zero_fwd_g;
	float accel_zero_lat_g;
	int64_t last_sample_us;
	void *drdy_sem;
} imu_t;

typedef struct {
	float yaw_deg;
	float yaw_rate_dps;
	float accel_fwd_g;
	float accel_lat_g;
	float dt_s;
} imu_sample_t;

bool imu_setup(imu_t *imu, i2c_master_bus_handle_t bus, uint8_t address);

bool imu_start_interrupt_pleasework(imu_t *imu, gpio_num_t int_pin);
bool imu_wait_for_data(imu_t *imu, uint32_t timeout_ms);

bool imu_fix_drift_pleasework(imu_t *imu, int samples);
bool imu_zero_the_accel(imu_t *imu, int samples);

bool imu_get_stuff_pleasework(imu_t *imu, imu_sample_t *sample);
void imu_reset_angle(imu_t *imu);

float imu_fix_angle_pleasework(float degrees);

#endif
