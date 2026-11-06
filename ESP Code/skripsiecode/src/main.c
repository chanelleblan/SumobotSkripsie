#include <inttypes.h>
#include <math.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu.h"
#include "pid.h"
#include "telemetry.h"
#include "vl53l0x.h"

#define LED_GPIO GPIO_NUM_2
#define BUTTON_GPIO GPIO_NUM_23
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define RIGHT_SENSOR_XSHUT_GPIO GPIO_NUM_16
#define LEFT_SENSOR_XSHUT_GPIO GPIO_NUM_17
#define LEFT_SIDE_SENSOR_XSHUT_GPIO GPIO_NUM_19
#define RIGHT_SIDE_SENSOR_XSHUT_GPIO GPIO_NUM_18
#define RIGHT_SENSOR_ADDRESS 0x30
#define LEFT_SENSOR_ADDRESS 0x31
#define LEFT_SIDE_SENSOR_ADDRESS 0x32
#define RIGHT_SIDE_SENSOR_ADDRESS 0x33
#define IMU_INT1_GPIO GPIO_NUM_4
#define LEFT_EDGE_SENSOR_GPIO GPIO_NUM_13
#define EDGE_SENSOR_WHITE_LEVEL 0

#define LED_DELAY_MS 4000
#define MOTOR_START_DELAY_MS 5000
#define TARGET_DISTANCE_MM 300
#define ATTACK_DISTANCE_MM 200
#define ATTACK_EXIT_DISTANCE_MM 250
#define SENSOR_LED_DISTANCE_MM 100
#define EDGE_SENSOR_POLL_INTERVAL_MS 1
#define EDGE_REVERSE_SPEED 255
#define EDGE_REVERSE_DURATION_MS 600
#define EDGE_TURN_FALLBACK_DURATION_MS 1200
#define EDGE_TURN_TOLERANCE_DEG 3.0f
#define EDGE_TURN_MAX_RATE_DPS 12.0f
#define EDGE_TURN_STABLE_MS 100
#define EDGE_TURN_TIMEOUT_MS 5000
#define EDGE_RETREAT_TIMEOUT_MS 1500
#define EDGE_SURFACE_CLEAR_STABLE_MS 30
#define SEARCH_SPIN_OUTPUT 170
#define TARGET_ALIGN_TURN_OUTPUT 90
#define EDGE_TURN_FALLBACK_OUTPUT TARGET_ALIGN_TURN_OUTPUT
#define TARGET_ALIGN_LOST_TIMEOUT_MS 300
#define TARGET_ALIGN_STABLE_MS 150
#define TARGET_ALIGN_MAX_FRONT_DIFFERENCE 0.15f
#define TARGET_ALIGN_MIN_FRONT_STRENGTH 0.05f
#define APPROACH_MIN_FORWARD_OUTPUT 40
#define APPROACH_MAX_FORWARD_OUTPUT 255
#define APPROACH_MAX_TURN_OUTPUT 150
#define STARTUP_TARGET_WAIT_MS 1000
#define FLANK_CLOSING_RATE_MM_S 800.0f
#define FLANK_REQUIRED_FAST_SAMPLES 2
#define FLANK_DODGE_TURN_DEG 48.0f
#define FLANK_TURN_OUTPUT TARGET_ALIGN_TURN_OUTPUT
#define FLANK_TURN_FALLBACK_DURATION_MS 320
#define FLANK_TURN_TIMEOUT_MS 3000
#define FLANK_STRAIGHT_DURATION_MS 400
#define FLANK_STRAIGHT_FORWARD_OUTPUT 120
#define FLANK_ARC_MIN_DURATION_MS 250
#define FLANK_ARC_FORWARD_OUTPUT 100
#define FLANK_ARC_TURN_OUTPUT 65
#define FLANK_ARC_TIMEOUT_MS 2500
#define IMPACT_ACCEL_DELTA_G 0.8f
#define IMPACT_ARM_DELAY_MS 150
#define CONTACT_MIN_PUSH_MS 500
#define WOODPECKER_STALL_TIMEOUT_MS 1000
#define WOODPECKER_NO_IMPACT_TIMEOUT_MS 500
#define WOODPECKER_FORWARD_DURATION_MS 150
#define WOODPECKER_REVERSE_DURATION_MS 100
#define WOODPECKER_REVERSE_OUTPUT 250
#define WOODPECKER_RANGE_PROGRESS_MM 15

#define MOTOR_A1_GPIO GPIO_NUM_27
#define MOTOR_A2_GPIO GPIO_NUM_26
#define MOTOR_A_PWM_GPIO GPIO_NUM_14
#define MOTOR_B1_GPIO GPIO_NUM_25
#define MOTOR_B2_GPIO GPIO_NUM_33
#define MOTOR_B_PWM_GPIO GPIO_NUM_32

#define MOTOR_PWM_FREQUENCY_HZ 20000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_MAX_OUTPUT 255

#define SENSOR_POLL_INTERVAL_MS 5
#define CONTROL_LOOP_TIMEOUT_MS 50

#define PID_KP 15.0f
#define PID_KI 0.0f
#define PID_KD 0.6f
#define PID_INTEGRAL_LIMIT 60.0f
#define PID_OUTPUT_LIMIT 200.0f

#define MATCH_TIMEOUT_MS 180000

typedef enum {
	ROBOT_IDLE,
	ROBOT_WAITING,
	ROBOT_SEARCHING,
	ROBOT_APPROACHING,
	ROBOT_STOPPED,
	ROBOT_ATTACKING,
	ROBOT_PUSHING,
	ROBOT_ALIGNING,
	ROBOT_EDGE_RECOVERY,
	ROBOT_WOODPECKER,
	ROBOT_STARTUP_WAIT,
	ROBOT_DODGE_TURN,
	ROBOT_DODGE_STRAIGHT,
	ROBOT_FLANKING
} robot_state_t;

static const char *TAG = "sumo";

static SemaphoreHandle_t motor_mutex;

static portMUX_TYPE target_lock_pleasework = portMUX_INITIALIZER_UNLOCKED;
static bool target_visible;
static uint16_t target_range_left_mm = UINT16_MAX;
static uint16_t target_range_right_mm = UINT16_MAX;
static bool target_left_near;
static bool target_right_near;
static bool target_left_side_near;
static bool target_right_side_near;
static uint16_t target_range_left_side_mm = UINT16_MAX;
static uint16_t target_range_right_side_mm = UINT16_MAX;

static volatile robot_state_t robot_state_now = ROBOT_IDLE;
static volatile TickType_t start_time_pleasework;

static imu_t imu;
static pid_t my_pid;
static vl53l0x_t left_sensor;
static vl53l0x_t right_sensor;
static vl53l0x_t left_side_sensor;
static vl53l0x_t right_side_sensor;
static bool left_sensor_initialized;
static bool right_sensor_initialized;
static bool left_side_sensor_initialized;
static bool right_side_sensor_initialized;
static volatile bool imu_ok;
static volatile bool imu_check_complete;
static volatile bool imu_drdy_fault;
static bool edge_stopped_pleasework;
static bool edge_detection_armed = true;
static volatile bool edge_surface_is_white;
static volatile TickType_t edge_recovery_detected_ticks;
static volatile TickType_t edge_surface_clear_since_ticks;

static void move_motor(gpio_num_t in1, gpio_num_t in2, ledc_channel_t channel, int speed)
{
	uint32_t duty = speed >= 0 ? (uint32_t)speed : (uint32_t)(-speed);

	if (speed > 0) {
		gpio_set_level(in1, 1);
		gpio_set_level(in2, 0);
	} else if (speed < 0) {
		gpio_set_level(in1, 0);
		gpio_set_level(in2, 1);
	} else {
		gpio_set_level(in1, 0);
		gpio_set_level(in2, 0);
	}

	ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void stop_motors_nolock_pleasework(void)
{
	move_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0, 0);
	move_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1, 0);
}

static void stop_motors_now(void)
{
	xSemaphoreTake(motor_mutex, portMAX_DELAY);
	stop_motors_nolock_pleasework();
	xSemaphoreGive(motor_mutex);
}

static bool change_state(robot_state_t expected, robot_state_t next)
{
	xSemaphoreTake(motor_mutex, portMAX_DELAY);
	bool transitioned = robot_state_now == expected && !edge_stopped_pleasework;
	if (transitioned) {
		robot_state_now = next;
	}
	xSemaphoreGive(motor_mutex);
	return transitioned;
}

static void clear_target_stuff_pleasework(void)
{
	portENTER_CRITICAL(&target_lock_pleasework);
	target_visible = false;
	target_range_left_mm = UINT16_MAX;
	target_range_right_mm = UINT16_MAX;
	target_range_left_side_mm = UINT16_MAX;
	target_range_right_side_mm = UINT16_MAX;
	portEXIT_CRITICAL(&target_lock_pleasework);
}

static float how_close(uint16_t range_mm)
{
	if (range_mm == UINT16_MAX || range_mm >= TARGET_DISTANCE_MM) {
		return 0.0f;
	}
	return fminf(1.0f, (float)(TARGET_DISTANCE_MM - range_mm) /
		(float)(TARGET_DISTANCE_MM - SENSOR_LED_DISTANCE_MM));
}

static void drive_internal(float forward, float turn, int16_t *out_a,
	int16_t *out_b, bool edge_recovery_command)
{
	xSemaphoreTake(motor_mutex, portMAX_DELAY);
	if (edge_stopped_pleasework &&
		(!edge_recovery_command || robot_state_now != ROBOT_EDGE_RECOVERY)) {
		*out_a = 0;
		*out_b = 0;
		stop_motors_nolock_pleasework();
		xSemaphoreGive(motor_mutex);
		return;
	}

	float a = forward - turn;
	float b = -forward - turn;

	float magnitude = fmaxf(fabsf(a), fabsf(b));
	if (magnitude > MOTOR_MAX_OUTPUT) {
		float scale = MOTOR_MAX_OUTPUT / magnitude;
		a *= scale;
		b *= scale;
	}

	*out_a = (int16_t)a;
	*out_b = (int16_t)b;
	move_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0, *out_a);
	move_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1, *out_b);
	xSemaphoreGive(motor_mutex);
}

static void drive_mix_pleasework(float forward, float turn, int16_t *out_a, int16_t *out_b)
{
	drive_internal(forward, turn, out_a, out_b, false);
}

static void drive_edge_escape(float forward, float turn, int16_t *out_a,
	int16_t *out_b)
{
	drive_internal(forward, turn, out_a, out_b, true);
}

static void setup_motor_pins(void)
{
	const gpio_config_t motor_gpio_config = {
		.pin_bit_mask = (1ULL << MOTOR_A1_GPIO) | (1ULL << MOTOR_A2_GPIO) |
			(1ULL << MOTOR_B1_GPIO) | (1ULL << MOTOR_B2_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	gpio_config(&motor_gpio_config);

	const ledc_timer_config_t pwm_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = MOTOR_PWM_RESOLUTION,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = MOTOR_PWM_FREQUENCY_HZ,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwm_timer);

	const ledc_channel_config_t pwm_channels[] = {
		{.gpio_num = MOTOR_A_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
		 .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
		{.gpio_num = MOTOR_B_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1,
		 .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0}
	};
	ledc_channel_config(&pwm_channels[0]);
	ledc_channel_config(&pwm_channels[1]);
	stop_motors_now();
}

static bool setup_i2c_pleasework(i2c_master_bus_handle_t *bus)
{
	const i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = I2C_SDA_GPIO,
		.scl_io_num = I2C_SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true
	};
	return i2c_new_master_bus(&bus_config, bus) == ESP_OK;
}

static void setup_edge_pin(void)
{
	const gpio_config_t sensor_config = {
		.pin_bit_mask = 1ULL << LEFT_EDGE_SENSOR_GPIO,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};

	ESP_ERROR_CHECK(gpio_config(&sensor_config));
	ESP_LOGI(TAG, "left edge sensor configured on GPIO %d (white=LOW)",
		LEFT_EDGE_SENSOR_GPIO);
}

static void edge_task(void *arg)
{
	(void)arg;
	while (1) {
		int level = gpio_get_level(LEFT_EDGE_SENSOR_GPIO);
		TickType_t now = xTaskGetTickCount();
		edge_surface_is_white = level == EDGE_SENSOR_WHITE_LEVEL;
		if (edge_surface_is_white) {
			edge_surface_clear_since_ticks = 0;
		} else if (edge_surface_clear_since_ticks == 0) {
			edge_surface_clear_since_ticks = now;
		}

		if (level == EDGE_SENSOR_WHITE_LEVEL) {
			xSemaphoreTake(motor_mutex, portMAX_DELAY);
			if (!edge_stopped_pleasework && robot_state_now == ROBOT_WAITING) {
				edge_stopped_pleasework = true;
				edge_detection_armed = false;
				robot_state_now = ROBOT_STOPPED;
				ESP_LOGW(TAG, "white edge detected during countdown; stopping the run");
				stop_motors_nolock_pleasework();
			} else if (edge_detection_armed &&
				(robot_state_now == ROBOT_SEARCHING || robot_state_now == ROBOT_APPROACHING ||
				 robot_state_now == ROBOT_ATTACKING || robot_state_now == ROBOT_PUSHING ||
				 robot_state_now == ROBOT_ALIGNING || robot_state_now == ROBOT_WOODPECKER ||
				 robot_state_now == ROBOT_STARTUP_WAIT ||
				 robot_state_now == ROBOT_DODGE_TURN ||
				 robot_state_now == ROBOT_DODGE_STRAIGHT ||
				 robot_state_now == ROBOT_FLANKING)) {
				edge_stopped_pleasework = true;
				edge_detection_armed = false;
				robot_state_now = ROBOT_EDGE_RECOVERY;
				edge_recovery_detected_ticks = now;
				clear_target_stuff_pleasework();
				move_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0,
					-EDGE_REVERSE_SPEED);
				move_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1,
					EDGE_REVERSE_SPEED);
				ESP_LOGW(TAG, "white edge detected (GPIO %d); immediately reversing",
					LEFT_EDGE_SENSOR_GPIO);
			}
			xSemaphoreGive(motor_mutex);
		} else {
			xSemaphoreTake(motor_mutex, portMAX_DELAY);
			if (robot_state_now != ROBOT_EDGE_RECOVERY) {
				edge_detection_armed = true;
			}
			xSemaphoreGive(motor_mutex);
		}

		vTaskDelay(pdMS_TO_TICKS(EDGE_SENSOR_POLL_INTERVAL_MS));
	}
}

static void setup_xshut_pins_pleasework(void)
{
	const gpio_config_t xshut_config = {
		.pin_bit_mask = (1ULL << RIGHT_SENSOR_XSHUT_GPIO) | (1ULL << LEFT_SENSOR_XSHUT_GPIO) |
			(1ULL << LEFT_SIDE_SENSOR_XSHUT_GPIO) | (1ULL << RIGHT_SIDE_SENSOR_XSHUT_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	gpio_config(&xshut_config);
	gpio_set_level(RIGHT_SENSOR_XSHUT_GPIO, 0);
	gpio_set_level(LEFT_SENSOR_XSHUT_GPIO, 0);
	gpio_set_level(LEFT_SIDE_SENSOR_XSHUT_GPIO, 0);
	gpio_set_level(RIGHT_SIDE_SENSOR_XSHUT_GPIO, 0);
}

static void distance_task(void *arg)
{
	(void)arg;
	bool warned_left_read = false;
	bool warned_right_read = false;
	bool warned_left_side_read = false;
	bool warned_right_side_read = false;

	while (1) {
		uint16_t left_mm = UINT16_MAX;
		uint16_t right_mm = UINT16_MAX;
		uint16_t left_side_mm = UINT16_MAX;
		uint16_t right_side_mm = UINT16_MAX;
		bool left_read_ok = left_sensor_initialized &&
			vl53l0x_get_distance_pleasework(&left_sensor, &left_mm);
		bool right_read_ok = right_sensor_initialized &&
			vl53l0x_get_distance_pleasework(&right_sensor, &right_mm);
		bool left_side_read_ok = left_side_sensor_initialized &&
			vl53l0x_get_distance_pleasework(&left_side_sensor, &left_side_mm);
		bool right_side_read_ok = right_side_sensor_initialized &&
			vl53l0x_get_distance_pleasework(&right_side_sensor, &right_side_mm);

		if (left_sensor_initialized && !left_read_ok && !warned_left_read) {
			ESP_LOGW(TAG, "left VL53L0X range read failed");
			warned_left_read = true;
		} else if (left_read_ok) {
			warned_left_read = false;
		}
		if (right_sensor_initialized && !right_read_ok && !warned_right_read) {
			ESP_LOGW(TAG, "right VL53L0X range read failed");
			warned_right_read = true;
		} else if (right_read_ok) {
			warned_right_read = false;
		}
		if (left_side_sensor_initialized && !left_side_read_ok && !warned_left_side_read) {
			ESP_LOGW(TAG, "left side VL53L0X range read failed");
			warned_left_side_read = true;
		} else if (left_side_read_ok) {
			warned_left_side_read = false;
		}
		if (right_side_sensor_initialized && !right_side_read_ok && !warned_right_side_read) {
			ESP_LOGW(TAG, "right side VL53L0X range read failed");
			warned_right_side_read = true;
		} else if (right_side_read_ok) {
			warned_right_side_read = false;
		}

		bool left_near = left_read_ok && left_mm <= SENSOR_LED_DISTANCE_MM;
		bool right_near = right_read_ok && right_mm <= SENSOR_LED_DISTANCE_MM;
		bool left_side_near = left_side_read_ok &&
			left_side_mm <= SENSOR_LED_DISTANCE_MM;
		bool right_side_near = right_side_read_ok &&
			right_side_mm <= SENSOR_LED_DISTANCE_MM;
		bool left_visible = left_read_ok && left_mm <= TARGET_DISTANCE_MM;
		bool right_visible = right_read_ok && right_mm <= TARGET_DISTANCE_MM;
		bool left_side_visible = left_side_read_ok &&
			left_side_mm <= TARGET_DISTANCE_MM;
		bool right_side_visible = right_side_read_ok &&
			right_side_mm <= TARGET_DISTANCE_MM;
		bool suppress_target_detection = edge_surface_is_white ||
			robot_state_now == ROBOT_EDGE_RECOVERY;
		if (suppress_target_detection) {
			left_visible = false;
			right_visible = false;
			left_side_visible = false;
			right_side_visible = false;
		}

		portENTER_CRITICAL(&target_lock_pleasework);
		target_visible = left_visible || right_visible || left_side_visible || right_side_visible;
		target_range_left_mm = left_visible ? left_mm : UINT16_MAX;
		target_range_right_mm = right_visible ? right_mm : UINT16_MAX;
		target_range_left_side_mm = left_side_visible ? left_side_mm : UINT16_MAX;
		target_range_right_side_mm = right_side_visible ? right_side_mm : UINT16_MAX;
		target_left_near = left_near;
		target_right_near = right_near;
		target_left_side_near = left_side_near;
		target_right_side_near = right_side_near;
		portEXIT_CRITICAL(&target_lock_pleasework);

		vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
	}
}

static bool led_blink_logic(uint8_t active_sensors, TickType_t now)
{
	if (active_sensors == 0x01) {
		return true;
	}
	if (active_sensors == 0x02) {
		return now % pdMS_TO_TICKS(1000) < pdMS_TO_TICKS(500);
	}
	if (active_sensors == 0x04) {
		return now % pdMS_TO_TICKS(500) < pdMS_TO_TICKS(250);
	}
	if (active_sensors == 0x08) {
		return now % pdMS_TO_TICKS(200) < pdMS_TO_TICKS(100);
	}
	if (active_sensors == 0) {
		return false;
	}

	TickType_t pulse_slot = pdMS_TO_TICKS(200);
	TickType_t cycle = pdMS_TO_TICKS(5000);
	TickType_t position = now % cycle;
	return position < (TickType_t)active_sensors * pulse_slot &&
		position % pulse_slot < pdMS_TO_TICKS(100);
}

static void led_task_pleasework(void *arg)
{
	(void)arg;

	while (1) {
		TickType_t now = xTaskGetTickCount();
		robot_state_t state = robot_state_now;

		if (state == ROBOT_IDLE) {
			if (!imu_check_complete) {
				gpio_set_level(LED_GPIO, 0);
			} else {
				bool left_near;
				bool right_near;
				bool left_side_near;
				bool right_side_near;
				portENTER_CRITICAL(&target_lock_pleasework);
				left_near = target_left_near;
				right_near = target_right_near;
				left_side_near = target_left_side_near;
				right_side_near = target_right_side_near;
				portEXIT_CRITICAL(&target_lock_pleasework);

				uint32_t phase = now / pdMS_TO_TICKS(100);
				uint32_t fault_phase = phase % 15;
				uint8_t active_sensors = (left_side_near ? 0x01 : 0) |
					(left_near ? 0x02 : 0) |
					(right_near ? 0x04 : 0) |
					(right_side_near ? 0x08 : 0);
				if (active_sensors != 0) {
					gpio_set_level(LED_GPIO, led_blink_logic(active_sensors, now));
				} else {
					bool fault_flash = !imu_ok ?
						(fault_phase == 0 || fault_phase == 2) :
						(imu_drdy_fault &&
						 (fault_phase == 0 || fault_phase == 2 || fault_phase == 4));
					gpio_set_level(LED_GPIO, fault_flash ? 1 : 0);
				}
			}
		} else if (state == ROBOT_WAITING) {
			gpio_set_level(LED_GPIO,
				now - start_time_pleasework >= pdMS_TO_TICKS(LED_DELAY_MS));
		} else if (state == ROBOT_SEARCHING || state == ROBOT_APPROACHING ||
			state == ROBOT_ATTACKING || state == ROBOT_PUSHING ||
			state == ROBOT_ALIGNING || state == ROBOT_EDGE_RECOVERY ||
			state == ROBOT_WOODPECKER || state == ROBOT_STARTUP_WAIT ||
			state == ROBOT_DODGE_TURN || state == ROBOT_DODGE_STRAIGHT ||
			state == ROBOT_FLANKING) {
			if (!imu_ok || imu_drdy_fault) {
				uint32_t phase = (now / pdMS_TO_TICKS(100)) % 15;
				bool fault_flash = !imu_ok ?
					(phase == 0 || phase == 2) :
					(phase == 0 || phase == 2 || phase == 4);
				gpio_set_level(LED_GPIO, fault_flash ? 1 : 0);
			} else {
				gpio_set_level(LED_GPIO, 1);
			}
		} else {
			gpio_set_level(LED_GPIO, 0);
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

static void main_control_task(void *arg)
{
	(void)arg;
	int64_t run_start_us = 0;
	bool warned_about_drdy = false;
	bool use_drdy = imu_ok;
	float old_accel_pleasework = 0.0f;
	bool have_previous_accel = false;
	TickType_t attack_started_ticks = 0;
	TickType_t contact_started_ticks = 0;
	TickType_t push_progress_since_ticks = 0;
	TickType_t edge_reverse_started_ticks = 0;
	TickType_t edge_turn_started_ticks = 0;
	TickType_t edge_turn_stable_since = 0;
	TickType_t woodpecker_phase_started_ticks = 0;
	TickType_t align_lost_ticks = 0;
	TickType_t front_aligned_since = 0;
	float align_turn_amount_pleasework = 0.0f;
	float edge_turn_target_deg = 0.0f;
	float edge_turn_output = 0.0f;
	bool edge_recovery_started = false;
	bool edge_turn_started = false;
	bool peck_forward = true;
	uint16_t push_range_pleasework = UINT16_MAX;
	TickType_t startup_wait_started_ticks = 0;
	TickType_t startup_previous_sample_ticks = 0;
	TickType_t startup_fast_approach_samples = 0;
	TickType_t flank_phase_started_ticks = 0;
	TickType_t flank_turn_stable_since = 0;
	bool startup_wait_initialized = false;
	bool flank_target_lost = false;
	uint16_t startup_previous_front_mm = UINT16_MAX;
	float flank_turn_target_deg = 0.0f;
	float flank_heading_target_deg = 0.0f;
	float flank_turn_out = 0.0f;

	while (1) {
		imu_sample_t sample;
		if (imu_ok) {
			if (!use_drdy) {
				vTaskDelay(pdMS_TO_TICKS(10));
			}

			bool drdy_received = use_drdy &&
				imu_wait_for_data(&imu, CONTROL_LOOP_TIMEOUT_MS);
			if (!drdy_received && !warned_about_drdy) {
				ESP_LOGW(TAG, "IMU data-ready timeout; falling back to timed reads - check INT1 on GPIO %d",
					IMU_INT1_GPIO);
				imu_drdy_fault = true;
				warned_about_drdy = true;
				use_drdy = false;
			}
			if (!imu_get_stuff_pleasework(&imu, &sample)) {
				continue;
			}
			warned_about_drdy = false;
		} else {

			sample.yaw_deg = 0.0f;
			sample.yaw_rate_dps = 0.0f;
			sample.accel_fwd_g = 0.0f;
			sample.accel_lat_g = 0.0f;
			sample.dt_s = (float)CONTROL_LOOP_TIMEOUT_MS / 1000.0f;
			vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_TIMEOUT_MS));
		}

		robot_state_t state = robot_state_now;
		if (state != ROBOT_SEARCHING && state != ROBOT_APPROACHING &&
			state != ROBOT_ATTACKING && state != ROBOT_PUSHING &&
			state != ROBOT_ALIGNING && state != ROBOT_EDGE_RECOVERY &&
			state != ROBOT_WOODPECKER && state != ROBOT_STARTUP_WAIT &&
			state != ROBOT_DODGE_TURN && state != ROBOT_DODGE_STRAIGHT &&
			state != ROBOT_FLANKING) {
			run_start_us = 0;
			edge_recovery_started = false;
			edge_turn_started = false;
			startup_wait_initialized = false;
			flank_target_lost = false;
			continue;
		}

		TickType_t now_ticks = xTaskGetTickCount();
		int64_t now_us = esp_timer_get_time();
		if (run_start_us == 0) {
			run_start_us = now_us;
		}

		bool visible;
		uint16_t range_left;
		uint16_t range_right;
		uint16_t range_left_side;
		uint16_t range_right_side;
		portENTER_CRITICAL(&target_lock_pleasework);
		visible = target_visible;
		range_left = target_range_left_mm;
		range_right = target_range_right_mm;
		range_left_side = target_range_left_side_mm;
		range_right_side = target_range_right_side_mm;
		portEXIT_CRITICAL(&target_lock_pleasework);

		float front_left = how_close(range_left);
		float front_right = how_close(range_right);
		float side_left = how_close(range_left_side);
		float side_right = how_close(range_right_side);
		float left_detection = (front_left + side_left) * 0.5f;
		float right_detection = (front_right + side_right) * 0.5f;
		float direction = 0.0f;
		float detection_total = left_detection + right_detection;
		if (detection_total > 0.0f) {
			direction = (left_detection - right_detection) / detection_total;
		}
		float best_detection = fmaxf(fmaxf(front_left, front_right),
			fmaxf(side_left, side_right));
		float front_strength = fmaxf(front_left, front_right);
		uint16_t nearest_front_mm = range_left < range_right ? range_left : range_right;
		bool front_visible = range_left != UINT16_MAX || range_right != UINT16_MAX;
		bool both_front_visible = range_left != UINT16_MAX &&
			range_right != UINT16_MAX;
		bool side_visible = range_left_side != UINT16_MAX ||
			range_right_side != UINT16_MAX;
		bool front_centered_pleasework = both_front_visible &&
			fabsf(front_left - front_right) <= TARGET_ALIGN_MAX_FRONT_DIFFERENCE &&
			front_strength >= TARGET_ALIGN_MIN_FRONT_STRENGTH;
		bool edge_surface_clear = !edge_surface_is_white &&
			edge_surface_clear_since_ticks != 0 &&
			now_ticks - edge_surface_clear_since_ticks >=
				pdMS_TO_TICKS(EDGE_SURFACE_CLEAR_STABLE_MS);

		if (state == ROBOT_STARTUP_WAIT) {
			if (!startup_wait_initialized) {
				startup_wait_initialized = true;
				startup_wait_started_ticks = now_ticks;
				startup_previous_sample_ticks = now_ticks;
				startup_previous_front_mm = nearest_front_mm;
				startup_fast_approach_samples = 0;
				ESP_LOGI(TAG, "opponent detected ahead at start; waiting 1 second");
			} else if (nearest_front_mm != UINT16_MAX) {
				if (startup_previous_front_mm != UINT16_MAX &&
					now_ticks > startup_previous_sample_ticks) {
					if (startup_previous_front_mm > nearest_front_mm) {
						float closing_speed_pleasework =
							(float)(startup_previous_front_mm - nearest_front_mm) *
							(float)configTICK_RATE_HZ /
							(float)(now_ticks - startup_previous_sample_ticks);
						if (closing_speed_pleasework >= FLANK_CLOSING_RATE_MM_S) {
							startup_fast_approach_samples++;
						} else {
							startup_fast_approach_samples = 0;
						}
					} else {
						startup_fast_approach_samples = 0;
					}
				}
				startup_previous_front_mm = nearest_front_mm;
				startup_previous_sample_ticks = now_ticks;
			} else {
				startup_previous_front_mm = UINT16_MAX;
				startup_previous_sample_ticks = now_ticks;
				startup_fast_approach_samples = 0;
			}

			if (now_ticks - startup_wait_started_ticks >=
				pdMS_TO_TICKS(STARTUP_TARGET_WAIT_MS)) {
				if (startup_fast_approach_samples >= FLANK_REQUIRED_FAST_SAMPLES &&
					change_state(ROBOT_STARTUP_WAIT, ROBOT_DODGE_TURN)) {
					state = ROBOT_DODGE_TURN;
					flank_phase_started_ticks = now_ticks;
					flank_turn_stable_since = 0;
					flank_turn_target_deg = imu_fix_angle_pleasework(
						sample.yaw_deg - FLANK_DODGE_TURN_DEG);
					pid_clear_pleasework(&my_pid);
					ESP_LOGI(TAG, "fast-closing opponent; dodging right by %.0f degrees",
						FLANK_DODGE_TURN_DEG);
				} else {
					robot_state_t next_state = visible ?
						ROBOT_ALIGNING : ROBOT_SEARCHING;
					if (change_state(ROBOT_STARTUP_WAIT, next_state)) {
						state = next_state;
						if (state == ROBOT_ALIGNING) {
							align_lost_ticks = now_ticks;
							front_aligned_since = 0;
							align_turn_amount_pleasework = direction < 0.0f ?
								-TARGET_ALIGN_TURN_OUTPUT :
								TARGET_ALIGN_TURN_OUTPUT;
						}
						ESP_LOGI(TAG, "startup wait complete; resuming normal target behavior");
					}
				}
			}
		}

		if (state == ROBOT_DODGE_TURN) {
			bool turn_done = false;
			if (imu_ok) {
				float turn_error = imu_fix_angle_pleasework(flank_turn_target_deg -
					sample.yaw_deg);
				flank_turn_out = pid_calc(&my_pid, turn_error,
					sample.yaw_rate_dps, sample.dt_s);
				if (fabsf(turn_error) <= EDGE_TURN_TOLERANCE_DEG &&
					fabsf(sample.yaw_rate_dps) <= EDGE_TURN_MAX_RATE_DPS) {
					if (flank_turn_stable_since == 0) {
						flank_turn_stable_since = now_ticks;
					} else if (now_ticks - flank_turn_stable_since >=
						pdMS_TO_TICKS(EDGE_TURN_STABLE_MS)) {
						turn_done = true;
					}
				} else {
					flank_turn_stable_since = 0;
				}
			} else {
				flank_turn_out = -FLANK_TURN_OUTPUT;
				turn_done = now_ticks - flank_phase_started_ticks >=
					pdMS_TO_TICKS(FLANK_TURN_FALLBACK_DURATION_MS);
			}

			if (turn_done &&
				change_state(ROBOT_DODGE_TURN, ROBOT_DODGE_STRAIGHT)) {
				state = ROBOT_DODGE_STRAIGHT;
				flank_phase_started_ticks = now_ticks;
				flank_heading_target_deg = sample.yaw_deg;
				pid_clear_pleasework(&my_pid);
				ESP_LOGI(TAG, "right dodge turn complete; moving straight before flanking");
			} else if (now_ticks - flank_phase_started_ticks >=
				pdMS_TO_TICKS(FLANK_TURN_TIMEOUT_MS)) {
				robot_state_t next_state = visible ? ROBOT_ALIGNING : ROBOT_SEARCHING;
				if (change_state(ROBOT_DODGE_TURN, next_state)) {
					state = next_state;
					if (state == ROBOT_ALIGNING) {
						align_lost_ticks = now_ticks;
						front_aligned_since = 0;
						align_turn_amount_pleasework = direction < 0.0f ?
							-TARGET_ALIGN_TURN_OUTPUT :
							TARGET_ALIGN_TURN_OUTPUT;
					}
					ESP_LOGW(TAG, "right dodge turn timed out; returning to normal target behavior");
				}
			}
		}

		if (state == ROBOT_DODGE_STRAIGHT &&
			now_ticks - flank_phase_started_ticks >=
				pdMS_TO_TICKS(FLANK_STRAIGHT_DURATION_MS) &&
			change_state(ROBOT_DODGE_STRAIGHT, ROBOT_FLANKING)) {
			state = ROBOT_FLANKING;
			flank_phase_started_ticks = now_ticks;
			flank_target_lost = !visible;
			ESP_LOGI(TAG, "straight segment complete; beginning right-flank arc");
		}

		if (state == ROBOT_FLANKING) {
			if (!visible) {
				flank_target_lost = true;
			}
			if (now_ticks - flank_phase_started_ticks >=
					pdMS_TO_TICKS(FLANK_ARC_MIN_DURATION_MS) &&
				(side_visible || (flank_target_lost && front_visible))) {
				if (change_state(ROBOT_FLANKING, ROBOT_ALIGNING)) {
					state = ROBOT_ALIGNING;
					align_lost_ticks = now_ticks;
					front_aligned_since = 0;
					align_turn_amount_pleasework = direction < 0.0f ?
						-TARGET_ALIGN_TURN_OUTPUT : TARGET_ALIGN_TURN_OUTPUT;
					ESP_LOGI(TAG, "target reacquired on flank; aligning to attack");
				}
			} else if (now_ticks - flank_phase_started_ticks >=
				pdMS_TO_TICKS(FLANK_ARC_TIMEOUT_MS) &&
				change_state(ROBOT_FLANKING, ROBOT_SEARCHING)) {
				state = ROBOT_SEARCHING;
				ESP_LOGI(TAG, "flank arc timed out without reacquisition; resuming search");
			}
		}

		if (state == ROBOT_EDGE_RECOVERY) {
			if (!edge_recovery_started) {
				edge_recovery_started = true;
				edge_reverse_started_ticks = edge_recovery_detected_ticks;
				edge_turn_target_deg = imu_fix_angle_pleasework(sample.yaw_deg + 180.0f);
				edge_turn_started = false;
				edge_turn_stable_since = 0;
				pid_clear_pleasework(&my_pid);
				ESP_LOGI(TAG, "edge recovery: reversing until the edge sensor clears");
			}

			if (edge_surface_is_white && edge_turn_started) {
				edge_turn_started = false;
				edge_reverse_started_ticks = now_ticks;
				edge_turn_stable_since = 0;
				ESP_LOGW(TAG, "edge detected during turn; backing away again");
			}

			if (now_ticks - edge_reverse_started_ticks >=
					pdMS_TO_TICKS(EDGE_REVERSE_DURATION_MS) &&
				edge_surface_clear) {
				if (!imu_ok) {
					if (!edge_turn_started) {
						edge_turn_started = true;
						edge_turn_started_ticks = now_ticks;
					}
					if (now_ticks - edge_turn_started_ticks >=
						pdMS_TO_TICKS(EDGE_TURN_FALLBACK_DURATION_MS)) {
						xSemaphoreTake(motor_mutex, portMAX_DELAY);
						if (robot_state_now == ROBOT_EDGE_RECOVERY) {
							robot_state_now = ROBOT_SEARCHING;
							edge_stopped_pleasework = false;
						}
						xSemaphoreGive(motor_mutex);
						clear_target_stuff_pleasework();
						edge_recovery_started = false;
						edge_turn_started = false;
						state = ROBOT_SEARCHING;
						ESP_LOGI(TAG, "timed edge turn complete; resuming search");
					}
				} else {
					if (!edge_turn_started) {
						edge_turn_started = true;
						edge_turn_started_ticks = now_ticks;
					}
					float turn_error = imu_fix_angle_pleasework(edge_turn_target_deg -
						sample.yaw_deg);
					edge_turn_output = pid_calc(&my_pid, turn_error,
						sample.yaw_rate_dps, sample.dt_s);

					if (fabsf(turn_error) <= EDGE_TURN_TOLERANCE_DEG &&
						fabsf(sample.yaw_rate_dps) <= EDGE_TURN_MAX_RATE_DPS) {
						if (edge_turn_stable_since == 0) {
							edge_turn_stable_since = now_ticks;
						} else if (edge_surface_clear &&
							now_ticks - edge_turn_stable_since >=
							pdMS_TO_TICKS(EDGE_TURN_STABLE_MS)) {
							xSemaphoreTake(motor_mutex, portMAX_DELAY);
							if (robot_state_now == ROBOT_EDGE_RECOVERY) {
								robot_state_now = ROBOT_SEARCHING;
								edge_stopped_pleasework = false;
							}
							xSemaphoreGive(motor_mutex);
							clear_target_stuff_pleasework();
							edge_recovery_started = false;
							edge_turn_started = false;
							state = ROBOT_SEARCHING;
							ESP_LOGI(TAG, "180-degree edge turn complete; resuming search");
						}
					} else {
						edge_turn_stable_since = 0;
					}

					if (state == ROBOT_EDGE_RECOVERY &&
						now_ticks - edge_turn_started_ticks >=
						pdMS_TO_TICKS(EDGE_TURN_TIMEOUT_MS)) {
						ESP_LOGE(TAG, "edge recovery turn timed out; stopping for safety");
						xSemaphoreTake(motor_mutex, portMAX_DELAY);
						robot_state_now = ROBOT_STOPPED;
						stop_motors_nolock_pleasework();
						xSemaphoreGive(motor_mutex);
						continue;
					}
				}
			} else if (now_ticks - edge_reverse_started_ticks >=
				pdMS_TO_TICKS(EDGE_RETREAT_TIMEOUT_MS)) {
				ESP_LOGE(TAG, "edge sensor did not clear during retreat; stopping for safety");
				xSemaphoreTake(motor_mutex, portMAX_DELAY);
				robot_state_now = ROBOT_STOPPED;
				stop_motors_nolock_pleasework();
				xSemaphoreGive(motor_mutex);
				continue;
			}
		}

		if (state == ROBOT_SEARCHING && visible) {
			if (change_state(ROBOT_SEARCHING, ROBOT_ALIGNING)) {
				state = ROBOT_ALIGNING;
				align_lost_ticks = now_ticks;
				front_aligned_since = 0;
				align_turn_amount_pleasework = direction < 0.0f ? -TARGET_ALIGN_TURN_OUTPUT :
					TARGET_ALIGN_TURN_OUTPUT;
			}
		}

		if (state == ROBOT_ALIGNING) {
			if (front_centered_pleasework) {
				align_turn_amount_pleasework = 0.0f;
				if (front_aligned_since == 0) {
					front_aligned_since = now_ticks;
				}
				if (now_ticks - front_aligned_since >=
					pdMS_TO_TICKS(TARGET_ALIGN_STABLE_MS) &&
					change_state(ROBOT_ALIGNING, ROBOT_APPROACHING)) {
					state = ROBOT_APPROACHING;
				}
			} else if (front_visible || side_visible) {
				front_aligned_since = 0;
				align_lost_ticks = now_ticks;
				float turn_direction = front_visible ?
					front_left - front_right : direction;
				if (fabsf(turn_direction) > 0.05f) {
					align_turn_amount_pleasework = fmaxf(-TARGET_ALIGN_TURN_OUTPUT,
						fminf(TARGET_ALIGN_TURN_OUTPUT,
							turn_direction * TARGET_ALIGN_TURN_OUTPUT));
				}
			} else if (now_ticks - align_lost_ticks >=
				pdMS_TO_TICKS(TARGET_ALIGN_LOST_TIMEOUT_MS)) {
				if (change_state(ROBOT_ALIGNING, ROBOT_SEARCHING)) {
					state = ROBOT_SEARCHING;
				}
			}
		}

		if (state == ROBOT_APPROACHING) {
			if (!front_visible) {
				if (change_state(ROBOT_APPROACHING,
					side_visible ? ROBOT_ALIGNING : ROBOT_SEARCHING)) {
					state = side_visible ? ROBOT_ALIGNING : ROBOT_SEARCHING;
					if (state == ROBOT_ALIGNING) {
						align_lost_ticks = now_ticks;
						front_aligned_since = 0;
						align_turn_amount_pleasework = direction < 0.0f ?
							-TARGET_ALIGN_TURN_OUTPUT : TARGET_ALIGN_TURN_OUTPUT;
					}
				}
			} else if (!visible) {
				if (change_state(ROBOT_APPROACHING, ROBOT_SEARCHING)) {
					state = ROBOT_SEARCHING;
				}
			} else if (nearest_front_mm <= ATTACK_DISTANCE_MM) {
				if (change_state(ROBOT_APPROACHING, ROBOT_ATTACKING)) {
					state = ROBOT_ATTACKING;
					attack_started_ticks = now_ticks;
				}
			}
		}

		if (state == ROBOT_ATTACKING) {
			if (nearest_front_mm > ATTACK_EXIT_DISTANCE_MM) {
				robot_state_t next_state = visible ? ROBOT_APPROACHING : ROBOT_SEARCHING;
				if (change_state(ROBOT_ATTACKING, next_state)) {
					state = next_state;
				}
			} else if (imu_ok && have_previous_accel &&
				now_ticks - attack_started_ticks >= pdMS_TO_TICKS(IMPACT_ARM_DELAY_MS) &&
				fabsf(sample.accel_fwd_g - old_accel_pleasework) >= IMPACT_ACCEL_DELTA_G) {
				if (change_state(ROBOT_ATTACKING, ROBOT_PUSHING)) {
					state = ROBOT_PUSHING;
					contact_started_ticks = now_ticks;
					push_progress_since_ticks = now_ticks;
					push_range_pleasework = nearest_front_mm;
					ESP_LOGI(TAG, "impact detected; switching to raw push drive");
				}
			} else if (now_ticks - attack_started_ticks >=
				pdMS_TO_TICKS(WOODPECKER_NO_IMPACT_TIMEOUT_MS)) {
				if (change_state(ROBOT_ATTACKING, ROBOT_WOODPECKER)) {
					state = ROBOT_WOODPECKER;
					peck_forward = true;
					woodpecker_phase_started_ticks = now_ticks;
					push_progress_since_ticks = now_ticks;
					push_range_pleasework = nearest_front_mm;
					ESP_LOGW(TAG, "no impact response in attack range; starting woodpecker pulses");
				}
			}
		}

		if (state == ROBOT_PUSHING) {
			if (nearest_front_mm != UINT16_MAX &&
				push_range_pleasework != UINT16_MAX &&
				nearest_front_mm > push_range_pleasework &&
				nearest_front_mm - push_range_pleasework >=
					WOODPECKER_RANGE_PROGRESS_MM) {
				push_range_pleasework = nearest_front_mm;
				push_progress_since_ticks = now_ticks;
			} else if (push_range_pleasework == UINT16_MAX &&
				nearest_front_mm != UINT16_MAX) {
				push_range_pleasework = nearest_front_mm;
				push_progress_since_ticks = now_ticks;
			}

			TickType_t push_elapsed = now_ticks - contact_started_ticks;
			TickType_t stalled_ticks = now_ticks - push_progress_since_ticks;
			if (!visible && push_elapsed >= pdMS_TO_TICKS(CONTACT_MIN_PUSH_MS)) {
				if (change_state(ROBOT_PUSHING, ROBOT_SEARCHING)) {
					state = ROBOT_SEARCHING;
				}
			} else if (visible &&
				stalled_ticks >= pdMS_TO_TICKS(WOODPECKER_STALL_TIMEOUT_MS)) {
				if (change_state(ROBOT_PUSHING, ROBOT_WOODPECKER)) {
					state = ROBOT_WOODPECKER;
					peck_forward = true;
					woodpecker_phase_started_ticks = now_ticks;
					ESP_LOGW(TAG, "no forward target progress after impact; starting woodpecker pulses");
				}
			}
		}

		if (state == ROBOT_WOODPECKER) {
			if (!visible) {
				if (change_state(ROBOT_WOODPECKER, ROBOT_SEARCHING)) {
					state = ROBOT_SEARCHING;
				}
			} else if (nearest_front_mm != UINT16_MAX &&
				push_range_pleasework != UINT16_MAX &&
				nearest_front_mm > push_range_pleasework &&
				nearest_front_mm - push_range_pleasework >=
					WOODPECKER_RANGE_PROGRESS_MM) {
				push_range_pleasework = nearest_front_mm;
				if (change_state(ROBOT_WOODPECKER, ROBOT_PUSHING)) {
					state = ROBOT_PUSHING;
					contact_started_ticks = now_ticks;
					push_progress_since_ticks = now_ticks;
					ESP_LOGI(TAG, "target moving outward; resuming continuous push");
				}
			}

			if (state == ROBOT_WOODPECKER) {
				TickType_t phase_duration = peck_forward ?
					pdMS_TO_TICKS(WOODPECKER_FORWARD_DURATION_MS) :
					pdMS_TO_TICKS(WOODPECKER_REVERSE_DURATION_MS);
				if (now_ticks - woodpecker_phase_started_ticks >= phase_duration) {
					peck_forward = !peck_forward;
					woodpecker_phase_started_ticks = now_ticks;
				}
			}
		}

		int16_t motor_a;
		int16_t motor_b;
		if (state == ROBOT_SEARCHING) {
			drive_mix_pleasework(0.0f, SEARCH_SPIN_OUTPUT, &motor_a, &motor_b);
		} else if (state == ROBOT_STARTUP_WAIT) {
			drive_mix_pleasework(0.0f, 0.0f, &motor_a, &motor_b);
		} else if (state == ROBOT_DODGE_TURN) {
			drive_mix_pleasework(0.0f, flank_turn_out, &motor_a, &motor_b);
		} else if (state == ROBOT_DODGE_STRAIGHT) {
			float turn = imu_ok ? pid_calc(&my_pid,
				imu_fix_angle_pleasework(flank_heading_target_deg - sample.yaw_deg),
				sample.yaw_rate_dps, sample.dt_s) : 0.0f;
			drive_mix_pleasework(FLANK_STRAIGHT_FORWARD_OUTPUT, turn, &motor_a, &motor_b);
		} else if (state == ROBOT_FLANKING) {
			drive_mix_pleasework(FLANK_ARC_FORWARD_OUTPUT, -FLANK_ARC_TURN_OUTPUT,
				&motor_a, &motor_b);
		} else if (state == ROBOT_EDGE_RECOVERY) {
			if (imu_ok && edge_turn_started &&
				now_ticks - edge_reverse_started_ticks >=
				pdMS_TO_TICKS(EDGE_REVERSE_DURATION_MS)) {
				drive_edge_escape(0.0f, edge_turn_output, &motor_a, &motor_b);
			} else if (!imu_ok && edge_turn_started) {
				drive_edge_escape(0.0f, EDGE_TURN_FALLBACK_OUTPUT,
					&motor_a, &motor_b);
			} else {
				drive_edge_escape(-EDGE_REVERSE_SPEED, 0.0f,
					&motor_a, &motor_b);
			}
		} else if (state == ROBOT_ALIGNING) {
			drive_mix_pleasework(0.0f, align_turn_amount_pleasework, &motor_a, &motor_b);
		} else if (state == ROBOT_WOODPECKER) {
			float forward = peck_forward ? MOTOR_MAX_OUTPUT :
				-WOODPECKER_REVERSE_OUTPUT;
			drive_mix_pleasework(forward, 0.0f, &motor_a, &motor_b);
		} else if (state == ROBOT_APPROACHING) {
			float forward = APPROACH_MIN_FORWARD_OUTPUT +
				best_detection * (APPROACH_MAX_FORWARD_OUTPUT -
					APPROACH_MIN_FORWARD_OUTPUT);
			float front_difference = front_left - front_right;
			float turn = front_difference * APPROACH_MAX_TURN_OUTPUT;
			drive_mix_pleasework(forward, turn, &motor_a, &motor_b);
		} else if (state == ROBOT_ATTACKING || state == ROBOT_PUSHING) {

			drive_mix_pleasework(MOTOR_MAX_OUTPUT, 0.0f, &motor_a, &motor_b);
		}
		old_accel_pleasework = sample.accel_fwd_g;
		have_previous_accel = true;

		uint8_t flags = visible ? TELEM_FLAG_TARGET_VISIBLE : 0;
		if (state == ROBOT_PUSHING || state == ROBOT_WOODPECKER) {
			flags |= TELEM_FLAG_CONTACT;
		}

		telem_record_t record = {
			.t_us = (uint32_t)(now_us - run_start_us),
			.yaw_ddeg = (int16_t)(sample.yaw_deg * TELEM_ANGLE_SCALE),
			.setpoint_ddeg = state == ROBOT_EDGE_RECOVERY ?
				(int16_t)(edge_turn_target_deg * TELEM_ANGLE_SCALE) :
				(state == ROBOT_DODGE_TURN ?
					(int16_t)(flank_turn_target_deg * TELEM_ANGLE_SCALE) :
					(state == ROBOT_DODGE_STRAIGHT ?
						(int16_t)(flank_heading_target_deg * TELEM_ANGLE_SCALE) : 0)),
			.yaw_rate_ddps = (int16_t)(sample.yaw_rate_dps * TELEM_RATE_SCALE),
			.accel_fwd_mg = (int16_t)(sample.accel_fwd_g * TELEM_ACCEL_SCALE),
			.accel_lat_mg = (int16_t)(sample.accel_lat_g * TELEM_ACCEL_SCALE),
			.p_term = (int16_t)(my_pid.p_term * TELEM_TERM_SCALE),
			.i_term = (int16_t)(my_pid.i_term * TELEM_TERM_SCALE),
			.d_term = (int16_t)(my_pid.d_term * TELEM_TERM_SCALE),
			.motor_a = motor_a,
			.motor_b = motor_b,
			.range_left_mm = range_left,
			.range_right_mm = range_right,
			.state = (uint8_t)state,
			.flags = flags,
			.reserved = 0
		};
		telemetry_save(&record);

		if (telemetry_full_pleasework() || now_us - run_start_us > (int64_t)MATCH_TIMEOUT_MS * 1000) {
			ESP_LOGI(TAG, "run complete (%s)",
				telemetry_full_pleasework() ? "buffer full" : "match timeout");
			robot_state_now = ROBOT_STOPPED;
		}
	}
}

void app_main(void)
{
	i2c_master_bus_handle_t sensor_bus;
	bool calibrated_pleasework = false;
	bool button_before = false;

	gpio_reset_pin(LED_GPIO);
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_GPIO, 0);
	gpio_reset_pin(BUTTON_GPIO);
	gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLDOWN_ONLY);
	motor_mutex = xSemaphoreCreateMutex();
	if (motor_mutex == NULL) {
		ESP_LOGE(TAG, "failed to create motor command mutex");
		return;
	}
	if (xTaskCreatePinnedToCore(led_task_pleasework, "status_led", 2048, NULL, 5, NULL, 0) != pdPASS) {
		ESP_LOGE(TAG, "failed to start status LED task");
	}
	setup_motor_pins();
	setup_edge_pin();

	pid_setup(&my_pid, PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT,
		PID_OUTPUT_LIMIT);
	telemetry_setup();

	setup_xshut_pins_pleasework();
	if (setup_i2c_pleasework(&sensor_bus)) {
		imu_ok = imu_setup(&imu, sensor_bus, LSM6DS3_ADDR_LOW);
		if (!imu_ok) {
			ESP_LOGE(TAG, "LSM6DS3 not found at 0x%02X or 0x%02X - check CS is tied high "
				"(I2C mode) and verify SA0 wiring", LSM6DS3_ADDR_LOW, LSM6DS3_ADDR_HIGH);
		} else if (!imu_start_interrupt_pleasework(&imu, IMU_INT1_GPIO)) {
			ESP_LOGE(TAG, "INT1 setup failed on GPIO %d", IMU_INT1_GPIO);
			imu_ok = false;
		} else {
			ESP_LOGI(TAG, "LSM6DS3 ready, data-ready on GPIO %d", IMU_INT1_GPIO);
			imu_reset_angle(&imu);
			if (imu_zero_the_accel(&imu, 20)) {
				ESP_LOGI(TAG, "startup orientation set as acceleration zero");
			} else {
				ESP_LOGW(TAG, "acceleration zero calibration failed");
			}
		}
		imu_check_complete = true;

		gpio_set_level(LEFT_SIDE_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		left_side_sensor_initialized = vl53l0x_setup(&left_side_sensor, sensor_bus,
			LEFT_SIDE_SENSOR_ADDRESS);
		if (!left_side_sensor_initialized) {
			ESP_LOGE(TAG, "left side VL53L0X initialization failed on GPIO %d",
				LEFT_SIDE_SENSOR_XSHUT_GPIO);
			gpio_set_level(LEFT_SIDE_SENSOR_XSHUT_GPIO, 0);
		}

		gpio_set_level(RIGHT_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		right_sensor_initialized = vl53l0x_setup(&right_sensor, sensor_bus, RIGHT_SENSOR_ADDRESS);
		if (!right_sensor_initialized) {
			ESP_LOGE(TAG, "right VL53L0X initialization failed on GPIO %d",
				RIGHT_SENSOR_XSHUT_GPIO);
			gpio_set_level(RIGHT_SENSOR_XSHUT_GPIO, 0);
		}

		gpio_set_level(LEFT_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		left_sensor_initialized = vl53l0x_setup(&left_sensor, sensor_bus, LEFT_SENSOR_ADDRESS);
		if (!left_sensor_initialized) {
			ESP_LOGE(TAG, "left VL53L0X initialization failed on GPIO %d",
				LEFT_SENSOR_XSHUT_GPIO);
			gpio_set_level(LEFT_SENSOR_XSHUT_GPIO, 0);
		}
		gpio_set_level(RIGHT_SIDE_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		right_side_sensor_initialized = vl53l0x_setup(&right_side_sensor, sensor_bus,
			RIGHT_SIDE_SENSOR_ADDRESS);
		if (!right_side_sensor_initialized) {
			ESP_LOGE(TAG, "right side VL53L0X initialization failed on GPIO %d",
				RIGHT_SIDE_SENSOR_XSHUT_GPIO);
			gpio_set_level(RIGHT_SIDE_SENSOR_XSHUT_GPIO, 0);
		}
		ESP_LOGI(TAG, "VL53L0X initialization: left=%s, right=%s, left side=%s, right side=%s",
			left_sensor_initialized ? "OK" : "FAILED",
			right_sensor_initialized ? "OK" : "FAILED",
			left_side_sensor_initialized ? "OK" : "FAILED",
			right_side_sensor_initialized ? "OK" : "FAILED");
	} else {
		ESP_LOGE(TAG, "I2C bus init failed");
		imu_check_complete = true;
	}

	if (!imu_ok) {
		ESP_LOGW(TAG, "no IMU - 90-degree turn runs are disabled");
	}

	if (xTaskCreatePinnedToCore(edge_task, "edge_sensor", 2048, NULL, 7, NULL, 0) != pdPASS) {
		ESP_LOGE(TAG, "failed to start edge sensor task; robot start is disabled");
		return;
	}

	if (left_sensor_initialized || right_sensor_initialized ||
		left_side_sensor_initialized || right_side_sensor_initialized) {
		if (xTaskCreatePinnedToCore(distance_task, "sensors", 4096, NULL, 5, NULL, 0) != pdPASS) {
			ESP_LOGE(TAG, "failed to start VL53L0X sensor task");
		}
	}
	xTaskCreatePinnedToCore(main_control_task, "control", 4096, NULL, 6, NULL, 1);
	ESP_LOGI(TAG, "motion control task started");

	while (1) {
		TickType_t now = xTaskGetTickCount();
		robot_state_t state = robot_state_now;
		bool button_pressed = gpio_get_level(BUTTON_GPIO) != 0;
		bool button_pressed_event = button_pressed && !button_before;
		button_before = button_pressed;

		if (state == ROBOT_IDLE && button_pressed_event) {
			start_time_pleasework = now;
			calibrated_pleasework = false;
			xSemaphoreTake(motor_mutex, portMAX_DELAY);
			edge_stopped_pleasework = false;
			edge_detection_armed = true;
			robot_state_now = ROBOT_WAITING;
			stop_motors_nolock_pleasework();
			xSemaphoreGive(motor_mutex);
			pid_clear_pleasework(&my_pid);
			telemetry_start_run_pleasework(0.0f, 0.0f, 0.0f, 0.0f,
				SEARCH_SPIN_OUTPUT, MOTOR_MAX_OUTPUT, TARGET_DISTANCE_MM);
		}

		if (state == ROBOT_WAITING && !calibrated_pleasework) {
			calibrated_pleasework = true;
			if (!imu_ok) {
				ESP_LOGW(TAG, "skipping gyro calibration: IMU unavailable");
			} else if (imu_fix_drift_pleasework(&imu, 300)) {
				ESP_LOGI(TAG, "gyro bias %.3f deg/s", imu.yaw_bias_dps);
			} else {
				ESP_LOGW(TAG, "bias calibration failed - heading may drift");
			}
		}

		if (state == ROBOT_WAITING && now - start_time_pleasework >= pdMS_TO_TICKS(MOTOR_START_DELAY_MS)) {
			if (imu_ok) {
				imu_reset_angle(&imu);
			}
			pid_clear_pleasework(&my_pid);
			xSemaphoreTake(motor_mutex, portMAX_DELAY);
			bool front_target_detected;
			portENTER_CRITICAL(&target_lock_pleasework);
			front_target_detected = target_range_left_mm != UINT16_MAX ||
				target_range_right_mm != UINT16_MAX;
			portEXIT_CRITICAL(&target_lock_pleasework);
			bool start_run = robot_state_now == ROBOT_WAITING && !edge_stopped_pleasework;
			if (start_run) {
				robot_state_now = front_target_detected ?
					ROBOT_STARTUP_WAIT : ROBOT_SEARCHING;
			}
			xSemaphoreGive(motor_mutex);
			if (start_run) {
				if (front_target_detected) {
					ESP_LOGI(TAG, "run %" PRIu32 " started with opponent ahead; pausing before moving",
						telemetry_run_num());
				} else {
					ESP_LOGI(TAG, "run %" PRIu32 " started; searching at output %d",
						telemetry_run_num(), SEARCH_SPIN_OUTPUT);
				}
			}
		}

		if ((state == ROBOT_SEARCHING || state == ROBOT_APPROACHING ||
			 state == ROBOT_ATTACKING || state == ROBOT_PUSHING ||
			 state == ROBOT_ALIGNING || state == ROBOT_EDGE_RECOVERY ||
			 state == ROBOT_WOODPECKER || state == ROBOT_STARTUP_WAIT ||
			 state == ROBOT_DODGE_TURN || state == ROBOT_DODGE_STRAIGHT ||
			 state == ROBOT_FLANKING) &&
			button_pressed_event) {
			xSemaphoreTake(motor_mutex, portMAX_DELAY);
			robot_state_now = ROBOT_STOPPED;
			stop_motors_nolock_pleasework();
			xSemaphoreGive(motor_mutex);
		}

		if (state == ROBOT_STOPPED) {
			stop_motors_now();
			telemetry_dump_pleasework();
			robot_state_now = ROBOT_IDLE;

			vTaskDelay(pdMS_TO_TICKS(1500));
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
