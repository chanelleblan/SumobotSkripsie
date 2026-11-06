#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define TELEM_MAGIC 0x4D4C4554u
#define TELEM_FOOTER_MAGIC 0x444E4554u
#define TELEM_VERSION 1

#define TELEM_CAPACITY 3000

#define TELEM_ANGLE_SCALE 10.0f
#define TELEM_RATE_SCALE 10.0f
#define TELEM_ACCEL_SCALE 1000.0f
#define TELEM_TERM_SCALE 10.0f

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t version;
	uint16_t record_size;
	uint32_t run_number;
	uint32_t reserved0;
	float kp;
	float ki;
	float kd;
	float integral_limit;
	float sample_rate_hz;
	int16_t drive_speed;
	int16_t turn_speed;
	uint16_t target_distance_mm;
	uint16_t reserved1;
	char build[20];
} telem_header_t;

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint32_t sample_count;
	uint32_t dropped_count;
	uint32_t duration_ms;
	uint8_t reserved[48];
} telem_footer_t;

typedef struct __attribute__((packed)) {
	uint32_t t_us;
	int16_t yaw_ddeg;
	int16_t setpoint_ddeg;
	int16_t yaw_rate_ddps;
	int16_t accel_fwd_mg;
	int16_t accel_lat_mg;
	int16_t p_term;
	int16_t i_term;
	int16_t d_term;
	int16_t motor_a;
	int16_t motor_b;
	uint16_t range_left_mm;
	uint16_t range_right_mm;
	uint8_t state;
	uint8_t flags;
	uint16_t reserved;
} telem_record_t;

#define TELEM_FLAG_TARGET_VISIBLE 0x01
#define TELEM_FLAG_CONTACT 0x02

bool telemetry_setup(void);

bool telemetry_start_run_pleasework(float kp, float ki, float kd, float integral_limit,
	int16_t drive_speed, int16_t turn_speed, uint16_t target_distance_mm);

void telemetry_save(const telem_record_t *record);

bool telemetry_dump_pleasework(void);

uint32_t telemetry_how_many(void);
uint32_t telemetry_run_num(void);
bool telemetry_full_pleasework(void);

#endif
