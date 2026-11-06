#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral_limit;
	float output_limit;
	float integral;

	float p_term;
	float i_term;
	float d_term;
	float error;
} pid_t;

void pid_setup(pid_t *pid, float kp, float ki, float kd,
	float integral_limit, float output_limit);

void pid_clear_pleasework(pid_t *pid);

float pid_calc(pid_t *pid, float error, float measured_rate, float dt_s);

#endif
