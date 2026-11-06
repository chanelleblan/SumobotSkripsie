#include "pid.h"

static float limit_value_pleasework(float value, float limit)
{
	if (value > limit) {
		return limit;
	}
	if (value < -limit) {
		return -limit;
	}
	return value;
}

void pid_setup(pid_t *pid, float kp, float ki, float kd,
	float integral_limit, float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral_limit = integral_limit;
	pid->output_limit = output_limit;
	pid_clear_pleasework(pid);
}

void pid_clear_pleasework(pid_t *pid)
{
	pid->integral = 0.0f;
	pid->p_term = 0.0f;
	pid->i_term = 0.0f;
	pid->d_term = 0.0f;
	pid->error = 0.0f;
}

float pid_calc(pid_t *pid, float error, float measured_rate, float dt_s)
{
	if (dt_s <= 0.0f) {
		return limit_value_pleasework(pid->p_term + pid->i_term + pid->d_term, pid->output_limit);
	}

	pid->error = error;
	pid->p_term = pid->kp * error;

	pid->d_term = -pid->kd * measured_rate;

	float maybe_integral = pid->integral + error * dt_s;
	float candidate_i_term = limit_value_pleasework(pid->ki * maybe_integral, pid->integral_limit);
	float total_before_limit_pleasework = pid->p_term + candidate_i_term + pid->d_term;

	bool windup_pleasework = (total_before_limit_pleasework > pid->output_limit && error > 0.0f) ||
		(total_before_limit_pleasework < -pid->output_limit && error < 0.0f);
	if (!windup_pleasework) {
		pid->integral = maybe_integral;
		pid->i_term = candidate_i_term;
		total_before_limit_pleasework = pid->p_term + pid->i_term + pid->d_term;
	} else {
		total_before_limit_pleasework = pid->p_term + pid->i_term + pid->d_term;
	}

	return limit_value_pleasework(total_before_limit_pleasework, pid->output_limit);
}
