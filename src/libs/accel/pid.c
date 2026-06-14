#include "pid.h"

void pid_init(struct pid* pid)
{
	pid->integral   = 0;
	pid->prev_error = 0;
}

float pid_update(struct pid* pid, float setpoint, float measured, float dt)
{
	float error = setpoint - measured;
	pid->integral += error * dt;
	float derivative = (error - pid->prev_error) / dt;
	pid->prev_error  = error;

	return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
