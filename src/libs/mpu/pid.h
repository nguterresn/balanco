#pragma once

struct pid {
	float kp, ki, kd;
	float integral;
	float prev_error;
	float set_point;
};

void  pid_init(struct pid* pid, float set_point, float kp, float ki, float kd);
float pid_update(struct pid* pid, float measured);
