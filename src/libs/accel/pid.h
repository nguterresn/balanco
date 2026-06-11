#pragma once

struct pid {
  float kp, ki, kd;
  float integral;
  float prev_error;
};

void pid_init(struct pid *pid);
float pid_update(struct pid *pid, float setpoint, float measured, float dt);
