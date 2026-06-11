#include "pid.h"
#include "string.h"

void pid_init(struct pid *pid) { memset(pid, 0, sizeof(struct pid)); }

float pid_update(struct pid *pid, float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  pid->integral += error * dt;
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
