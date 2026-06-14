#pragma once

struct Kalman1D {
  float angle; // estimated pitch (degrees)
  float bias;  // estimated gyro bias (deg/s)
  float P[2][2];
};

void kalman_init(struct Kalman1D *k);
float kalman_update(struct Kalman1D *k, float pitch_accel, float gyro, float dt,
                    float Q_angle, float Q_bias, float R_measure);
