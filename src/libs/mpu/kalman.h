#pragma once

struct kalman1d {
	float angle; // estimated pitch (degrees)
	float bias;  // estimated gyro bias (deg/s)
	float P[2][2];
};

void  kalman_init(struct kalman1d* k);
float kalman_update(struct kalman1d* k, float pitch_accel, float gyr_y_deg,
                    float dt);
