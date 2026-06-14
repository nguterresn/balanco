#pragma once

#include <sys/cdefs.h>
#include <zephyr/drivers/sensor.h>

struct packet_accel {
	float    accel_x;
	float    accel_y;
	float    accel_z;
	float    gyro_x;
	float    gyro_y;
	float    gyro_z;
	uint32_t dt;
};

struct accel_calibration {
	float set_point;
	bool  done;
};

int   accel_init();
float accel_get_pitch(float accel_x, float accel_y, float accel_z);
float accel_gyro_get_pitch(float accel_x, float accel_y, float accel_z,
                           float prev_pitch, float gyro_y, float dt);
