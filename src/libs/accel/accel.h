#pragma once

#include <sys/cdefs.h>
#include <zephyr/drivers/sensor.h>

int accel_init();
float accel_gyro_get_pitch(float prev_pitch, float gyro, float ax, float ay,
                           float az, float dt, float alpha);
