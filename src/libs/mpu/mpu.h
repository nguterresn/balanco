#pragma once

#include <sys/cdefs.h>
#include <zephyr/drivers/sensor.h>

struct packet_pitch {
	float angle;
	float dt; // in s!
};

int mpu_init();
