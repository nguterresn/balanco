#pragma once

#include <stdint.h>

enum packet_id {
	PCKT_MPU_PITCH,
	PCKT_PID_TUN
};

struct packet {
	enum packet_id id;

	union {
		struct {
			float pitch;
			float dt;
		} mpu;

		char character;
	};
};

