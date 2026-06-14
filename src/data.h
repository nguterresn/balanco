#pragma once

#include <stdint.h>
#include "libs/mpu/mpu.h"

enum packet_id {
	PCKT_MPU_PITCH,
	PCKT_PID_TUN
};

struct packet {
	enum packet_id id;

	union {
		struct packet_pitch pitch;
		char                character;
	};
};

