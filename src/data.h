#pragma once

#include <stdint.h>

enum packet_id {
	PCKT_ACCEL
};

struct packet {
	enum packet_id id;
  void *data;
} __packed;
