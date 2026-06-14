#pragma once

#include <stdint.h>
#include "data.h"

int central_init();
int central_send(enum packet_id id, void* data);
int central_recv(struct packet* packet);
