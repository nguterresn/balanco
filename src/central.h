#pragma once

#include <stdint.h>
#include "data.h"

int central_init();
int central_send(struct packet* packet);
int central_recv(struct packet* packet);
