#pragma once

#include <stdint.h>

int motor_init();
int motor_brake();
int motor_forward(uint8_t percent);
int motor_backwards(uint8_t percent);
