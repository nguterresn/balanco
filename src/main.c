#include "libs/accel/accel.h"
#include "libs/motor/motor.h"
#include <stdio.h>
#include <zephyr/kernel.h>

int main() {
  accel_init();

  for (;;) {
    printf("ping\n");
    k_sleep(K_MSEC(2000));

    for (int i = 0; i < 100; i++) {
      motor_backwards(i);
      k_sleep(K_MSEC(20));
    }
    for (int i = 100; i > 0; i--) {
      motor_backwards(i);
      k_sleep(K_MSEC(20));
    }

    for (int i = 0; i < 100; i++) {
      motor_forward(i);
      k_sleep(K_MSEC(20));
    }
    for (int i = 100; i > 0; i--) {
      motor_forward(i);
      k_sleep(K_MSEC(20));
    }
  }
}
