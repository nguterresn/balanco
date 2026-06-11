#include "libs/accel/accel.h"
#include "libs/motor/motor.h"
#include <stdio.h>
#include <zephyr/kernel.h>

int main() {
  // Let me connect.
  // Hack for now until I figure out a better way.
  k_sleep(K_MSEC(2000));

  // Now you can start.
  int error = accel_init();
  if (error) {
    printf("Failed to initilize accel_init %d\n", error);
  }

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
