#include <stdio.h>
#include <zephyr/kernel.h>
#include "central.h"

int main()
{
	int error = central_init();
	if (error) {
		printf("Failed to initilize mpu_init %d\n", error);
	}

	for (;;) {
		printf("ping\n");
		k_sleep(K_MSEC(2000));
	}
}
