#include <stdio.h>
#include <sys/cdefs.h>
#include <zephyr/kernel.h>

#include "central.h"

int main()
{
	k_sleep(K_MSEC(2000));

	int error = central_init();
	if (error) {
		printf("Failed to initilize mpu_init %d\n", error);
	}

	for (;;) {
		printf("ping\n");
		k_sleep(K_MSEC(2000));
	}
}
