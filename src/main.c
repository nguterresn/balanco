#include <stdio.h>
#include <zephyr/kernel.h>
#include "central.h"

int main()
{
	// Let me connect.
	// Hack for now until I figure out a better way.
	k_sleep(K_MSEC(2000));

	int error = central_init();
	if (error) {
		printf("Failed to initilize accel_init %d\n", error);
	}

	for (;;) {
		printf("ping\n");
		k_sleep(K_MSEC(2000));
	}
}
