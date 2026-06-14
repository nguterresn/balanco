#include "data.h"
#include "libs/accel/accel.h"
#include <stdint.h>
#include <sys/_intsup.h>
#include <zephyr/kernel.h>
#include "libs/accel/pid.h"
#include "libs/motor/motor.h"

static void th(void*, void*, void*);
static void handle_accel(struct packet_accel* accel);

K_THREAD_DEFINE(th_id, 1024, th, NULL, NULL, NULL, K_LOWEST_THREAD_PRIO, 0, 0);
K_MSGQ_DEFINE(msgq, sizeof(struct packet), 4, 1);

// Keep KP as (3), so that 100 > (3) * [0º, 30º]
static struct pid               pid = { .kp = 3, .ki = 0, .kd = 0 };
static struct accel_calibration cal;

int central_init()
{
	int error = accel_init();
	if (error) {
		return error;
	}

	pid_init(&pid);
	cal.done = false;

	return 0;
}

int central_send(enum packet_id id, void* data)
{
	struct packet pkt = { .id = id, .data = data };
	return k_msgq_put(&msgq, &pkt, K_NO_WAIT);
}

static int central_recv(struct packet* packet)
{
	return k_msgq_get(&msgq, packet, K_FOREVER);
}

static void th(void* arg1, void* arg2, void* arg3)
{
	int           error = 0;
	struct packet packet;

	for (;;) {
		error = central_recv(&packet);
		if (error) {
			continue;
		}

		switch (packet.id) {
		case PCKT_ACCEL:
			handle_accel(packet.data);
			break;
		default:
			break;
		}
	}
}

static void handle_accel(struct packet_accel* accel)
{
	static float prev_pitch = 0;
	float        pitch      = accel_gyro_get_pitch(accel->accel_x,
	                                               accel->accel_y,
	                                               accel->accel_z,
	                                               prev_pitch,
	                                               accel->gyro_y,
	                                               accel->dt);
	prev_pitch              = pitch;

	printf("[%f] [x, y, z] => [%f, %f, %f]\n",
	       pitch,
	       accel->accel_x,
	       accel->accel_y,
	       accel->accel_z);
	if (pitch > 30 || pitch < -30) {
		motor_brake();
		goto exit;
	}

	int32_t percent = pid_update(&pid, 0, pitch, accel->dt);
	if (percent == 0) {
		motor_brake();
	}
	else if (percent > 0) {
		motor_forward(percent);
	}
	else if (percent < 0) {
		motor_backwards(abs(percent));
	}

exit:
	free(accel);
}

