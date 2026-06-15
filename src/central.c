#include "data.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <zephyr/kernel.h>
#include "libs/mpu/pid.h"
#include "libs/mpu/mpu.h"
#include "libs/motor/motor.h"

static void th(void*, void*, void*);
static void handle_accel(float pitch, float dt);
static void handle_pid_tunning(char ch);

K_THREAD_DEFINE(th_id, 1024, th, NULL, NULL, NULL, K_LOWEST_THREAD_PRIO, 0, 0);
K_MSGQ_DEFINE(msgq, sizeof(struct packet), 4, 1);

static struct pid pid;

int central_init()
{
	int error = 0;

	// error = pid_init(&pid, 0, 20.0, 0, 0.60);
	error = pid_init(&pid, 0, 25.0, 0.01, 0.30);
	if (error) {
		return error;
	}

	error = mpu_init();
	if (error) {
		return error;
	}

	return 0;
}

int central_send(struct packet* packet)
{
	return k_msgq_put(&msgq, packet, K_NO_WAIT);
}

inline static int central_recv(struct packet* packet, k_timeout_t timeout)
{
	return k_msgq_get(&msgq, packet, timeout);
}

static void th(void* arg1, void* arg2, void* arg3)
{
	int           error = 0;
	struct packet packet;

	for (;;) {
		error = central_recv(&packet, K_FOREVER);
		if (error) {
			continue;
		}

		switch (packet.id) {
		case PCKT_MPU_PITCH:
			handle_accel(packet.mpu.pitch, packet.mpu.dt);
			break;
		case PCKT_PID_TUN:
			handle_pid_tunning(packet.character);
			break;
		default:
			break;
		}
	}
}

static void handle_accel(float pitch, float dt)
{
	// printf("[%02.7f]:[%02.7f]\n", (double)dt, (double)pitch);

	int32_t percent = pid_update(&pid, pitch, dt);
	if (percent == 0 || (pitch > 45.0f || pitch < -45.0f)) {
		motor_brake();
	}
	else if (percent > 0) {
		motor_forward(percent);
	}
	else if (percent < 0) {
		motor_backwards(abs(percent));
	}
}

static void handle_pid_tunning(char ch)
{
	switch (ch) {
	case 'q':
		pid.kp += 0.1f;
		break;
	case 'a':
		pid.kp -= !pid.kp ? 0.0f : 0.1f;
		break;
	case 'w':
		pid.ki += 0.1f;
		break;
	case 's':
		pid.ki -= !pid.ki ? 0.0f : 0.1f;
		break;
	case 'e':
		pid.kd += 0.01f;
		break;
	case 'd':
		pid.kd -= !pid.kd ? 0.0f : 0.01f;
		break;
	case 'r':
		pid.kp = 0.0f;
		pid.ki = 0.0f;
		pid.kd = 0.0f;
		break;
	}

	printf("kp=%f ki=%f kd=%f\n",
	       (double)pid.kp,
	       (double)pid.ki,
	       (double)pid.kd);
}

