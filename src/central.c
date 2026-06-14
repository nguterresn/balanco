#include "data.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <zephyr/kernel.h>
#include "libs/mpu/pid.h"
#include "libs/mpu/mpu.h"
#include "libs/motor/motor.h"

static void th(void*, void*, void*);
static void handle_accel(float pitch);
static void handle_pid_tunning(char ch);

K_THREAD_DEFINE(th_id, 1024, th, NULL, NULL, NULL, K_LOWEST_THREAD_PRIO, 0, 0);
K_MSGQ_DEFINE(msgq, sizeof(struct packet), 4, 1);

static struct pid pid;

int central_init()
{
	pid_init(&pid, 0, 8.6, 0.00, 0.50);

	int error = mpu_init();
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
			handle_accel(packet.pitch);
			break;
		case PCKT_PID_TUN:
			handle_pid_tunning(packet.character);
			break;
		default:
			break;
		}
	}
}

static void handle_accel(float pitch)
{
	// printf("%f\n", pitch);

	int32_t percent = pid_update(&pid, pitch);
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
		pid.kp += 0.1;
		break;
	case 'a':
		pid.kp -= !pid.kp ? 0 : 0.1;
		break;
	case 'w':
		pid.ki += 0.01;
		break;
	case 's':
		pid.ki -= !pid.ki ? 0 : 0.01;
		break;
	case 'e':
		pid.kd += 0.01;
		break;
	case 'd':
		pid.kd -= !pid.kd ? 0 : 0.01;
		break;
	case 'r':
		pid.kp = 0;
		pid.ki = 0;
		pid.kd = 0;
		break;
	}

	printf("kp=%f ki=%f kd=%f\n", pid.kp, pid.ki, pid.kd);
}

