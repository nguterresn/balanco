#include "accel.h"
#include "central.h"
#include "data.h"
#include "zephyr/drivers/sensor.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define M_PI  3.14159265358979323846
#define ALPHA 0.98f // 0.98 = 98% gyro, 2% accel

static int process_mpu6050(const struct device* dev);

static const struct device* const
                mpu6050   = DEVICE_DT_GET_ONE(invensense_mpu6050);
static uint32_t timestamp = 0;

// TODO(nuno): I'm not entirely sure if this function is called
// from an hardware or software interrupt.
// Regardless, I'll follow common sense and
// offload the processing to another thread.
inline static void handle_mpu6050_drdy(const struct device*         dev,
                                       const struct sensor_trigger* trig)
{
	int error = process_mpu6050(dev);

	if (error < 0) {
		// TODO(nuno): Error handling is missing.
		return;
	}
}

static int process_mpu6050(const struct device* dev)
{
	struct sensor_value accel[3];
	struct sensor_value gyro_y;

	int error = sensor_sample_fetch(dev);
	if (error < 0) {
		return error;
	}

	error = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (error < 0) {
		return error;
	}

	error = sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	if (error < 0) {
		return error;
	}

	double x = sensor_value_to_double(&accel[0]);
	double y = sensor_value_to_double(&accel[1]);
	double z = sensor_value_to_double(&accel[2]);

	double g_y = sensor_value_to_double(&gyro_y);

	uint32_t now = k_uptime_get_32();
	uint32_t dt  = now - timestamp;
	timestamp    = now;

	struct packet_accel* pckt = (struct packet_accel*)
	    malloc(sizeof(struct packet_accel));

	pckt->accel_x = x;
	pckt->accel_y = y;
	pckt->accel_z = z;
	pckt->gyro_y  = g_y;
	pckt->dt      = dt;

	return central_send(PCKT_ACCEL, pckt);
}

int accel_init()
{
	int error;

	if (!device_is_ready(mpu6050)) {
		printf("Device %s is not ready\n", mpu6050->name);
		return -ENXIO;
	}

	struct sensor_trigger trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	error = sensor_trigger_set(mpu6050, &trigger, handle_mpu6050_drdy);
	if (error < 0) {
		printf("[%s] Cannot configure trigger\n", __func__);
		return error;
	}

	printk("[%s] Configured for triggered sampling.\n", __func__);
	return 0;
}

float accel_get_pitch(float accel_x, float accel_y, float accel_z)
{
	return atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) *
	       (180.0 / M_PI);
}

float accel_gyro_get_pitch(float accel_x, float accel_y, float accel_z,
                           float prev_pitch, float gyro_y, float dt)
{
	float pitch_accel = accel_get_pitch(accel_x, accel_y, accel_z);
	float pitch_gyro  = prev_pitch + gyro_y * dt;
	return ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_accel;
}
