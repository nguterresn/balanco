#include "mpu.h"
#include "central.h"
#include "data.h"
#include "zephyr/drivers/sensor.h"
#include "kalman.h"
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

struct cal_offset {
	float x;
	float y;
	float z;
};

#define CALIBRATION_PERFORMED 0
#if !CALIBRATION_PERFORMED
#define CALIBRATION_SAMPLES 1000
#endif

#define M_PI 3.14159265358979323846f

static int          mpu_calibrate(const struct device* dev);
inline static float mpu_get_accel_pitch(float accel_x, float accel_y,
                                        float accel_z);

static const struct device* const
    mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);

// TODO(nuno): calibration flag should come from flash, done once.
static struct cal_offset accel_off;
static struct cal_offset gyro_off;
static struct kalman1d   k;

inline static void handle_mpu6050_drdy(const struct device*         dev,
                                       const struct sensor_trigger* trig)
{
	static uint32_t timestamp_in_ms = 0;

	struct sensor_value accel[3];
	struct sensor_value gyro_y;

	int error = sensor_sample_fetch(dev);
	if (error < 0) {
		return;
	}

	error = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (error < 0) {
		return;
	}

	error = sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	if (error < 0) {
		return;
	}

	float x = sensor_value_to_float(&accel[0]);
	float y = sensor_value_to_float(&accel[1]);
	float z = sensor_value_to_float(&accel[2]);

	float g_y = sensor_value_to_double(&gyro_y);

	uint32_t now    = k_uptime_get_32();
	uint32_t dt_ms  = now - timestamp_in_ms;
	float    dt     = timestamp_in_ms == 0 ? 0 : (float)dt_ms / 1000.0f;
	timestamp_in_ms = now;

	float pitch = kalman_update(&k,
	                            mpu_get_accel_pitch(x - accel_off.x,
	                                                y - accel_off.y,
	                                                z - accel_off.z),
	                            (g_y - gyro_off.y) * (180.0f / M_PI),
	                            dt);

	struct packet pck = { .id = PCKT_MPU_PITCH, .mpu = { pitch, dt } };
	central_send(&pck);
}

int mpu_init()
{
	int error;

	if (!device_is_ready(mpu6050)) {
		printf("Device %s is not ready\n", mpu6050->name);
		return -ENXIO;
	}

#if CALIBRATION_PERFORMED
// TODO(nuno): fetch offset from flash.
#error "Reading calibration offsets from flash is not yet supported!"
#else
	mpu_calibrate(mpu6050);
#endif

	kalman_init(&k);

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

static int mpu_calibrate(const struct device* dev)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];

	for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
		int error = sensor_sample_fetch(dev);
		if (error < 0) {
			return error;
		}

		error = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (error < 0) {
			return error;
		}

		error = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		if (error < 0) {
			return error;
		}

		accel_off.x += sensor_value_to_float(&accel[0]);
		accel_off.y += sensor_value_to_float(&accel[1]);
		accel_off.z += sensor_value_to_float(&accel[2]);

		gyro_off.x += sensor_value_to_float(&gyro[0]);
		gyro_off.y += sensor_value_to_float(&gyro[1]);
		gyro_off.z += sensor_value_to_float(&gyro[2]);

		k_sleep(K_MSEC(5));
	}

	accel_off.x /= CALIBRATION_SAMPLES;
	accel_off.y /= CALIBRATION_SAMPLES;
	accel_off.z /= CALIBRATION_SAMPLES;
	accel_off.z -= 9.81f; // Gravity applies.

	gyro_off.x /= CALIBRATION_SAMPLES;
	gyro_off.y /= CALIBRATION_SAMPLES;
	gyro_off.z /= CALIBRATION_SAMPLES;

	printf("[OFFSETS] accel [%f, %f, %f] gyro [%f, %f, %f]\n",
	       (double)accel_off.x,
	       (double)accel_off.y,
	       (double)accel_off.z,
	       (double)gyro_off.x,
	       (double)gyro_off.y,
	       (double)gyro_off.z);

	// TODO:
	// Save the offsets into flash.

	return 0;
}

inline static float mpu_get_accel_pitch(float accel_x, float accel_y,
                                        float accel_z)
{
	return atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) *
	       (180.0f / M_PI);
}
