#include "accel.h"
#include "zephyr/drivers/sensor.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define M_PI 3.14159265358979323846
#define ALPHA 0.98f // 0.98 = 98% gyro, 2% accel

static float accel_get_pitch(float ax, float ay, float az);
static int process_mpu6050(const struct device *dev);

static const struct device *const mpu6050 =
    DEVICE_DT_GET_ONE(invensense_mpu6050);
static uint32_t timestamp = 0;

inline static void handle_mpu6050_drdy(const struct device *dev,
                                       const struct sensor_trigger *trig) {
  int error = process_mpu6050(dev);

  if (error < 0) {
    // Error handling is missing.
    return;
  }
}

// This should (but does not) happen, more or less, every 4ms (250Hz)!
static int process_mpu6050(const struct device *dev) {
  struct sensor_value accel[3];
  int error = sensor_sample_fetch(dev);
  if (error < 0) {
    return error;
  }

  error = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  if (error < 0) {
    return error;
  }

  double x = sensor_value_to_double(&accel[0]);
  double y = sensor_value_to_double(&accel[1]);
  double z = sensor_value_to_double(&accel[2]);

  float pitch = accel_get_pitch(x, y, z);

  uint32_t now = k_uptime_get_32();
  uint32_t dt = now - timestamp;
  timestamp = now;

  printf("[%f]º [x, y, z] => [%f, %f, %f]\n", pitch, x, y, z);

  return 0;
}

int accel_init() {
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

static float accel_get_pitch(float ax, float ay, float az) {
  return atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0 / M_PI);
}

float accel_gyro_get_pitch(float prev_pitch, float gyro, float ax, float ay,
                           float az, float dt, float alpha) {
  float pitch_accel = accel_get_pitch(ax, ay, az);
  float pitch_gyro = prev_pitch + gyro * dt;
  return alpha * pitch_gyro + (1.0f - alpha) * pitch_accel;
}
