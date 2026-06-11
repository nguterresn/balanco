#include "accel.h"
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

static const struct device *const mpu6050 =
    DEVICE_DT_GET_ONE(invensense_mpu6050);

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

  printf("[x, y, z] => [%f, %f, %f]\n", x, y, z);

  return 0;
}

inline static void handle_mpu6050_drdy(const struct device *dev,
                                       const struct sensor_trigger *trig) {
  int error = process_mpu6050(dev);

  if (error < 0) {
    // Error handling is missing.
    return;
  }
}

int accel_init() {
  int error;
  struct sensor_value temperature;

  if (!device_is_ready(mpu6050)) {
    printf("Device %s is not ready\n", mpu6050->name);
    return -ENXIO;
  }

  error = sensor_channel_get(mpu6050, SENSOR_CHAN_DIE_TEMP, &temperature);
  if (error) {
    return error;
  }

  printf("Device's temperature => %d\n", temperature.val1);

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
