#include "motor.h"
#include <stdint.h>
#include <sys/errno.h>
#include <zephyr/drivers/pwm.h>

// Here's a very good source on how to improve the motor's performance.
// https://learn.adafruit.com/improve-brushed-dc-motor-performance?view=all
//
// This motor lib uses SLOW DECAY to provide better performance across
// different duty cycles.

static const struct pwm_dt_spec
    gpio0 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_ch0_gpio0));
static const struct pwm_dt_spec
    gpio1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_ch1_gpio1));
static const struct pwm_dt_spec
    gpio2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_ch2_gpio2));
static const struct pwm_dt_spec
    gpio3 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_ch3_gpio3));

int motor_brake();

int motor_init()
{
	if (!device_is_ready(gpio0.dev) || !device_is_ready(gpio1.dev) ||
	    !device_is_ready(gpio2.dev) || !device_is_ready(gpio3.dev)) {
		return -ENODEV;
	}

	return motor_brake();
}

int motor_brake()
{
	return pwm_set_dt(&gpio0, gpio0.period, gpio0.period) ||
	       pwm_set_dt(&gpio1, gpio1.period, gpio0.period) ||
	       pwm_set_dt(&gpio2, gpio2.period, gpio0.period) ||
	       pwm_set_dt(&gpio3, gpio3.period, gpio0.period);
}

int motor_forward(uint8_t percent)
{
	if (percent > 100) {
		return -EINVAL;
	}

	percent              = 100 - percent;
	uint32_t pulse_gpio1 = gpio1.period / 100 * percent;
	uint32_t pulse_gpio3 = gpio3.period / 100 * percent;

	return pwm_set_dt(&gpio0, gpio0.period, gpio0.period) || // xIN1
	       pwm_set_dt(&gpio1, gpio1.period, pulse_gpio1) ||  // xIN2
	       pwm_set_dt(&gpio2, gpio2.period, gpio2.period) ||
	       pwm_set_dt(&gpio3, gpio3.period, pulse_gpio3);
}

int motor_backwards(uint8_t percent)
{
	if (percent > 100) {
		return -EINVAL;
	}

	percent              = 100 - percent;
	uint32_t pulse_gpio0 = gpio0.period / 100 * percent;
	uint32_t pulse_gpio2 = gpio2.period / 100 * percent;

	return pwm_set_dt(&gpio0, gpio0.period, pulse_gpio0) ||  // xIN1
	       pwm_set_dt(&gpio1, gpio1.period, gpio1.period) || // xIN2
	       pwm_set_dt(&gpio2, gpio2.period, pulse_gpio2) ||
	       pwm_set_dt(&gpio3, gpio3.period, gpio3.period);

	return 0;
}
