#include "motor.h"
#include <stdint.h>
#include <sys/errno.h>
#include <zephyr/drivers/pwm.h>

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
	return pwm_set_dt(&gpio0, gpio0.period, 0) ||
	       pwm_set_dt(&gpio1, gpio1.period, 0) ||
	       pwm_set_dt(&gpio2, gpio2.period, 0) ||
	       pwm_set_dt(&gpio3, gpio3.period, 0);
}

int motor_forward(uint8_t percent)
{
	if (percent > 100) {
		return -EINVAL;
	}

	uint32_t pulse_gpio0 = gpio0.period / 100 * percent;
	uint32_t pulse_gpio2 = gpio2.period / 100 * percent;

	return pwm_set_dt(&gpio0, gpio0.period, pulse_gpio0) ||
	       pwm_set_dt(&gpio1, gpio1.period, 0) ||
	       pwm_set_dt(&gpio2, gpio2.period, pulse_gpio2) ||
	       pwm_set_dt(&gpio3, gpio3.period, 0);
}

int motor_backwards(uint8_t percent)
{
	if (percent > 100) {
		return -EINVAL;
	}

	uint32_t pulse_gpio1 = gpio1.period / 100 * percent;
	uint32_t pulse_gpio3 = gpio3.period / 100 * percent;

	return pwm_set_dt(&gpio0, gpio0.period, 0) ||
	       pwm_set_dt(&gpio1, gpio1.period, pulse_gpio1) ||
	       pwm_set_dt(&gpio2, gpio2.period, 0) ||
	       pwm_set_dt(&gpio3, gpio3.period, pulse_gpio3);

	return 0;
}
