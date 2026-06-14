#include "pid.h"
#include "data.h"
#include "central.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define PID_TUNER 1

#if PID_TUNER
static const struct device* const
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));

static void pid_serial_cb(const struct device* dev, void* user_data)
{
	(void)user_data;

	uint8_t c;
	int     error = 0;

	error = uart_irq_update(uart_dev);
	if (error < 0) {
		return;
	}

	error = uart_irq_rx_ready(uart_dev);
	if (error < 0) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		struct packet pck = { .id = PCKT_PID_TUN, .character = c };
		error             = central_send(&pck);
		if (error) {
			break;
		}
	}
}
#endif

void pid_init(struct pid* pid, float set_point, float kp, float ki, float kd)
{
	pid->set_point  = set_point;
	pid->kp         = kp;
	pid->kd         = kd;
	pid->ki         = ki;
	pid->integral   = 0;
	pid->prev_error = 0;

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

#if PID_TUNER
	int ret = uart_irq_callback_user_data_set(uart_dev, pid_serial_cb, NULL);
	if (ret < 0) {
		printk("UART failed to map IRQ callback!");
		return;
	}

	uart_irq_rx_enable(uart_dev);
#endif
}

float pid_update(struct pid* pid, float measured)
{
	static uint32_t timestamp = 0;

	uint32_t now   = k_uptime_get_32();
	uint32_t dt_ms = now - timestamp;
	timestamp      = now;

	float error = pid->set_point - measured;
	float dt    = (float)dt_ms / 1000.0f;
	pid->integral += error * dt;
	float derivative = (error - pid->prev_error) / dt;
	pid->prev_error  = error;

	return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
