/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interactive power mode switch (HPM SDK power_mode_switch port).
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "hpm_clock_drv.h"
#include "hpm5100_lp.h"

static void show_clocks(void)
{
	uint32_t cpu_hz = clock_get_frequency(clock_cpu0);
	uint32_t ahb_hz = clock_get_frequency(clock_ahb);

	printk("CPU %u Hz (%u MHz), AHB %u Hz (%u MHz)\r\n",
	       cpu_hz, cpu_hz / 1000000U, ahb_hz, ahb_hz / 1000000U);
}

static void show_menu(void)
{
	printk("\r\nPower mode switch demo:\r\n"
	       "  1 - Enter wait mode\r\n"
	       "  2 - Enter stop mode\r\n"
	       "  3 - Enter standby mode\r\n"
	       "  4 - Enter shutdown mode\r\n"
	       "  5 - Idle stop 3s (PTMR sysclock)\r\n\r\n");
}

int main(void)
{
	const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!device_is_ready(console_dev)) {
		printk("console device not ready\r\n");
		return 0;
	}

	printk("Power mode switch starting\r\n");
	show_clocks();
	prepare_soc_low_power();
	show_menu();

	while (true) {
		unsigned char c;
		int ret = uart_poll_in(console_dev, &c);

		if (ret != 0) {
			k_yield();
			continue;
		}

		if ((c == 'w') || (c == '\r') || (c == '\n')) {
			continue;
		}

		switch (c) {
		case '1':
			if (enter_wait_mode() == 0) {
				printk("Woke up from the wait mode\r\n");
			}
			break;
		case '2':
			if (enter_stop_mode() == 0) {
				printk("Woke up from the stop mode\r\n");
			}
			break;
		case '3':
			enter_standby_mode();
			break;
		case '4':
			enter_shutdown_mode();
			break;
		case '5':
			if (enter_stop_timer_mode() == 0) {
				printk("Woke up from the stop mode\r\n");
			}
			break;
		default:
			printk("[!!] Please choose options listed above.\r\n");
			break;
		}

		show_menu();
	}

	return 0;
}
