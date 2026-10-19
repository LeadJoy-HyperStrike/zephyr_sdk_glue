/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * HPM5100EVK power-mode validation shell. SoC register programming and the
 * dedicated WKUP interrupt are owned by soc/hpmicro/HPM5100/power.c.
 */

#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/state.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/printk.h>

#include "hpm5100_lp.h"

#define HPM_PM_STOP_SUBSTATE 1U
#define HPM_PM_WAIT_SUBSTATE 1U
#define HPM_STOP_TIMER_SLEEP_MS 3000

static volatile bool wait_wake_from_uart;
static volatile bool stop_wake_from_puart;
static K_SEM_DEFINE(pm_wake_sem, 0, 1);

static void pm_state_exit(enum pm_state state)
{
	if ((state == PM_STATE_RUNTIME_IDLE) ||
	    (state == PM_STATE_SUSPEND_TO_IDLE)) {
		k_sem_give(&pm_wake_sem);
	}
}

static struct pm_notifier pm_notifier = {
	.state_exit = pm_state_exit,
};

static void uart_wake_isr(const struct device *dev, void *user_data)
{
	volatile bool *woke = user_data;

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		uint8_t byte;

		(void)uart_fifo_read(dev, &byte, 1);
	}

	*woke = true;
}

static int arm_uart_wake(const struct device *dev, volatile bool *woke)
{
	uint8_t stale;
	int ret;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	while (uart_poll_in(dev, &stale) == 0) {
	}

	ret = uart_irq_callback_user_data_set(dev, uart_wake_isr,
					      (void *)woke);
	if (ret != 0) {
		return ret;
	}

	if (!pm_device_wakeup_enable(dev, true)) {
		(void)uart_irq_callback_user_data_set(dev, NULL, NULL);
		return -ENOTSUP;
	}

	*woke = false;
	uart_irq_rx_enable(dev);

	return 0;
}

static void disarm_uart_wake(const struct device *dev)
{
	if (!device_is_ready(dev)) {
		return;
	}

	uart_irq_rx_disable(dev);
	(void)uart_irq_callback_user_data_set(dev, NULL, NULL);
	(void)pm_device_wakeup_enable(dev, false);
}

static const struct pm_state_info *get_pm_state_info(enum pm_state state,
						     uint8_t substate_id)
{
	const struct pm_state_info *states;
	uint8_t count = pm_state_cpu_get_all(0, &states);

	for (uint8_t i = 0; i < count; i++) {
		if ((states[i].state == state) &&
		    (states[i].substate_id == substate_id)) {
			return &states[i];
		}
	}

	return NULL;
}

static int request_pm_state(enum pm_state state, uint8_t substate_id)
{
	const struct pm_state_info *info = get_pm_state_info(state, substate_id);

	if (info == NULL) {
		printk("PM state %u/%u not described by DTS\n", state, substate_id);
		return -EINVAL;
	}

	/*
	 * pm_state_force() bypasses pm_policy_next_state(), so retain the
	 * all-devices-idle guard used by the normal PM policy.
	 */
	if (pm_device_is_any_busy()) {
		printk("PM state %u/%u blocked by a busy device\n",
		       state, substate_id);
		return -EBUSY;
	}

	k_sem_reset(&pm_wake_sem);
	if (!pm_state_force(0U, info)) {
		printk("Failed to force PM state %u/%u\n", state, substate_id);
		return -EIO;
	}

	k_sem_take(&pm_wake_sem, K_FOREVER);

	return 0;
}

void prepare_soc_low_power(void)
{
	pm_notifier_register(&pm_notifier);
	/*
	 * This sample only enters low power via pm_state_force(). Lock idle
	 * Wait/Stop/soft-off so the menu loop cannot auto-Stop (JTAG all ones).
	 */
	pm_policy_state_lock_get(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
}

int enter_wait_mode(void)
{
	const struct device *console =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	int ret;

	ret = arm_uart_wake(console, &wait_wake_from_uart);
	if (ret != 0) {
		printk("Failed to arm console UART wake: %d\r\n", ret);
		return ret;
	}

	printk("Entering wait mode\r\n");
	printk("Press WKUP (PY07 / P1[8]) or send a byte to UART0 RX\r\n");

	ret = request_pm_state(PM_STATE_RUNTIME_IDLE, HPM_PM_WAIT_SUBSTATE);
	disarm_uart_wake(console);
	if (ret != 0) {
		return ret;
	}

	if (hpm_pm_wakeup_from_wkup_pin()) {
		printk("Wake source: WKUP pin (PY07)\r\n");
	} else if (wait_wake_from_uart) {
		printk("Wake source: UART RX\r\n");
	} else {
		printk("Wake source: unknown\r\n");
	}

	wait_wake_from_uart = false;

	return 0;
}

int enter_stop_mode(void)
{
	const struct device *puart = DEVICE_DT_GET(DT_NODELABEL(puart));
	int ret;

	ret = arm_uart_wake(puart, &stop_wake_from_puart);
	if (ret != 0) {
		printk("Failed to arm PUART Stop wake: %d\r\n", ret);
		return ret;
	}

	printk("Entering stop mode\r\n");
	printk("Press WKUP (PY07 / P1[8]) or send a byte to PUART RX (PY01)\r\n");

	ret = request_pm_state(PM_STATE_SUSPEND_TO_IDLE,
			       HPM_PM_STOP_SUBSTATE);
	disarm_uart_wake(puart);
	if (ret != 0) {
		return ret;
	}

	if (hpm_pm_wakeup_from_wkup_pin()) {
		printk("Wake source: WKUP pin (PY07)\r\n");
	} else if (stop_wake_from_puart) {
		printk("Wake source: PUART RX (PY01)\r\n");
	} else {
		printk("Wake source: unknown\r\n");
	}

	stop_wake_from_puart = false;

	return 0;
}

int enter_stop_timer_mode(void)
{
	int64_t t0;
	int64_t elapsed;

	printk("Idle Stop for %d ms (PTMR sysclock)\r\n",
	       HPM_STOP_TIMER_SLEEP_MS);
	printk("WKUP (PY07 / P1[8]) can interrupt Stop, sleep still runs\r\n");

	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	t0 = k_uptime_get();
	k_msleep(HPM_STOP_TIMER_SLEEP_MS);
	elapsed = k_uptime_get() - t0;

	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	if (hpm_pm_wakeup_from_wkup_pin()) {
		printk("Wake source: WKUP pin (PY07), slept %lld ms\r\n",
		       elapsed);
	} else {
		printk("Wake source: PTMR sysclock, slept %lld ms\r\n",
		       elapsed);
	}

	return 0;
}

void enter_standby_mode(void)
{
	printk("standby mode is not supported on HPM5151\n");
}

void enter_shutdown_mode(void)
{
	printk("Entering shutdown mode\n");
	printk("Press WKUP (PY07) or RESET to restart\n");
	z_sys_poweroff();
}
