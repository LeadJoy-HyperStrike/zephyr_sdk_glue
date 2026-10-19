/*
 * Copyright (c) 2025 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "usbh_core.h"

extern int msc_main(void);

#ifdef CONFIG_PM
#include <soc.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/state.h>

#define HPM_PM_STOP_SUBSTATE 1U
#define HPM_STOP_TIMER_SLEEP_MS 3000

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

static const struct pm_state_info *get_stop_info(void)
{
	const struct pm_state_info *states;
	uint8_t count = pm_state_cpu_get_all(0, &states);

	for (uint8_t i = 0; i < count; i++) {
		if ((states[i].state == PM_STATE_SUSPEND_TO_IDLE) &&
		    (states[i].substate_id == HPM_PM_STOP_SUBSTATE)) {
			return &states[i];
		}
	}

	return NULL;
}

static int one_shot_stop(int32_t timeout_ms)
{
	const struct pm_state_info *info = get_stop_info();
	int64_t t0;
	int64_t elapsed;
	int ret;

	if (info == NULL) {
		printk("Stop not in DTS\r\n");
		return -EINVAL;
	}

	k_sem_reset(&pm_wake_sem);
	if (!pm_state_force(0U, info)) {
		printk("pm_state_force failed\r\n");
		return -EIO;
	}

	printk("one-shot Stop, %d ms or PY07 (RESET if silent)\r\n",
	       timeout_ms);
	t0 = k_uptime_get();
	ret = k_sem_take(&pm_wake_sem, K_MSEC(timeout_ms));
	elapsed = k_uptime_get() - t0;

	if (hpm_pm_wakeup_from_wkup_pin()) {
		printk("wake PY07, %lld ms sem=%d\r\n", elapsed, ret);
	} else {
		printk("wake PTMR/other, %lld ms sem=%d\r\n", elapsed, ret);
	}

	return 0;
}
#endif

static bool wait_msc_sda(const char *tag, int32_t timeout_ms)
{
	int32_t left = timeout_ms;

	while (left > 0) {
		if (usbh_find_class_instance("/dev/sda") != NULL) {
			printk("%s: /dev/sda ready, remain %d ms\r\n", tag, left);
			return true;
		}
		k_msleep(100);
		left -= 100;
	}

	printk("%s: no /dev/sda after %d ms\r\n", tag, timeout_ms);
	return false;
}

int main(void)
{
	uint32_t usb_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));

	printk("Start usb host task...\r\n");
	usbh_initialize(0, usb_base);

#ifdef CONFIG_PM
	int64_t t0;
	int64_t elapsed;

	pm_notifier_register(&pm_notifier);

	printk("USB host up (Stop locked), wait MSC\r\n");
	if (!wait_msc_sda("enum", 15000)) {
		return 0;
	}

	printk("USB host deinit...\r\n");
	usbh_deinitialize(0);
	printk("USB host down\r\n");

	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);

	t0 = k_uptime_get();
	k_msleep(500);
	elapsed = k_uptime_get() - t0;
	printk("PTMR run/wait slept %lld ms (Stop locked)\r\n", elapsed);

	one_shot_stop(HPM_STOP_TIMER_SLEEP_MS);

	printk("USB host re-init, wait MSC\r\n");
	usbh_initialize(0, usb_base);
	if (!wait_msc_sda("re-enum", 15000)) {
		return 0;
	}
#endif

	while (usbh_find_class_instance("/dev/sda") == NULL) {
		k_sleep(K_MSEC(1000));
	}

	msc_main();

	return 0;
}
