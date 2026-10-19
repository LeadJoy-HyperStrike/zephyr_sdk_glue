/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPTMR0 CH0 top IRQ smoke (~1 Hz).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define COUNTER_NODE DT_ALIAS(counter0)

static const struct device *cnt_dev = DEVICE_DT_GET(COUNTER_NODE);
static struct k_sem top_sem;

static void top_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	k_sem_give(&top_sem);
}

int main(void)
{
	uint32_t freq;
	uint32_t ticks;
	int ret;
	int i;

	k_sem_init(&top_sem, 0, 1);

	if (!device_is_ready(cnt_dev)) {
		printk("GPTMR not ready\n");
		return 0;
	}

	freq = counter_get_frequency(cnt_dev);
	printk("GPTMR0 counter smoke freq=%u\n", freq);
	if (freq == 0U) {
		printk("freq 0\n");
		return 0;
	}

	ticks = freq; /* ~1s top */
	ret = counter_set_top_value(cnt_dev,
				    &(const struct counter_top_cfg){
					    .ticks = ticks,
					    .callback = top_cb,
					    .user_data = NULL,
					    .flags = 0,
				    });
	if (ret) {
		printk("set_top %d\n", ret);
		return 0;
	}
	printk("top ticks=%u (~1s)\n", ticks);

	ret = counter_start(cnt_dev);
	if (ret) {
		printk("start %d\n", ret);
		return 0;
	}
	printk("started\n");

	for (i = 0; i < 3; i++) {
		ret = k_sem_take(&top_sem, K_MSEC(2000));
		if (ret) {
			printk("TOP FAIL timeout i=%d\n", i);
			return 0;
		}
		printk("top irq #%d\n", i + 1);
	}

	(void)counter_stop(cnt_dev);
	printk("GPTMR_OK\n");
	return 0;
}
