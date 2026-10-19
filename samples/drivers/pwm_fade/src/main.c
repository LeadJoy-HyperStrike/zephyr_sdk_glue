/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Smoke: PWM3 ch0 PA16 + ch1 PA17.
 * True complementary: same pulse/CMP, ch1 inverted — edges must stay locked.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static const struct pwm_dt_spec pwm_out0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
static const struct pwm_dt_spec pwm_out1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));

#define PERIOD_START_NSEC PWM_USEC(1000)
#define PERIOD_MIN_NSEC   PWM_USEC(100)
/* Hold fixed duty so scope can confirm no phase walk. */
#define SYNC_HOLD_MS      8000

static int pwm_pair_set(uint32_t period, uint32_t pulse)
{
	int ret;

	ret = pwm_set(pwm_out0.dev, pwm_out0.channel, period, pulse,
		      PWM_POLARITY_NORMAL);
	if (ret) {
		return ret;
	}

	return pwm_set(pwm_out1.dev, pwm_out1.channel, period, pulse,
		       PWM_POLARITY_INVERTED);
}

int main(void)
{
	uint32_t period = PERIOD_START_NSEC;
	uint32_t pulse;
	uint8_t step;
	int ret;

	if (!pwm_is_ready_dt(&pwm_out0) || !pwm_is_ready_dt(&pwm_out1)) {
		printk("PWM3 not ready\n");
		return 0;
	}

	printk("PWM3 fade ch0@PA16 + ch1@PA17 (shared CMP, ch1 inverted)\n");

	while (pwm_pair_set(period, period / 2U)) {
		period /= 2U;
		if (period < PERIOD_MIN_NSEC) {
			printk("Error: no usable period\n");
			return 0;
		}
	}

	pulse = (period * 3U) / 10U; /* 30% — easy to see lock */
	ret = pwm_pair_set(period, pulse);
	if (ret) {
		printk("pwm_pair_set failed %d\n", ret);
		return 0;
	}

	printk("period %u ns pulse %u; HOLD %u ms — check PA16/PA17 edge lock\n",
	       period, pulse, SYNC_HOLD_MS);
	k_msleep(SYNC_HOLD_MS);
	printk("hold done; sweeping duty\n");

	while (1) {
		for (step = 1; step <= 10; step++) {
			pulse = (period * step) / 10U;
			ret = pwm_pair_set(period, pulse);
			if (ret) {
				printk("pwm_pair_set failed %d\n", ret);
				return 0;
			}
			printk("OK pulse %u/%u (ch1 inverted, shared CMP)\n",
			       pulse, period);
			k_msleep(500);
		}
	}

	return 0;
}
