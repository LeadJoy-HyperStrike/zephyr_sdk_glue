/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include "hpm_clock_drv.h"
#include "dt-bindings/pwm/hpmicro-pwm-common.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_hpmicro, CONFIG_PWM_LOG_LEVEL);

/* Support both PWM and PWMv2 */
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED && CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
#error "Cannot enable both PWM and PWMv2 in the same build"
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
#include <hpm_pwm_drv.h>
#define DT_DRV_COMPAT hpmicro_hpm_pwm
#define HPM_PWM_BASE_TYPE PWM_Type
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
#include <hpm_pwmv2_drv.h>
#define DT_DRV_COMPAT hpmicro_hpm_pwmv2
#define HPM_PWM_BASE_TYPE PWMV2_Type  /* PWMv2 may use different type, adjust as needed */
#else
#error "No PWM peripheral enabled"
#endif

struct pwm_hpmicro_config {
	HPM_PWM_BASE_TYPE *base;
	uint32_t clock_name;
	uint32_t period;
	uint32_t dead_zone_in_half_cycle;
	const struct pinctrl_dev_config *pincfg;
};

struct pwm_hpmicro_data {
	uint32_t configured_period;
	uint8_t channel_ready_mask;
	uint8_t invert_mask; /* bit N = channel N inverted */
};

/* PWM version specific implementations */
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
/*
 * Edge-aligned PWM v1 (SDK pwm_output style).
 * - 64-bit duty math (uint32 pulse*rld overflows above ~30% @120MHz/1kHz)
 * - One counter/reload for the module
 * - CMP shadow updates on reload (atomic vs update_on_modify glitches)
 * - Opposite polarity on ch0/ch1 → both bind to CMP0 (true complementary lock)
 */
static uint8_t hpmicro_pwm_v1_cmp_index(const struct pwm_hpmicro_data *data,
					uint32_t channel, bool invert)
{
	uint8_t partner = (uint8_t)(channel ^ 1U);
	bool partner_ready;
	bool partner_invert;

	if (channel > 1U) {
		return (uint8_t)channel;
	}

	partner_ready = (data->channel_ready_mask & BIT(partner)) != 0U;
	partner_invert = (data->invert_mask & BIT(partner)) != 0U;

	/* Complementary pair: share CMP0 so edges cannot drift. */
	if (partner_ready && (partner_invert != invert)) {
		return 0U;
	}

	return (uint8_t)channel;
}

static int hpmicro_pwm_v1_set_cycles(const struct device *dev, uint32_t channel,
		       uint32_t period_cycles, uint32_t pulse_cycles,
		       pwm_flags_t flags)
{
	const struct pwm_hpmicro_config *config = dev->config;
	struct pwm_hpmicro_data *data = dev->data;
	pwm_config_t pwm_config;
	pwm_cmp_config_t cmp_config = {0};
	pwm_output_channel_t ch_config = {0};
	HPM_PWM_BASE_TYPE *pwm_base = config->base;
	uint32_t rld;
	uint32_t target_cmp;
	uint8_t cmp_index;
	bool invert;
	bool reinit;
	bool first_channel;
	bool ch_ready;
	bool invert_changed;
	bool need_setup;

	if (channel >= PWM_SOC_PWM_MAX_COUNT) {
		return -EINVAL;
	}

	if (period_cycles == 0U || period_cycles > 0xffffffU) {
		return -ENOTSUP;
	}

	if (pulse_cycles > period_cycles) {
		pulse_cycles = period_cycles;
	}

	invert = (flags & PWM_POLARITY_INVERTED) != 0;

	rld = (period_cycles > 1U) ? (period_cycles - 1U) : period_cycles;

	if (pulse_cycles == 0U) {
		target_cmp = rld + 1U;
	} else if (pulse_cycles >= period_cycles) {
		target_cmp = 0U;
	} else {
		target_cmp = rld - (uint32_t)(((uint64_t)pulse_cycles * rld) /
					     period_cycles);
		if (target_cmp == rld) {
			target_cmp = rld + 1U;
		}
	}

	/* Period is shared; polarity is per-channel and must not force reinit. */
	reinit = (data->channel_ready_mask != 0U) &&
		 (data->configured_period != rld);
	ch_ready = (data->channel_ready_mask & BIT(channel)) != 0U;
	invert_changed = ch_ready &&
			 (((data->invert_mask & BIT(channel)) != 0U) != invert);
	need_setup = reinit || !ch_ready || invert_changed;

	if (need_setup) {
		first_channel = (data->channel_ready_mask == 0U) || reinit;

		if (reinit || first_channel) {
			pwm_stop_counter(pwm_base);
			pwm_deinit(pwm_base);
			data->channel_ready_mask = 0U;
			data->invert_mask = 0U;
			pwm_set_reload(pwm_base, 0, rld);
			pwm_set_start_count(pwm_base, 0, 0);
		}

		if (invert) {
			data->invert_mask |= BIT(channel);
		} else {
			data->invert_mask &= (uint8_t)~BIT(channel);
		}
		data->channel_ready_mask |= BIT(channel);
		data->configured_period = rld;

		cmp_index = hpmicro_pwm_v1_cmp_index(data, channel, invert);

		pwm_get_default_pwm_config(pwm_base, &pwm_config);
		pwm_config.enable_output = true;
		pwm_config.dead_zone_in_half_cycle = config->dead_zone_in_half_cycle;
		pwm_config.invert_output = invert;

		cmp_config.mode = pwm_cmp_mode_output_compare;
		cmp_config.cmp = target_cmp;
		cmp_config.enable_ex_cmp = false;
		/* Reload-timed: both channels see the same shadow load point. */
		cmp_config.update_trigger = pwm_shadow_register_update_on_hw_event;

		if (pwm_setup_waveform(pwm_base, channel, &pwm_config, cmp_index,
				       &cmp_config, 1) != status_success) {
			LOG_ERR("failed to setup waveform ch%u", channel);
			return -EIO;
		}

		/*
		 * If this completes a ch0/ch1 complementary pair, retarget the
		 * partner to CMP0 as well (shared compare → edge-locked).
		 */
		if (channel <= 1U) {
			uint8_t partner = (uint8_t)(channel ^ 1U);

			if ((data->channel_ready_mask & BIT(partner)) != 0U &&
			    (((data->invert_mask & BIT(partner)) != 0U) != invert)) {
				ch_config.cmp_start_index = 0;
				ch_config.cmp_end_index = 0;
				ch_config.invert_output =
					(data->invert_mask & BIT(partner)) != 0U;
				pwm_config_output_channel(pwm_base, partner, &ch_config);

				ch_config.invert_output = invert;
				pwm_config_output_channel(pwm_base, channel, &ch_config);
				cmp_index = 0U;
			}
		}

		/* CMP@reload triggers shadow load (SDK pwm_output pattern). */
		if (first_channel) {
			pwm_cmp_config_t trig = {0};

			trig.mode = pwm_cmp_mode_output_compare;
			trig.cmp = rld;
			trig.update_trigger = pwm_shadow_register_update_on_modify;
			pwm_load_cmp_shadow_on_match(pwm_base, PWM_SOC_CMP_MAX_COUNT - 1U,
						     &trig);
			pwm_start_counter(pwm_base);
			pwm_issue_shadow_register_lock_event(pwm_base);
		} else {
			pwm_issue_shadow_register_lock_event(pwm_base);
		}

		return 0;
	}

	cmp_index = hpmicro_pwm_v1_cmp_index(data, channel, invert);
	pwm_shadow_register_unlock(pwm_base);
	pwm_cmp_update_cmp_value(pwm_base, cmp_index, target_cmp, 0);
	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED */

#if CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
static int hpmicro_pwm_v2_set_cycles(const struct device *dev, uint32_t channel,
		       uint32_t period_cycles, uint32_t pulse_cycles,
		       pwm_flags_t flags)
{
	const struct pwm_hpmicro_config *config = dev->config;
	HPM_PWM_BASE_TYPE *pwm_base = config->base;
	uint32_t rld, xrld, prld, rld_cmp, xrld_cmp;
	uint16_t i, j;
	
	if (channel > 7) {
		return -ENOTSUP;
	}
	/* Calculate counter and cmp index based on channel
	 * Channel 0,1 -> Counter 0, CMP 0-3
	 * Channel 2,3 -> Counter 1, CMP 4-7
	 * Channel 4,5 -> Counter 2, CMP 8-11
	 * Channel 6,7 -> Counter 3, CMP 12-15
	 */
	pwm_counter_t counter = (pwm_counter_t)(channel >> 1);
	uint8_t cmp_start_index = (channel >> 1) << 2;  /* counter * 4 */
	uint8_t cmp_index1 = cmp_start_index + ((channel & 0x01) << 1);     /* +0/+2 for even/odd */
	uint8_t cmp_index2 = cmp_index1 + 1;                                /* +1/+3 for even/odd */
	
	/* Shadow register allocation for PWMv2:
	 * Each counter uses 5 shadow registers:
	 * - Counter 0: shadows 0(reload), 1,2(ch0), 3,4(ch1)
	 * - Counter 1: shadows 5(reload), 6,7(ch2), 8,9(ch3)  
	 * - Counter 2: shadows 10(reload), 11,12(ch4), 13,14(ch5)
	 * - Counter 3: shadows 15(reload), 16,17(ch6), 18,19(ch7)
	 */
	uint8_t shadow_reload = counter * 5;                                 /* 0, 5, 10, 15 */
	uint8_t shadow_cmp1 = shadow_reload + 1 + ((channel & 0x01) << 1);  /* ch0:1, ch1:3 */
	uint8_t shadow_cmp2 = shadow_cmp1 + 1;                              /* ch0:2, ch1:4 */

	/* Handle polarity inversion */
	bool invert_output = false;
	if (flags == PWM_POLARITY_INVERTED) {
		invert_output = true;
	} else if (flags == PWM_POLARITY_NORMAL) {
		invert_output = false;
	} else {
		return -ENOTSUP;
	}

	/* pwmv2 only support period <= 0xffffff */
	if (period_cycles > 0xffffff) {
		return -ENOTSUP;
	} else {
		rld = period_cycles;
		xrld = 0;
		prld = period_cycles;
		rld_cmp = period_cycles - pulse_cycles;
		xrld_cmp = 0;
	}

	/* Unlock shadow registers */
	pwmv2_shadow_register_unlock(pwm_base);

	/* Set reload value with extended precision */
	pwmv2_set_shadow_val(pwm_base, shadow_reload, rld, xrld, false);
	
	/* Set compare values for PWMv2 edge-aligned mode 
	* PWMv2 needs two compare values to generate one PWM waveform:
	* - cmp1: rising edge (usually 0 for edge-aligned)  
	* - cmp2: falling edge (duty cycle point)
	*/
	uint32_t cmp1_val = 0;  /* Rising edge at counter start */
	uint32_t cmp2_val = rld_cmp;  /* Falling edge for duty cycle */
	
	pwmv2_set_shadow_val(pwm_base, shadow_cmp1, cmp1_val, 0, false);
	pwmv2_set_shadow_val(pwm_base, shadow_cmp2, cmp2_val, xrld_cmp, false);

	/* Configure counter (first time only or when period changes) */
	pwmv2_counter_select_data_offset_from_shadow_value(pwm_base, counter, shadow_reload);
	pwmv2_counter_burst_disable(pwm_base, counter);
	pwmv2_set_reload_update_time(pwm_base, counter, pwm_reload_update_on_reload);

	/* Configure compare sources for two CMP values */
	pwmv2_select_cmp_source(pwm_base, cmp_index1, cmp_value_from_shadow_val, shadow_cmp1);
	pwmv2_select_cmp_source(pwm_base, cmp_index2, cmp_value_from_shadow_val, shadow_cmp2);
	pwmv2_cmp_update_trig_time(pwm_base, cmp_index1, pwm_shadow_register_update_on_reload);
	pwmv2_cmp_update_trig_time(pwm_base, cmp_index2, pwm_shadow_register_update_on_reload);

	/* Set output polarity */
	if (invert_output) {
		pwmv2_enable_output_invert(pwm_base, (pwm_channel_t)channel);
	} else {
		pwmv2_disable_output_invert(pwm_base, (pwm_channel_t)channel);
	}

	/* Set dead zone if configured */
	if (config->dead_zone_in_half_cycle > 0) {
		pwmv2_set_dead_area(pwm_base, (pwm_channel_t)channel, config->dead_zone_in_half_cycle);
	}

	/* Lock shadow registers to apply changes */
	pwmv2_shadow_register_lock(pwm_base);

	/* Handle four cmp mode for odd channels (1, 3, 5, 7) */
	if (channel & 0x01) {
		if (pwmv2_get_cmp_working_status(pwm_base, (pwm_channel_t)(channel - 1)) == 0xFFFFFF00) {
			/* Odd channel - need to enable four_cmp for proper operation */
			pwmv2_enable_four_cmp(pwm_base, (pwm_channel_t)(channel - 1));  /* Enable for even channel */
		} else {
			pwmv2_disable_four_cmp(pwm_base, (pwm_channel_t)(channel - 1));
		}
	} else {
		/* Even channel - disable four cmp for normal PWM operation */
		pwmv2_disable_four_cmp(pwm_base, (pwm_channel_t)channel);
	}
	/* Enable channel output */
	pwmv2_channel_enable_output(pwm_base, (pwm_channel_t)channel);
	
	/* Enable and start counter */
	pwmv2_enable_counter(pwm_base, counter);
	pwmv2_start_pwm_output(pwm_base, counter);

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED */

static int hpmicro_pwm_set_cycles(const struct device *dev, uint32_t channel,
		       uint32_t period_cycles, uint32_t pulse_cycles,
		       pwm_flags_t flags)
{
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
	return hpmicro_pwm_v1_set_cycles(dev, channel, period_cycles, pulse_cycles, flags);
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
	return hpmicro_pwm_v2_set_cycles(dev, channel, period_cycles, pulse_cycles, flags);
#endif
}

#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
static int hpmicro_pwm_v1_get_cycles_per_sec(const struct device *dev,
			       uint32_t channel, uint64_t *cycles)
{
	const struct pwm_hpmicro_config *config = dev->config;
	uint32_t freqc;

	freqc = clock_get_frequency(config->clock_name);
	*cycles = freqc;

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED */

#if CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
static int hpmicro_pwm_v2_get_cycles_per_sec(const struct device *dev,
			       uint32_t channel, uint64_t *cycles)
{
	const struct pwm_hpmicro_config *config = dev->config;
	uint32_t freqc;

	freqc = clock_get_frequency(config->clock_name);
	*cycles = freqc;

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED */

static int hpmicro_pwm_get_cycles_per_sec(const struct device *dev,
			       uint32_t channel, uint64_t *cycles)
{
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
	return hpmicro_pwm_v1_get_cycles_per_sec(dev, channel, cycles);
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
	return hpmicro_pwm_v2_get_cycles_per_sec(dev, channel, cycles);
#endif
}

#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
static int pwm_hpmicro_v1_init(const struct device *dev)
{
	const struct pwm_hpmicro_config *config = dev->config;
	struct pwm_hpmicro_data *data = dev->data;
	int err;

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	data->channel_ready_mask = 0;
	data->configured_period = 0;
	data->invert_mask = 0;
	/* Clock already in group0 via soc.c (clock_pwm0..3). */

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED */

#if CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
static int pwm_hpmicro_v2_init(const struct device *dev)
{
	const struct pwm_hpmicro_config *config = dev->config;
	HPM_PWM_BASE_TYPE *pwm_base = config->base;
	int err;

	/* Apply pin configuration */
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Deinitialize PWMv2 to reset to known state */
	pwmv2_deinit(pwm_base);

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED */

static int pwm_hpmicro_init(const struct device *dev)
{
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
	return pwm_hpmicro_v1_init(dev);
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
	return pwm_hpmicro_v2_init(dev);
#endif
}

static const struct pwm_driver_api pwm_hpmicro_driver_api = {
	.set_cycles = hpmicro_pwm_set_cycles,
	.get_cycles_per_sec = hpmicro_pwm_get_cycles_per_sec,
};

#define PWM_DEVICE_INIT_HPMICRO(n)			  \
	static struct pwm_hpmicro_data pwm_hpmicro_data_##n;		  \
	PINCTRL_DT_INST_DEFINE(n);					  \
									  \
	static const struct pwm_hpmicro_config pwm_hpmicro_config_##n = {     \
		.base = (HPM_PWM_BASE_TYPE *)DT_INST_REG_ADDR(n),	  \
		.clock_name = DT_INST_CLOCKS_CELL(n, name),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		  \
		.period = DT_INST_PROP(n, period_init),			\
		.dead_zone_in_half_cycle = DT_INST_PROP(n, dead_zone_in_half_cycle),	\
	};								  \
									  \
	DEVICE_DT_INST_DEFINE(n,					  \
			    pwm_hpmicro_init,				  \
			    NULL,					  \
			    &pwm_hpmicro_data_##n,			  \
			    &pwm_hpmicro_config_##n,			  \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
			    &pwm_hpmicro_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT_HPMICRO)
