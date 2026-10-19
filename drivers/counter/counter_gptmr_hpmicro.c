/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPTMRv2 channel used as a Zephyr counter (reload / top IRQ).
 */

#define DT_DRV_COMPAT hpmicro_hpm_gptmr

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include "hpm_clock_drv.h"
#include "hpm_gptmrv2_drv.h"

LOG_MODULE_REGISTER(counter_hpm_gptmr, CONFIG_COUNTER_LOG_LEVEL);

struct hpm_gptmr_config {
	struct counter_config_info info;
	GPTMRV2_Type *base;
	clock_name_t clock_name;
	uint8_t channel;
	void (*irq_config)(const struct device *dev);
};

struct hpm_gptmr_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t top;
	uint32_t tick_freq;
	uint32_t prescaler;
	bool started;
	bool pm_suspended;
};

static int hpm_gptmr_start(const struct device *dev)
{
	const struct hpm_gptmr_config *cfg = dev->config;
	struct hpm_gptmr_data *data = dev->data;

	gptmr_enable_irq(cfg->base, GPTMR_CH_RLD_IRQ_MASK(cfg->channel));
	gptmr_start_counter(cfg->base, cfg->channel);
	data->started = true;
	return 0;
}

static int hpm_gptmr_stop(const struct device *dev)
{
	const struct hpm_gptmr_config *cfg = dev->config;
	struct hpm_gptmr_data *data = dev->data;

	gptmr_stop_counter(cfg->base, cfg->channel);
	gptmr_disable_irq(cfg->base, GPTMR_CH_RLD_IRQ_MASK(cfg->channel));
	data->started = false;
	return 0;
}

static int hpm_gptmr_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct hpm_gptmr_config *cfg = dev->config;

	*ticks = gptmr_channel_get_counter(cfg->base, cfg->channel,
					   gptmr_counter_type_normal);
	return 0;
}

static int hpm_gptmr_set_top_value(const struct device *dev,
				   const struct counter_top_cfg *top_cfg)
{
	const struct hpm_gptmr_config *cfg = dev->config;
	struct hpm_gptmr_data *data = dev->data;
	bool was_started = data->started;

	if (top_cfg->ticks == 0U || top_cfg->ticks > 0xFFFFFFU) {
		return -EINVAL;
	}

	data->top_cb = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;
	data->top = top_cfg->ticks;

	if (was_started) {
		if ((top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) != 0) {
			return -ENOTSUP;
		}
		gptmr_stop_counter(cfg->base, cfg->channel);
	}

	gptmr_channel_config_update_reload(cfg->base, cfg->channel, top_cfg->ticks);
	gptmr_channel_reset_count(cfg->base, cfg->channel);

	if (was_started) {
		gptmr_start_counter(cfg->base, cfg->channel);
	}

	return 0;
}

static uint32_t hpm_gptmr_get_pending_int(const struct device *dev)
{
	const struct hpm_gptmr_config *cfg = dev->config;

	return gptmr_check_status(cfg->base, GPTMR_CH_RLD_STAT_MASK(cfg->channel)) ? 1U : 0U;
}

static uint32_t hpm_gptmr_get_top_value(const struct device *dev)
{
	struct hpm_gptmr_data *data = dev->data;

	return data->top;
}

static uint32_t hpm_gptmr_get_freq(const struct device *dev)
{
	struct hpm_gptmr_data *data = dev->data;

	return data->tick_freq;
}

__attribute__((section(".isr")))
static void hpm_gptmr_isr(const struct device *dev)
{
	const struct hpm_gptmr_config *cfg = dev->config;
	struct hpm_gptmr_data *data = dev->data;
	uint32_t mask = GPTMR_CH_RLD_STAT_MASK(cfg->channel);

	if (gptmr_check_status(cfg->base, mask)) {
		gptmr_clear_status(cfg->base, mask);
		if (data->top_cb != NULL) {
			data->top_cb(dev, data->top_user_data);
		}
	}
}

#ifdef CONFIG_PM_DEVICE
static int hpm_gptmr_pm_action(const struct device *dev,
			       enum pm_device_action action)
{
	const struct hpm_gptmr_config *cfg = dev->config;
	struct hpm_gptmr_data *data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		if (data->pm_suspended ||
		    pm_device_wakeup_is_enabled(dev)) {
			return 0;
		}

		if (data->started) {
			return -EBUSY;
		}

		clock_remove_from_group(cfg->clock_name, 0);
		data->pm_suspended = true;
		return 0;

	case PM_DEVICE_ACTION_RESUME:
		if (!data->pm_suspended) {
			return 0;
		}

		clock_add_to_group(cfg->clock_name, 0);
		data->pm_suspended = false;
		return 0;

	default:
		return -ENOTSUP;
	}
}
#endif

static int hpm_gptmr_init(const struct device *dev)
{
	const struct hpm_gptmr_config *cfg = dev->config;
	struct hpm_gptmr_data *data = dev->data;
	gptmr_channel_config_t ch_cfg;
	uint32_t src_freq;

	/*
	 * SDK board_init_gptmr_clock(): only clock_add_to_group().
	 * Source comes from SYSCTL preset / BootROM (not forced to OSC24M;
	 * OSC24M is only used for mchtmr0 in init_board_clock_source()).
	 */
	clock_add_to_group(cfg->clock_name, 0);
	src_freq = clock_get_frequency(cfg->clock_name);
	if (src_freq == 0U) {
		LOG_ERR("GPTMR clock freq 0");
		return -EIO;
	}

	/*
	 * GPTMR reload is 24-bit. Prescale toward ~1 MHz tick (SDK samples
	 * often use APP_GPTMR_TARGET_FREQ=100kHz the same way).
	 */
	data->prescaler = src_freq / 1000000U;
	if (data->prescaler < 1U) {
		data->prescaler = 1U;
	}
	data->tick_freq = src_freq / data->prescaler;

	gptmr_channel_get_default_config(cfg->base, &ch_cfg);
	ch_cfg.prescaler = data->prescaler;
	ch_cfg.reload = cfg->info.max_top_value;
	if (gptmr_channel_config(cfg->base, cfg->channel, &ch_cfg, false) !=
	    status_success) {
		return -EIO;
	}

	data->top = cfg->info.max_top_value;
	data->top_cb = NULL;
	data->started = false;
	data->pm_suspended = false;

	cfg->irq_config(dev);
	return 0;
}

static const struct counter_driver_api hpm_gptmr_api = {
	.start = hpm_gptmr_start,
	.stop = hpm_gptmr_stop,
	.get_value = hpm_gptmr_get_value,
	.set_top_value = hpm_gptmr_set_top_value,
	.get_pending_int = hpm_gptmr_get_pending_int,
	.get_top_value = hpm_gptmr_get_top_value,
	.get_freq = hpm_gptmr_get_freq,
};

#define HPM_GPTMR_INIT(n)                                                          \
	IF_DISABLED(DT_INST_PROP_OR(n, sys_clock, 0), (                            \
	static void hpm_gptmr_irq_config_##n(const struct device *dev);            \
	PM_DEVICE_DT_INST_DEFINE(n, hpm_gptmr_pm_action);                           \
	static struct hpm_gptmr_data hpm_gptmr_data_##n;                           \
	static const struct hpm_gptmr_config hpm_gptmr_cfg_##n = {                  \
		.info = {                                                          \
			.max_top_value = 0xFFFFFFU,                                    \
			.freq = 0,                                                 \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,                     \
			.channels = 0,                                             \
		},                                                                 \
		.base = (GPTMRV2_Type *)DT_INST_REG_ADDR(n),                       \
		.clock_name = (clock_name_t)DT_INST_PROP(n, clk_name),             \
		.channel = DT_INST_PROP(n, channel),                               \
		.irq_config = hpm_gptmr_irq_config_##n,                            \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(n, hpm_gptmr_init, PM_DEVICE_DT_INST_GET(n),         \
			      &hpm_gptmr_data_##n,                                  \
			      &hpm_gptmr_cfg_##n, POST_KERNEL,                     \
			      CONFIG_COUNTER_INIT_PRIORITY, &hpm_gptmr_api);       \
	static void hpm_gptmr_irq_config_##n(const struct device *dev)             \
	{                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),             \
			    hpm_gptmr_isr, DEVICE_DT_INST_GET(n), 0);              \
		irq_enable(DT_INST_IRQN(n));                                       \
	}))

DT_INST_FOREACH_STATUS_OKAY(HPM_GPTMR_INIT)
