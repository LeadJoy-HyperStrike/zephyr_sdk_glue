/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_adc16

#include <string.h>
#include <zephyr/drivers/adc.h>
#include <hpm_adc16_drv.h>
#include <hpm_clock_drv.h>
/* core_local_mem_to_sys_address() lives in the SoC hpm_misc.h (static inline);
 * the stock driver uses it for the seq-DMA buffer but omits the include (same
 * pattern the dma/ethernet glue drivers include explicitly). Without it the
 * call is an implicit decl and fails to link. */
#include <hpm_misc.h>
#include <hpm_l1c_drv.h>
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
#include <hpm_trgm_drv.h>
#endif
#include <zephyr/drivers/pinctrl.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_hpmicro_adc16);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc/adc_context.h"

struct hpmicro_adc16_config {
	ADC16_Type *base;
	clock_name_t adc_clock_name;
	clk_src_t adc_clock_src;
	clock_name_t src_clock_name;
	clk_src_t src_clock_src;
	uint32_t src_clock_div;
	uint32_t sample_time;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	bool trig_en;
	TRGM_Type *trig_reg;
	uint32_t trig_num;
	uint32_t trig_input_src;
#endif
};

struct hpmicro_adc16_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	/*
	 * Written by the ADC seq-DMA and read by the ISR after an l1c
	 * invalidate: must own its D-cache lines exclusively (64 B lines on
	 * HPM6E80/HPM5151). Without the alignment the invalidate would discard
	 * dirty neighbouring members sharing the line, and dirty-line eviction
	 * would overwrite DMA-written results.
	 */
	uint32_t seq_buffer[ADC_SOC_SEQ_MAX_LEN] __aligned(64);
	uint8_t channel_id;
	uint8_t channel_num;
	uint8_t resolution;
};

BUILD_ASSERT((ADC_SOC_SEQ_MAX_LEN * sizeof(uint32_t)) % 64 == 0,
	     "seq_buffer must cover whole D-cache lines");

#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
static void hpmicro_init_trigger_mux(TRGM_Type * ptr, uint32_t hpm_trig_input_src, uint32_t trig_num)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_pulse_at_input_rising_edge;
    trgm_output_cfg.input  = hpm_trig_input_src;
    trgm_output_config(ptr, trig_num, &trgm_output_cfg);
}
#endif

static int hpmicro_adc16_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	uint8_t channel_id = channel_cfg->channel_id;

	if (ADC16_IS_CHANNEL_INVALID(channel_id)) {
		LOG_ERR("Invalid channel %d", channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported channel acquisition time");
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported channel gain %d", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported channel reference");
		return -ENOTSUP;
	}

	return 0;
}

static int hpmicro_adc16_start_read(const struct device *dev,
				 const struct adc_sequence *sequence)
{
	struct hpmicro_adc16_data *data = dev->data;
	int error;

	if (sequence->oversampling != 0) {
		LOG_ERR("Unsupported oversampling");
		return -ENOTSUP;
	}
	data->resolution = sequence->resolution;
	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);
	error = adc_context_wait_for_completion(&data->ctx);

	return error;
}

static int hpmicro_adc16_read_async(const struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct hpmicro_adc16_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, async ? true : false, async);
	error = hpmicro_adc16_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

static int hpmicro_adc16_read(const struct device *dev,
			   const struct adc_sequence *sequence)
{
	return hpmicro_adc16_read_async(dev, sequence, NULL);
}

static void hpmicro_adc16_start_channel(const struct device *dev)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	ADC16_Type *base = config->base;

	adc16_channel_config_t channel_config;
	adc16_seq_config_t seq_cfg;
	adc16_dma_config_t dma_cfg;
	uint32_t channels;
	uint32_t channel_id;
	uint8_t channel_num = 0;

	/*
	 * channel_config/seq_cfg are stack variables and adc16_init_channel() /
	 * adc16_set_seq_config() write their fields straight into registers:
	 * uninitialized fields would leak stack garbage into PRD_THSHD_CFG /
	 * INT_EN / SEQ_QUE (random watchdog thresholds and interrupt enables).
	 * Zero seq_cfg/dma_cfg and start channel_config from
	 * adc16_get_channel_default_config() like the official adc16 sample.
	 */
	memset(&seq_cfg, 0, sizeof(seq_cfg));
	memset(&dma_cfg, 0, sizeof(dma_cfg));
	adc16_get_channel_default_config(&channel_config);
	channel_config.sample_cycle = config->sample_time;
	channels = data->channels;
	while (channels) {
		channel_id = find_lsb_set(channels) - 1;
		channels &= ~BIT(channel_id);

		channel_config.ch = channel_id;
		adc16_init_channel(base, &channel_config);
		seq_cfg.queue[channel_num].ch = channel_id;
		seq_cfg.queue[channel_num].seq_int_en = false;
		LOG_DBG("Starting channel %d", channel_id);
		channel_num++;
	};
	data->channel_num = channel_num;
	seq_cfg.seq_len = channel_num;
	seq_cfg.restart_en = false;
	/*
	 * HS2 fix kept across the vendor drop: one-shot, NOT continuous.
	 * Zephyr's adc_read() is a single acquisition. With cont_en=true the
	 * sequence free-runs after the trigger and (with seq_int_en on the last
	 * queue entry) fires one IRQ per pass back-to-back -> an ISR storm that
	 * starves the USBD/shell threads, so the board hangs on the very first
	 * adc_read (USB enumeration fails, no shell prompt).
	 */
	seq_cfg.cont_en = false;
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	if (config->trig_en) {
		seq_cfg.hw_trig_en = true;
		seq_cfg.sw_trig_en = false;
	} else {
		seq_cfg.hw_trig_en = false;
		seq_cfg.sw_trig_en = true;
	}
#else
	seq_cfg.hw_trig_en = false;
	seq_cfg.sw_trig_en = true;
#endif
	/* Match SDK: single-complete IRQ on last queue entry. */
	if (channel_num > 0) {
		seq_cfg.queue[channel_num - 1].seq_int_en = true;
	}
	adc16_set_seq_config(base, &seq_cfg);

	/* Set DMA config — results land in seq_buffer (SDK process_seq_data). */
	dma_cfg.start_addr = (uint32_t *)core_local_mem_to_sys_address(
		0, (uint32_t)data->seq_buffer);
	dma_cfg.buff_len_in_4bytes = channel_num;
	dma_cfg.stop_en = false;
	dma_cfg.stop_pos = 0;
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	if (config->trig_en) {
		hpmicro_init_trigger_mux(config->trig_reg, config->trig_input_src,
					 config->trig_num);
	}
#endif
	adc16_init_seq_dma(base, &dma_cfg);

	/*
	 * adc16_init_seq_dma() memset() the buffer through the D-cache, leaving
	 * dirty zero lines that may evict at any time and overwrite results the
	 * ADC DMA has meanwhile written to RAM. Flush (writeback + invalidate)
	 * so no dirty line is outstanding while the DMA runs; the ISR
	 * invalidates again before reading. The writeback must not be dropped
	 * for a plain invalidate: the zeroing is what clears the cycle bits the
	 * seq-DMA protocol relies on.
	 *
	 * Only correct because the sequence is not running yet. With
	 * config->trig_en the trigger mux is already live above, so a hardware
	 * trigger landing between the memset and this flush would have its
	 * results written back over. Nothing enables en-hw-trig today; revisit
	 * this ordering before the first user does.
	 */
	l1c_dc_flush((uint32_t)data->seq_buffer, sizeof(data->seq_buffer));

	adc16_enable_interrupts(base, adc16_event_seq_single_complete);
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	if (!config->trig_en) {
#endif
		adc16_trigger_seq_by_sw(base);
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	}
#endif
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct hpmicro_adc16_data *data =
		CONTAINER_OF(ctx, struct hpmicro_adc16_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	hpmicro_adc16_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct hpmicro_adc16_data *data =
		CONTAINER_OF(ctx, struct hpmicro_adc16_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

__attribute__((section(".isr"))) static void hpmicro_adc16_isr(const struct device *dev)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	ADC16_Type *base = config->base;
	adc16_seq_dma_data_t *dma_data = (adc16_seq_dma_data_t *)data->seq_buffer;
	uint32_t status;
	uint8_t channel_id;
	uint32_t channels;
	uint16_t result;
	uint8_t seq_idx = 0;

	status = adc16_get_status_flags(base);

	if (ADC16_INT_STS_SEQ_CVC_GET(status)) {
		adc16_clear_status_flags(base, status);
		/*
		 * ADC seq-DMA wrote seq_buffer; drop stale D-cache lines before
		 * reading. Deliberately NOT sys_cache_data_invd_range(): without
		 * CONFIG_CACHE_MANAGEMENT (our builds) that API is a silent
		 * no-op stub. The HPM l1c HAL works regardless of the Zephyr
		 * cache config. seq_buffer is __aligned(64) and whole-line
		 * sized, so the invalidate cannot touch neighbouring members.
		 */
		l1c_dc_invalidate((uint32_t)data->seq_buffer,
				  sizeof(data->seq_buffer));
		channels = data->channels;
		while (channels) {
			channel_id = find_lsb_set(channels) - 1;
			channels &= ~BIT(channel_id);
			ARG_UNUSED(channel_id);
			/* Same order as queue[] fill: LSB channel first. */
			result = dma_data[seq_idx++].result;
			result = (result & 0xffff) >> (16 - data->resolution);
			*data->buffer++ = result;
		};
		data->channels = 0;
		adc_context_on_sampling_done(&data->ctx, dev);
	}
	adc16_clear_status_flags(base, status);
}

static int hpmicro_adc16_init(const struct device *dev)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	ADC16_Type *base = config->base;
	adc16_config_t adc_config;
	int err;

	/*
	 * HS2 fix: ungate the ADC peripheral clock BEFORE touching ADC registers.
	 * clock_set_adc_source() only writes the ADCCLK mux; it does NOT enable the
	 * peripheral. Without adding the ADC clock to a clock group the block stays
	 * gated and the adc16_init() register access below stalls the AHB bus -- a
	 * silent, banner-less boot hang (the recurring "enable ADC => dead board").
	 * The HPM SDK's board_init_adc_clock() does this add-to-group first; the
	 * Zephyr glue driver omitted it.
	 */
	clock_add_to_group(config->adc_clock_name, 0);
	clock_set_adc_source(config->adc_clock_name, config->adc_clock_src);
	clock_set_source_divider(config->src_clock_name, config->src_clock_src,
				 config->src_clock_div);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	adc16_get_default_config(&adc_config);

	/* Align with SDK adc16 sample init_common_config(). */
	adc_config.res = adc16_res_16_bits;
	adc_config.conv_mode = adc16_conv_mode_sequence;
	/*
	 * RM 71.2.1.1: the convert clock must not exceed 50 MHz at 16-bit
	 * resolution. The controller clock is 200 MHz on this family (AHB0, or
	 * ANA fed from PLL1/4), so divide by 4 like the official adc16 sample.
	 */
	adc_config.adc_clk_div = adc16_clock_divider_4;
#if !defined(HPM_IP_FEATURE_ADC16_FORCE_SYNC_AHB) || !HPM_IP_FEATURE_ADC16_FORCE_SYNC_AHB
	/*
	 * RM 71.4.17 ADC_CFG0[SEL_SYNC_AHB] is only legal when the ADC controller
	 * clock and the DMA bus clock share the same source. Mirror the official
	 * sample: take the synchronous fast path only when the clock mux selects
	 * AHB0. Forcing it with an ANA source makes the seq-DMA bus interface
	 * cross asynchronous clock domains unsynchronized, which intermittently
	 * hangs the AHB bus (whole-system freeze once the DMA stream runs).
	 * On SoCs with HPM_IP_FEATURE_ADC16_FORCE_SYNC_AHB (HPM5151) the field
	 * does not exist and adc16_init() forces the legal value itself.
	 */
	adc_config.sel_sync_ahb = (clock_get_source(config->adc_clock_name) == clk_adc_src_ahb0);
#endif
	if (adc_config.conv_mode == adc16_conv_mode_sequence ||
	    adc_config.conv_mode == adc16_conv_mode_preemption) {
		adc_config.adc_ahb_en = true;
	}

	adc16_init(base, &adc_config);

	config->irq_config_func(dev);
	data->dev = dev;

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api hpmicro_adc16_driver_api = {
	.channel_setup = hpmicro_adc16_channel_setup,
	.read = hpmicro_adc16_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = hpmicro_adc16_read_async,
#endif
	/* VREFH typically tied to 3.3V on HPM EVKs (used by adc_raw_to_millivolts_dt). */
	.ref_internal = 3300,
};

#if CONFIG_ADC_TRIG
	#define ADC_TRIG_CFG(n)	\
	.trig_en = DT_INST_PROP(n, en_hw_trig),	\
	.trig_reg = (TRGM_Type *)DT_INST_PROP(n, trig_base),	\
	.trig_num = DT_INST_PROP(n, trig_num),	\
	.trig_input_src = DT_INST_PROP(n, trig_input_src)

#else
	#define ADC_TRIG_CFG(n)
#endif

#define ACD12_HPMICRO_INIT(n)						\
	static void hpmicro_adc16_config_func_##n(const struct device *dev); \
									\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static const struct hpmicro_adc16_config hpmicro_adc16_config_##n = {	\
		.base = (ADC16_Type *)DT_INST_REG_ADDR(n),		\
		.adc_clock_name = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, name),\
		.adc_clock_src = DT_INST_CLOCKS_CELL_BY_IDX(n, 0,  src),\
		.src_clock_name = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), name),\
		.src_clock_src = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), src),\
		.src_clock_div = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), div),\
		.sample_time = DT_INST_PROP(n, sample_time),	\
		.irq_config_func = hpmicro_adc16_config_func_##n,		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		ADC_TRIG_CFG(n)	\
	};								\
									\
	static struct hpmicro_adc16_data hpmicro_adc16_data_##n = {		\
		ADC_CONTEXT_INIT_TIMER(hpmicro_adc16_data_##n, ctx),	\
		ADC_CONTEXT_INIT_LOCK(hpmicro_adc16_data_##n, ctx),	\
		ADC_CONTEXT_INIT_SYNC(hpmicro_adc16_data_##n, ctx),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, hpmicro_adc16_init,			\
			    NULL, &hpmicro_adc16_data_##n,			\
			    &hpmicro_adc16_config_##n, POST_KERNEL,	\
			    CONFIG_ADC_INIT_PRIORITY,			\
			    &hpmicro_adc16_driver_api);			\
									\
	static void hpmicro_adc16_config_func_##n(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority), hpmicro_adc16_isr,	\
			    DEVICE_DT_INST_GET(n), 0);			\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(ACD12_HPMICRO_INIT)
