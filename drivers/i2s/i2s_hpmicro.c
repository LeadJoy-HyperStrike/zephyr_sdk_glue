/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hpmicro_hpm_i2s

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <hpm_clock_drv.h>
#define i2s_config hpm_sdk_i2s_config
#include <hpm_i2s_drv.h>
#undef i2s_config

struct hpmicro_i2s_config {
	I2S_Type *base;
	uint32_t clock_name;
	uint32_t clock_src;
	uint32_t clock_div;
	const struct pinctrl_dev_config *pincfg;
};

struct hpmicro_i2s_stream {
	struct i2s_config cfg;
	enum i2s_state state;
	bool configured;
};

#ifdef CONFIG_I2S_HPMICRO_DMA
#define HPMICRO_I2S_TX_QUEUE_DEPTH CONFIG_I2S_HPMICRO_DMA_TX_QUEUE_DEPTH
#define HPMICRO_I2S_RX_QUEUE_DEPTH CONFIG_I2S_HPMICRO_DMA_RX_QUEUE_DEPTH
#define HPMICRO_I2S_TX_WATCHDOG_MS CONFIG_I2S_HPMICRO_DMA_TX_WATCHDOG_MS
#define HPMICRO_I2S_RX_WATCHDOG_MS CONFIG_I2S_HPMICRO_DMA_RX_WATCHDOG_MS
#else
#define HPMICRO_I2S_TX_QUEUE_DEPTH 1
#define HPMICRO_I2S_RX_QUEUE_DEPTH 1
#endif
struct hpmicro_i2s_data;

#ifdef CONFIG_I2S_HPMICRO_DMA
struct hpmicro_i2s_dma_chan {
	struct hpmicro_i2s_data *owner;
	bool is_tx;
	const struct device *dma_dev;
	uint32_t channel;
	uint32_t slot;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
	struct k_sem sync;
	volatile int cb_status;
	volatile bool done;
	bool available;
};
#endif

struct hpmicro_i2s_tx_block {
	void *mem_block;
	size_t size;
};

struct hpmicro_i2s_rx_block {
	void *mem_block;
	size_t size;
};

struct hpmicro_i2s_data {
	const struct device *dev;
	struct hpmicro_i2s_stream rx;
	struct hpmicro_i2s_stream tx;
	struct k_mutex lock;
#ifdef CONFIG_I2S_HPMICRO_DMA
	struct k_work tx_dma_work;
	struct k_work rx_dma_work;
	struct k_work_delayable tx_dma_watchdog_work;
	struct k_work_delayable rx_dma_watchdog_work;
	bool tx_dma_active;
	void *tx_inflight_block;
	size_t tx_inflight_size;
	int tx_last_err;
	struct hpmicro_i2s_tx_block tx_queue[HPMICRO_I2S_TX_QUEUE_DEPTH];
	uint8_t tx_q_head;
	uint8_t tx_q_tail;
	uint8_t tx_q_count;
	bool rx_dma_active;
	void *rx_inflight_block;
	size_t rx_inflight_size;
	int rx_last_err;
	struct hpmicro_i2s_rx_block rx_done_queue[HPMICRO_I2S_RX_QUEUE_DEPTH];
	uint8_t rx_q_head;
	uint8_t rx_q_tail;
	uint8_t rx_q_count;
#endif
#ifdef CONFIG_I2S_HPMICRO_DMA
	struct hpmicro_i2s_dma_chan dma_tx;
	struct hpmicro_i2s_dma_chan dma_rx;
#endif
};

#if defined(clock_aud0) && defined(clock_aud1) && defined(clk_src_pll2_clk0)
static void hpmicro_i2s_prepare_audio_root(uint32_t i2s_clock_name)
{
	/*
	 * Align with HPM SDK board clock defaults:
	 * aud0/aud1 sourced from pll2_clk0 with divider 21.
	 */
	if (i2s_clock_name == clock_i2s0) {
		clock_set_source_divider(clock_aud0, clk_src_pll2_clk0, 21U);
	} else if (i2s_clock_name == clock_i2s1) {
		clock_set_source_divider(clock_aud1, clk_src_pll2_clk0, 21U);
	}
}
#else
static void hpmicro_i2s_prepare_audio_root(uint32_t i2s_clock_name)
{
	ARG_UNUSED(i2s_clock_name);
}
#endif

#ifdef CONFIG_I2S_HPMICRO_DMA
static void hpmicro_i2s_tx_dma_work_handler(struct k_work *work);
static void hpmicro_i2s_rx_dma_work_handler(struct k_work *work);
static void hpmicro_i2s_tx_dma_watchdog_handler(struct k_work *work);
static void hpmicro_i2s_rx_dma_watchdog_handler(struct k_work *work);

static void hpmicro_i2s_dma_cb(const struct device *dev, void *arg,
			       uint32_t channel, int status)
{
	struct hpmicro_i2s_dma_chan *dma_chan = arg;

	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	dma_chan->cb_status = status;
	dma_chan->done = true;
	if (dma_chan->owner != NULL) {
		if (dma_chan->is_tx) {
			(void)k_work_submit(&dma_chan->owner->tx_dma_work);
		} else {
			(void)k_work_submit(&dma_chan->owner->rx_dma_work);
		}
	}
	k_sem_give(&dma_chan->sync);
}

static bool hpmicro_i2s_txq_push(struct hpmicro_i2s_data *data, void *mem_block, size_t size)
{
	uint8_t tail;

	if (data->tx_q_count >= HPMICRO_I2S_TX_QUEUE_DEPTH) {
		return false;
	}

	tail = data->tx_q_tail;
	data->tx_queue[tail].mem_block = mem_block;
	data->tx_queue[tail].size = size;
	data->tx_q_tail = (tail + 1U) % HPMICRO_I2S_TX_QUEUE_DEPTH;
	data->tx_q_count++;
	return true;
}

static bool hpmicro_i2s_txq_pop(struct hpmicro_i2s_data *data, struct hpmicro_i2s_tx_block *out)
{
	uint8_t head;

	if (data->tx_q_count == 0U) {
		return false;
	}

	head = data->tx_q_head;
	*out = data->tx_queue[head];
	data->tx_q_head = (head + 1U) % HPMICRO_I2S_TX_QUEUE_DEPTH;
	data->tx_q_count--;
	return true;
}

static void hpmicro_i2s_txq_reset(struct hpmicro_i2s_data *data)
{
	data->tx_q_head = 0U;
	data->tx_q_tail = 0U;
	data->tx_q_count = 0U;
}

static bool hpmicro_i2s_rxq_push(struct hpmicro_i2s_data *data, void *mem_block, size_t size)
{
	uint8_t tail;

	if (data->rx_q_count >= HPMICRO_I2S_RX_QUEUE_DEPTH) {
		return false;
	}

	tail = data->rx_q_tail;
	data->rx_done_queue[tail].mem_block = mem_block;
	data->rx_done_queue[tail].size = size;
	data->rx_q_tail = (tail + 1U) % HPMICRO_I2S_RX_QUEUE_DEPTH;
	data->rx_q_count++;
	return true;
}

static bool hpmicro_i2s_rxq_pop(struct hpmicro_i2s_data *data, struct hpmicro_i2s_rx_block *out)
{
	uint8_t head;

	if (data->rx_q_count == 0U) {
		return false;
	}

	head = data->rx_q_head;
	*out = data->rx_done_queue[head];
	data->rx_q_head = (head + 1U) % HPMICRO_I2S_RX_QUEUE_DEPTH;
	data->rx_q_count--;
	return true;
}

static void hpmicro_i2s_rxq_reset(struct hpmicro_i2s_data *data)
{
	data->rx_q_head = 0U;
	data->rx_q_tail = 0U;
	data->rx_q_count = 0U;
}

static void hpmicro_i2s_txq_free_all_locked(struct hpmicro_i2s_data *data,
					    struct hpmicro_i2s_stream *stream)
{
	struct hpmicro_i2s_tx_block blk;

	while (hpmicro_i2s_txq_pop(data, &blk)) {
		if (stream->cfg.mem_slab != NULL) {
			k_mem_slab_free(stream->cfg.mem_slab, blk.mem_block);
		}
	}
	hpmicro_i2s_txq_reset(data);
}

static void hpmicro_i2s_rxq_free_all_locked(struct hpmicro_i2s_data *data,
					    struct hpmicro_i2s_stream *stream)
{
	struct hpmicro_i2s_rx_block blk;

	while (hpmicro_i2s_rxq_pop(data, &blk)) {
		if (stream->cfg.mem_slab != NULL) {
			k_mem_slab_free(stream->cfg.mem_slab, blk.mem_block);
		}
	}
	hpmicro_i2s_rxq_reset(data);
}

static uint32_t hpmicro_i2s_dma_width_from_word(uint32_t word_size)
{
	if (word_size <= 8U) {
		return 1U;
	}

	if (word_size <= 16U) {
		return 2U;
	}

	return 4U;
}

static uint32_t hpmicro_i2s_dma_data_shift_from_word(uint32_t word_size)
{
	/*
	 * Align with SDK i2s_dma sample:
	 * for 16-bit audio, DMA writes/reads upper half-word of TXD/RXD.
	 */
	return (word_size <= 16U) ? 2U : 0U;
}

static __maybe_unused int hpmicro_i2s_do_dma_xfer(const struct hpmicro_i2s_config *dev_cfg,
						  struct hpmicro_i2s_stream *stream,
						  struct hpmicro_i2s_dma_chan *dma_chan,
						  enum i2s_dir dir, void *buf, size_t size)
{
	uint32_t width;
	uint32_t data_shift;
	int ret;
	k_timeout_t timeout;

	if (!dma_chan->available || (dma_chan->dma_dev == NULL)) {
		return -ENODEV;
	}

	width = hpmicro_i2s_dma_width_from_word(stream->cfg.word_size);
	data_shift = hpmicro_i2s_dma_data_shift_from_word(stream->cfg.word_size);
	(void)memset(&dma_chan->dma_cfg, 0, sizeof(dma_chan->dma_cfg));
	(void)memset(&dma_chan->blk_cfg, 0, sizeof(dma_chan->blk_cfg));

	dma_chan->dma_cfg.channel_direction = (dir == I2S_DIR_TX) ?
					      MEMORY_TO_PERIPHERAL :
					      PERIPHERAL_TO_MEMORY;
	dma_chan->dma_cfg.dma_callback = hpmicro_i2s_dma_cb;
	dma_chan->dma_cfg.user_data = dma_chan;
	dma_chan->dma_cfg.block_count = 1;
	dma_chan->dma_cfg.head_block = &dma_chan->blk_cfg;
	dma_chan->dma_cfg.source_data_size = width;
	dma_chan->dma_cfg.dest_data_size = width;
	dma_chan->dma_cfg.source_burst_length = 1U;
	dma_chan->dma_cfg.dest_burst_length = 1U;
	dma_chan->dma_cfg.dma_slot = dma_chan->slot;
	dma_chan->dma_cfg.complete_callback_en = 1;
	dma_chan->dma_cfg.error_callback_dis = 0;

	if (dir == I2S_DIR_TX) {
		dma_chan->blk_cfg.source_address = (uint32_t)(uintptr_t)buf;
		dma_chan->blk_cfg.dest_address =
			(uint32_t)(uintptr_t)&dev_cfg->base->TXD[I2S_DATA_LINE_0] + data_shift;
		dma_chan->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_chan->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dma_chan->blk_cfg.source_address =
			(uint32_t)(uintptr_t)&dev_cfg->base->RXD[I2S_DATA_LINE_0] + data_shift;
		dma_chan->blk_cfg.dest_address = (uint32_t)(uintptr_t)buf;
		dma_chan->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_chan->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}
	dma_chan->blk_cfg.block_size = size;

	ret = dma_config(dma_chan->dma_dev, dma_chan->channel, &dma_chan->dma_cfg);
	if (ret != 0) {
		return ret;
	}

	dma_chan->cb_status = 0;
	k_sem_reset(&dma_chan->sync);
	ret = dma_start(dma_chan->dma_dev, dma_chan->channel);
	if (ret != 0) {
		return ret;
	}

	if (stream->cfg.timeout == SYS_FOREVER_MS) {
		timeout = K_FOREVER;
	} else if (stream->cfg.timeout <= 0) {
		timeout = K_NO_WAIT;
	} else {
		timeout = K_MSEC(stream->cfg.timeout);
	}

	ret = k_sem_take(&dma_chan->sync, timeout);
	if (ret != 0) {
		(void)dma_stop(dma_chan->dma_dev, dma_chan->channel);
		return (stream->cfg.timeout <= 0) ? -EBUSY : -EAGAIN;
	}

	if (dma_chan->cb_status != 0) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_i2s_start_dma_xfer_async(const struct hpmicro_i2s_config *dev_cfg,
					    struct hpmicro_i2s_stream *stream,
					    struct hpmicro_i2s_dma_chan *dma_chan,
					    enum i2s_dir dir, void *buf, size_t size)
{
	uint32_t width;
	uint32_t data_shift;
	int ret;

	if (!dma_chan->available || (dma_chan->dma_dev == NULL)) {
		return -ENODEV;
	}

	width = hpmicro_i2s_dma_width_from_word(stream->cfg.word_size);
	data_shift = hpmicro_i2s_dma_data_shift_from_word(stream->cfg.word_size);
	(void)memset(&dma_chan->dma_cfg, 0, sizeof(dma_chan->dma_cfg));
	(void)memset(&dma_chan->blk_cfg, 0, sizeof(dma_chan->blk_cfg));

	dma_chan->dma_cfg.channel_direction = (dir == I2S_DIR_TX) ?
					      MEMORY_TO_PERIPHERAL :
					      PERIPHERAL_TO_MEMORY;
	dma_chan->dma_cfg.dma_callback = hpmicro_i2s_dma_cb;
	dma_chan->dma_cfg.user_data = dma_chan;
	dma_chan->dma_cfg.block_count = 1;
	dma_chan->dma_cfg.head_block = &dma_chan->blk_cfg;
	dma_chan->dma_cfg.source_data_size = width;
	dma_chan->dma_cfg.dest_data_size = width;
	dma_chan->dma_cfg.source_burst_length = 1U;
	dma_chan->dma_cfg.dest_burst_length = 1U;
	dma_chan->dma_cfg.dma_slot = dma_chan->slot;
	dma_chan->dma_cfg.complete_callback_en = 1;
	dma_chan->dma_cfg.error_callback_dis = 0;

	if (dir == I2S_DIR_TX) {
		dma_chan->blk_cfg.source_address = (uint32_t)(uintptr_t)buf;
		dma_chan->blk_cfg.dest_address =
			(uint32_t)(uintptr_t)&dev_cfg->base->TXD[I2S_DATA_LINE_0] + data_shift;
		dma_chan->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_chan->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dma_chan->blk_cfg.source_address =
			(uint32_t)(uintptr_t)&dev_cfg->base->RXD[I2S_DATA_LINE_0] + data_shift;
		dma_chan->blk_cfg.dest_address = (uint32_t)(uintptr_t)buf;
		dma_chan->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_chan->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}
	dma_chan->blk_cfg.block_size = size;

	dma_chan->cb_status = 0;
	dma_chan->done = false;

	ret = dma_config(dma_chan->dma_dev, dma_chan->channel, &dma_chan->dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(dma_chan->dma_dev, dma_chan->channel);
	if (ret != 0) {
		return ret;
	}

	if (dma_chan->owner != NULL) {
		if (dir == I2S_DIR_TX) {
			(void)k_work_schedule(&dma_chan->owner->tx_dma_watchdog_work,
					      K_MSEC(HPMICRO_I2S_TX_WATCHDOG_MS));
		} else {
			(void)k_work_schedule(&dma_chan->owner->rx_dma_watchdog_work,
					      K_MSEC(HPMICRO_I2S_RX_WATCHDOG_MS));
		}
	}

	return 0;
}

static void hpmicro_i2s_tx_dma_watchdog_handler(struct k_work *work)
{
	struct hpmicro_i2s_data *data =
		CONTAINER_OF(k_work_delayable_from_work(work),
			     struct hpmicro_i2s_data, tx_dma_watchdog_work);
	struct dma_status stat = {0};
	bool active = false;
	bool mark_done = false;

	k_mutex_lock(&data->lock, K_FOREVER);
	active = data->tx_dma_active && !data->dma_tx.done &&
		 (data->dma_tx.dma_dev != NULL);
	k_mutex_unlock(&data->lock);

	if (!active) {
		return;
	}

	if (dma_get_status(data->dma_tx.dma_dev, data->dma_tx.channel, &stat) == 0) {
		if ((!stat.busy) || (stat.pending_length == 0U)) {
			mark_done = true;
		}
	}

	if (mark_done) {
		k_mutex_lock(&data->lock, K_FOREVER);
		if (data->tx_dma_active && !data->dma_tx.done) {
			data->dma_tx.cb_status = 0;
			data->dma_tx.done = true;
		}
		k_mutex_unlock(&data->lock);
		(void)k_work_submit(&data->tx_dma_work);
		return;
	}

	(void)k_work_schedule(&data->tx_dma_watchdog_work,
			      K_MSEC(HPMICRO_I2S_TX_WATCHDOG_MS));
}

static void hpmicro_i2s_rx_dma_watchdog_handler(struct k_work *work)
{
	struct hpmicro_i2s_data *data =
		CONTAINER_OF(k_work_delayable_from_work(work),
			     struct hpmicro_i2s_data, rx_dma_watchdog_work);
	struct dma_status stat = {0};
	bool active = false;
	bool mark_done = false;

	k_mutex_lock(&data->lock, K_FOREVER);
	active = data->rx_dma_active && !data->dma_rx.done &&
		 (data->dma_rx.dma_dev != NULL);
	k_mutex_unlock(&data->lock);

	if (!active) {
		return;
	}

	if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.channel, &stat) == 0) {
		if ((!stat.busy) || (stat.pending_length == 0U)) {
			mark_done = true;
		}
	}

	if (mark_done) {
		k_mutex_lock(&data->lock, K_FOREVER);
		if (data->rx_dma_active && !data->dma_rx.done) {
			data->dma_rx.cb_status = 0;
			data->dma_rx.done = true;
		}
		k_mutex_unlock(&data->lock);
		(void)k_work_submit(&data->rx_dma_work);
		return;
	}

	(void)k_work_schedule(&data->rx_dma_watchdog_work,
			      K_MSEC(HPMICRO_I2S_RX_WATCHDOG_MS));
}

static void hpmicro_i2s_tx_dma_work_handler(struct k_work *work)
{
	struct hpmicro_i2s_data *data = CONTAINER_OF(work, struct hpmicro_i2s_data, tx_dma_work);
	const struct device *dev = data->dev;
	const struct hpmicro_i2s_config *dev_cfg = dev->config;
	struct hpmicro_i2s_stream *stream = &data->tx;
	struct hpmicro_i2s_tx_block next_blk;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->dma_tx.done && data->tx_dma_active) {
		data->dma_tx.done = false;
		(void)k_work_cancel_delayable(&data->tx_dma_watchdog_work);

		if ((stream->cfg.mem_slab != NULL) && (data->tx_inflight_block != NULL)) {
			k_mem_slab_free(stream->cfg.mem_slab, data->tx_inflight_block);
		}

		if (data->dma_tx.cb_status != 0) {
			stream->state = I2S_STATE_ERROR;
			data->tx_last_err = -EIO;
			hpmicro_i2s_txq_free_all_locked(data, stream);
		}

		data->tx_inflight_block = NULL;
		data->tx_inflight_size = 0U;
		data->tx_dma_active = false;
	}

	if ((stream->state == I2S_STATE_RUNNING) && (data->tx_last_err == 0) &&
	    !data->tx_dma_active && hpmicro_i2s_txq_pop(data, &next_blk)) {
		ret = hpmicro_i2s_start_dma_xfer_async(dev_cfg, stream, &data->dma_tx,
						       I2S_DIR_TX, next_blk.mem_block, next_blk.size);
		if (ret != 0) {
			stream->state = I2S_STATE_ERROR;
			data->tx_last_err = ret;
			if (stream->cfg.mem_slab != NULL) {
				k_mem_slab_free(stream->cfg.mem_slab, next_blk.mem_block);
			}
			hpmicro_i2s_txq_free_all_locked(data, stream);
		} else {
			data->tx_inflight_block = next_blk.mem_block;
			data->tx_inflight_size = next_blk.size;
			data->tx_dma_active = true;
		}
	}

	k_mutex_unlock(&data->lock);
}

static void hpmicro_i2s_rx_dma_work_handler(struct k_work *work)
{
	struct hpmicro_i2s_data *data = CONTAINER_OF(work, struct hpmicro_i2s_data, rx_dma_work);
	const struct device *dev = data->dev;
	const struct hpmicro_i2s_config *dev_cfg = dev->config;
	struct hpmicro_i2s_stream *stream = &data->rx;
	void *next_block = NULL;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->dma_rx.done && data->rx_dma_active) {
		data->dma_rx.done = false;
		(void)k_work_cancel_delayable(&data->rx_dma_watchdog_work);

		if (data->dma_rx.cb_status != 0) {
			stream->state = I2S_STATE_ERROR;
			data->rx_last_err = -EIO;
			if ((stream->cfg.mem_slab != NULL) && (data->rx_inflight_block != NULL)) {
				k_mem_slab_free(stream->cfg.mem_slab, data->rx_inflight_block);
			}
		} else if (!hpmicro_i2s_rxq_push(data, data->rx_inflight_block, data->rx_inflight_size)) {
			/* Queue full: drop the newest block to preserve driver stability. */
			if (stream->cfg.mem_slab != NULL) {
				k_mem_slab_free(stream->cfg.mem_slab, data->rx_inflight_block);
			}
		}

		data->rx_inflight_block = NULL;
		data->rx_inflight_size = 0U;
		data->rx_dma_active = false;
	}

	if ((stream->state == I2S_STATE_RUNNING) && (data->rx_last_err == 0) &&
	    !data->rx_dma_active && (stream->cfg.mem_slab != NULL)) {
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &next_block, K_NO_WAIT);
		if (ret == 0) {
			ret = hpmicro_i2s_start_dma_xfer_async(dev_cfg, stream, &data->dma_rx,
						       I2S_DIR_RX, next_block, stream->cfg.block_size);
			if (ret != 0) {
				data->rx_last_err = ret;
				stream->state = I2S_STATE_ERROR;
				k_mem_slab_free(stream->cfg.mem_slab, next_block);
			} else {
				data->rx_inflight_block = next_block;
				data->rx_inflight_size = stream->cfg.block_size;
				data->rx_dma_active = true;
			}
		}
	}

	k_mutex_unlock(&data->lock);
}
#endif

static struct hpmicro_i2s_stream *stream_from_dir(struct hpmicro_i2s_data *data,
						  enum i2s_dir dir)
{
	if (dir == I2S_DIR_TX) {
		return &data->tx;
	}

	if (dir == I2S_DIR_RX) {
		return &data->rx;
	}

	return NULL;
}

static int hpmicro_i2s_protocol_from_fmt(i2s_fmt_t fmt)
{
	switch (fmt & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		return I2S_PROTOCOL_I2S_PHILIPS;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		return I2S_PROTOCOL_PCM;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		return I2S_PROTOCOL_LEFT_JUSTIFIED;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		return I2S_PROTOCOL_RIGHT_JUSTIFIED;
	default:
		return -EINVAL;
	}
}

static int hpmicro_i2s_apply_hw_config(const struct hpmicro_i2s_config *dev_cfg,
				       enum i2s_dir dir, const struct i2s_config *cfg)
{
	i2s_transfer_config_t transfer_cfg;
	uint32_t mclk_hz = clock_get_frequency(dev_cfg->clock_name);
	int protocol;
	bool slave_mode;
	hpm_stat_t stat;

	if ((cfg->word_size != 16U) && (cfg->word_size != 24U) && (cfg->word_size != 32U)) {
		return -EINVAL;
	}

	if ((cfg->channels == 0U) || (cfg->channels > 2U)) {
		return -EINVAL;
	}

	protocol = hpmicro_i2s_protocol_from_fmt(cfg->format);
	if (protocol < 0) {
		return protocol;
	}

	slave_mode = ((cfg->options & I2S_OPT_BIT_CLK_SLAVE) != 0U) ||
		     ((cfg->options & I2S_OPT_FRAME_CLK_SLAVE) != 0U);

	i2s_get_default_transfer_config(&transfer_cfg);
	transfer_cfg.sample_rate = cfg->frame_clk_freq;
	transfer_cfg.audio_depth = cfg->word_size;
	transfer_cfg.channel_num_per_frame = cfg->channels;
	transfer_cfg.channel_length = (cfg->word_size <= 16U) ? 16U : 32U;
	transfer_cfg.master_mode = !slave_mode;
	transfer_cfg.protocol = (uint8_t)protocol;
	transfer_cfg.data_line = I2S_DATA_LINE_0;
	transfer_cfg.channel_slot_mask = (1U << cfg->channels) - 1U;

	if (dir == I2S_DIR_TX) {
		stat = slave_mode ? i2s_config_tx_slave(dev_cfg->base, &transfer_cfg)
				  : i2s_config_tx(dev_cfg->base, mclk_hz, &transfer_cfg);
	} else {
		stat = slave_mode ? i2s_config_rx_slave(dev_cfg->base, &transfer_cfg)
				  : i2s_config_rx(dev_cfg->base, mclk_hz, &transfer_cfg);
	}

	return (stat == status_success) ? 0 : -EIO;
}

static int hpmicro_i2s_configure_one(struct hpmicro_i2s_stream *stream,
				     const struct i2s_config *cfg)
{
	if (cfg->frame_clk_freq == 0U) {
		stream->configured = false;
		stream->state = I2S_STATE_NOT_READY;
		(void)memset(&stream->cfg, 0, sizeof(stream->cfg));
		return 0;
	}

	stream->cfg = *cfg;
	stream->configured = true;
	stream->state = I2S_STATE_READY;
	return 0;
}

static int hpmicro_i2s_configure(const struct device *dev, enum i2s_dir dir,
				 const struct i2s_config *cfg)
{
	const struct hpmicro_i2s_config *dev_cfg = dev->config;
	struct hpmicro_i2s_data *data = dev->data;
	int ret = 0;

	if ((cfg == NULL) || ((dir != I2S_DIR_RX) && (dir != I2S_DIR_TX) &&
			      (dir != I2S_DIR_BOTH))) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	hpmicro_i2s_prepare_audio_root(dev_cfg->clock_name);
	clock_set_source_divider(dev_cfg->clock_name, dev_cfg->clock_src, dev_cfg->clock_div);
	clock_add_to_group(dev_cfg->clock_name, 0);

	if (dir == I2S_DIR_BOTH) {
		ret = hpmicro_i2s_configure_one(&data->tx, cfg);
		if (ret == 0) {
			ret = hpmicro_i2s_apply_hw_config(dev_cfg, I2S_DIR_TX, cfg);
		}
		if (ret == 0) {
			ret = hpmicro_i2s_configure_one(&data->rx, cfg);
		}
		if (ret == 0) {
			ret = hpmicro_i2s_apply_hw_config(dev_cfg, I2S_DIR_RX, cfg);
		}
	} else {
		struct hpmicro_i2s_stream *stream = stream_from_dir(data, dir);

		if (stream == NULL) {
			ret = -EINVAL;
		} else {
			ret = hpmicro_i2s_configure_one(stream, cfg);
			if (ret == 0) {
				ret = hpmicro_i2s_apply_hw_config(dev_cfg, dir, cfg);
			}
		}
	}

#ifdef CONFIG_I2S_HPMICRO_DMA
	if ((ret == 0) && ((dir == I2S_DIR_TX) || (dir == I2S_DIR_BOTH))) {
		data->tx_last_err = 0;
	}
	if ((ret == 0) && ((dir == I2S_DIR_RX) || (dir == I2S_DIR_BOTH))) {
		data->rx_last_err = 0;
	}
#endif

	k_mutex_unlock(&data->lock);
	return ret;
}

static const struct i2s_config *hpmicro_i2s_config_get(const struct device *dev,
							enum i2s_dir dir)
{
	const struct hpmicro_i2s_data *data = dev->data;
	const struct hpmicro_i2s_stream *stream;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return NULL;
	}

	return stream->configured ? &stream->cfg : NULL;
}

static int hpmicro_i2s_read(const struct device *dev, void **mem_block, size_t *size)
{
	const struct hpmicro_i2s_config *dev_cfg = dev->config;
	struct hpmicro_i2s_data *data = dev->data;
	struct hpmicro_i2s_stream *stream = &data->rx;
	int ret;
	uint8_t *dst;
	void *block;
	size_t copied = 0;

	if ((mem_block == NULL) || (size == NULL)) {
		return -EINVAL;
	}

	if (!stream->configured || (stream->cfg.mem_slab == NULL)) {
		return -EIO;
	}

	if ((stream->state != I2S_STATE_READY) && (stream->state != I2S_STATE_RUNNING)) {
		return -EIO;
	}

#ifdef CONFIG_I2S_HPMICRO_DMA
	if (data->dma_rx.available) {
		int32_t wait_ms;
		int32_t elapsed_ms = 0;
		struct hpmicro_i2s_rx_block rx_blk;

		if (stream->state == I2S_STATE_READY) {
			i2s_reset_rx(dev_cfg->base);
			i2s_enable_rx_line(dev_cfg->base, I2S_DATA_LINE_0);
			i2s_enable_rx_dma_request(dev_cfg->base);
			i2s_enable(dev_cfg->base);
			stream->state = I2S_STATE_RUNNING;
			(void)k_work_submit(&data->rx_dma_work);
		}

		wait_ms = stream->cfg.timeout;
		while (true) {
			k_mutex_lock(&data->lock, K_FOREVER);
			if (hpmicro_i2s_rxq_pop(data, &rx_blk)) {
				k_mutex_unlock(&data->lock);
				*mem_block = rx_blk.mem_block;
				*size = rx_blk.size;
				return 0;
			}

			if (data->rx_last_err != 0) {
				ret = data->rx_last_err;
				k_mutex_unlock(&data->lock);
				return ret;
			}
			k_mutex_unlock(&data->lock);

			if (wait_ms == 0) {
				return -EBUSY;
			}

			if (wait_ms != SYS_FOREVER_MS) {
				if (elapsed_ms >= wait_ms) {
					return -EAGAIN;
				}
				k_sleep(K_MSEC(1));
				elapsed_ms++;
			} else {
				k_sleep(K_MSEC(1));
			}
		}
	}
#endif

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &block,
			       SYS_TIMEOUT_MS(stream->cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	dst = (uint8_t *)block;
	while (copied < stream->cfg.block_size) {
		uint32_t fifo_stat = i2s_check_data_line_status(dev_cfg->base, I2S_DATA_LINE_0);
		uint32_t chunk = i2s_receive_buff(dev_cfg->base, I2S_DATA_LINE_0,
						  stream->cfg.word_size,
						  &dst[copied],
						  stream->cfg.block_size - copied);

		if (fifo_stat & i2s_data_line_rx_fifo_overrun) {
			stream->state = I2S_STATE_ERROR;
			k_mem_slab_free(stream->cfg.mem_slab, block);
			return -EIO;
		}

		if (chunk > 0U) {
			copied += chunk;
			continue;
		}

		if (stream->cfg.timeout == 0) {
			k_mem_slab_free(stream->cfg.mem_slab, block);
			return -EBUSY;
		}

		if (stream->cfg.timeout != SYS_FOREVER_MS) {
			k_mem_slab_free(stream->cfg.mem_slab, block);
			return -EAGAIN;
		}

		k_yield();
	}

	*mem_block = block;
	*size = copied;
	return 0;
}

static int hpmicro_i2s_push_tx(const struct hpmicro_i2s_config *dev_cfg,
			       struct hpmicro_i2s_stream *stream,
			       uint8_t *src, size_t size)
{
	size_t sent = 0;

	while (sent < size) {
		uint32_t fifo_stat = i2s_check_data_line_status(dev_cfg->base, I2S_DATA_LINE_0);
		uint32_t chunk = i2s_send_buff(dev_cfg->base, I2S_DATA_LINE_0,
					       stream->cfg.word_size, &src[sent],
					       size - sent);

		if (fifo_stat & i2s_data_line_tx_fifo_underrun) {
			stream->state = I2S_STATE_ERROR;
			return -EIO;
		}

		if (chunk > 0U) {
			sent += chunk;
			continue;
		}

		if (stream->cfg.timeout == 0) {
			return -EBUSY;
		}

		if (stream->cfg.timeout != SYS_FOREVER_MS) {
			return -EAGAIN;
		}

		k_yield();
	}

	return 0;
}

static int hpmicro_i2s_write(const struct device *dev, void *mem_block, size_t size)
{
	const struct hpmicro_i2s_config *dev_cfg = dev->config;
	struct hpmicro_i2s_data *data = dev->data;
	struct hpmicro_i2s_stream *stream = &data->tx;
	int ret;

	if ((mem_block == NULL) || (size == 0U)) {
		return -EINVAL;
	}

	if (!stream->configured || (stream->cfg.mem_slab == NULL)) {
		return -EIO;
	}

	if (size > stream->cfg.block_size) {
		return -EINVAL;
	}

	if ((stream->state != I2S_STATE_READY) && (stream->state != I2S_STATE_RUNNING)) {
		return -EIO;
	}

	if (stream->state == I2S_STATE_READY) {
		i2s_reset_tx(dev_cfg->base);
		(void)i2s_fill_tx_dummy_data(dev_cfg->base, I2S_DATA_LINE_0,
					     (stream->cfg.channels == 0U) ? 1U : stream->cfg.channels);
		i2s_enable_tx_line(dev_cfg->base, I2S_DATA_LINE_0);
#ifdef CONFIG_I2S_HPMICRO_DMA
		if (data->dma_tx.available) {
			i2s_enable_tx_dma_request(dev_cfg->base);
		}
#endif
		i2s_enable(dev_cfg->base);
		stream->state = I2S_STATE_RUNNING;
	}

#ifdef CONFIG_I2S_HPMICRO_DMA
	if (data->dma_tx.available) {
		if (data->tx_last_err != 0) {
			return data->tx_last_err;
		}

		k_mutex_lock(&data->lock, K_FOREVER);
		if (!hpmicro_i2s_txq_push(data, mem_block, size)) {
			k_mutex_unlock(&data->lock);
			return (stream->cfg.timeout == 0) ? -EBUSY : -EAGAIN;
		}
		k_mutex_unlock(&data->lock);

		(void)k_work_submit(&data->tx_dma_work);
		return 0;
	}
#endif

	ret = hpmicro_i2s_push_tx(dev_cfg, stream, (uint8_t *)mem_block, size);
	k_mem_slab_free(stream->cfg.mem_slab, mem_block);
	return ret;
}

static int hpmicro_i2s_trigger_one(const struct hpmicro_i2s_config *dev_cfg,
				   struct hpmicro_i2s_data *data,
				   struct hpmicro_i2s_stream *stream,
				   enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			return -EIO;
		}
		if (dir == I2S_DIR_TX) {
			i2s_reset_tx(dev_cfg->base);
			(void)i2s_fill_tx_dummy_data(dev_cfg->base, I2S_DATA_LINE_0,
						     (stream->cfg.channels == 0U) ? 1U : stream->cfg.channels);
			i2s_enable_tx_line(dev_cfg->base, I2S_DATA_LINE_0);
#ifdef CONFIG_I2S_HPMICRO_DMA
			if (data->dma_tx.available) {
				i2s_enable_tx_dma_request(dev_cfg->base);
			}
#endif
		} else {
			i2s_reset_rx(dev_cfg->base);
			i2s_enable_rx_line(dev_cfg->base, I2S_DATA_LINE_0);
#ifdef CONFIG_I2S_HPMICRO_DMA
			if (data->dma_rx.available) {
				data->rx_last_err = 0;
				i2s_enable_rx_dma_request(dev_cfg->base);
			}
#endif
		}
		i2s_enable(dev_cfg->base);
		stream->state = I2S_STATE_RUNNING;
#ifdef CONFIG_I2S_HPMICRO_DMA
		if ((dir == I2S_DIR_TX) && data->dma_tx.available &&
		    (data->tx_q_count > 0U) && !data->tx_dma_active) {
			(void)k_work_submit(&data->tx_dma_work);
		}
		if ((dir == I2S_DIR_RX) && data->dma_rx.available &&
		    !data->rx_dma_active) {
			(void)k_work_submit(&data->rx_dma_work);
		}
#endif
		return 0;

	case I2S_TRIGGER_STOP:
	case I2S_TRIGGER_DRAIN:
		if (stream->state != I2S_STATE_RUNNING) {
			return -EIO;
		}
#ifdef CONFIG_I2S_HPMICRO_DMA
		if ((dir == I2S_DIR_TX) && data->dma_tx.available) {
			if (cmd == I2S_TRIGGER_DRAIN) {
				int32_t elapsed_ms = 0;

				while ((data->tx_q_count > 0U) || data->tx_dma_active) {
					k_mutex_unlock(&data->lock);
					k_sleep(K_MSEC(1));
					k_mutex_lock(&data->lock, K_FOREVER);
					if (stream->state == I2S_STATE_ERROR) {
						return -EIO;
					}
					if (stream->cfg.timeout == 0) {
						return -EBUSY;
					}
					if ((stream->cfg.timeout != SYS_FOREVER_MS) &&
					    (elapsed_ms >= stream->cfg.timeout)) {
						return -EAGAIN;
					}
					elapsed_ms++;
				}
			} else {
				(void)dma_stop(data->dma_tx.dma_dev, data->dma_tx.channel);
				(void)k_work_cancel_delayable(&data->tx_dma_watchdog_work);
				hpmicro_i2s_txq_free_all_locked(data, stream);
				if ((stream->cfg.mem_slab != NULL) && (data->tx_inflight_block != NULL)) {
					k_mem_slab_free(stream->cfg.mem_slab,
							data->tx_inflight_block);
				}
				data->tx_inflight_block = NULL;
				data->tx_inflight_size = 0U;
				data->tx_dma_active = false;
			}
		}
#endif
		i2s_stop_transfer(dev_cfg->base);
		if (dir == I2S_DIR_TX) {
			i2s_disable_tx_line(dev_cfg->base, I2S_DATA_LINE_0);
#ifdef CONFIG_I2S_HPMICRO_DMA
			i2s_disable_tx_dma_request(dev_cfg->base);
#endif
		} else {
#ifdef CONFIG_I2S_HPMICRO_DMA
			if (data->dma_rx.available) {
				(void)dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
				(void)k_work_cancel_delayable(&data->rx_dma_watchdog_work);
				hpmicro_i2s_rxq_free_all_locked(data, stream);
				if ((stream->cfg.mem_slab != NULL) && (data->rx_inflight_block != NULL)) {
					k_mem_slab_free(stream->cfg.mem_slab, data->rx_inflight_block);
				}
				data->rx_inflight_block = NULL;
				data->rx_inflight_size = 0U;
				data->rx_dma_active = false;
			}
#endif
			i2s_disable_rx_line(dev_cfg->base, I2S_DATA_LINE_0);
#ifdef CONFIG_I2S_HPMICRO_DMA
			i2s_disable_rx_dma_request(dev_cfg->base);
#endif
		}
		stream->state = I2S_STATE_READY;
		return 0;

	case I2S_TRIGGER_DROP:
#ifdef CONFIG_I2S_HPMICRO_DMA
		if ((dir == I2S_DIR_TX) && data->dma_tx.available) {
			(void)dma_stop(data->dma_tx.dma_dev, data->dma_tx.channel);
			(void)k_work_cancel_delayable(&data->tx_dma_watchdog_work);
			hpmicro_i2s_txq_free_all_locked(data, stream);
			if ((stream->cfg.mem_slab != NULL) && (data->tx_inflight_block != NULL)) {
				k_mem_slab_free(stream->cfg.mem_slab,
						data->tx_inflight_block);
			}
			data->tx_inflight_block = NULL;
			data->tx_inflight_size = 0U;
			data->tx_dma_active = false;
		}
		if ((dir == I2S_DIR_RX) && data->dma_rx.available) {
			(void)dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
			(void)k_work_cancel_delayable(&data->rx_dma_watchdog_work);
			hpmicro_i2s_rxq_free_all_locked(data, stream);
			if ((stream->cfg.mem_slab != NULL) && (data->rx_inflight_block != NULL)) {
				k_mem_slab_free(stream->cfg.mem_slab, data->rx_inflight_block);
			}
			data->rx_inflight_block = NULL;
			data->rx_inflight_size = 0U;
			data->rx_dma_active = false;
		}
#endif
		i2s_stop_transfer(dev_cfg->base);
		i2s_disable_tx_line(dev_cfg->base, I2S_DATA_LINE_0);
		i2s_disable_rx_line(dev_cfg->base, I2S_DATA_LINE_0);
#ifdef CONFIG_I2S_HPMICRO_DMA
		i2s_disable_tx_dma_request(dev_cfg->base);
		i2s_disable_rx_dma_request(dev_cfg->base);
#endif
		stream->state = stream->configured ? I2S_STATE_READY : I2S_STATE_NOT_READY;
		return 0;

	case I2S_TRIGGER_PREPARE:
		if (!stream->configured) {
			return -EIO;
		}
		stream->state = I2S_STATE_READY;
		return 0;

	default:
		return -ENOTSUP;
	}
}

static int hpmicro_i2s_trigger(const struct device *dev, enum i2s_dir dir,
			       enum i2s_trigger_cmd cmd)
{
	const struct hpmicro_i2s_config *dev_cfg = dev->config;
	struct hpmicro_i2s_data *data = dev->data;
	int ret = 0;

	if ((dir != I2S_DIR_RX) && (dir != I2S_DIR_TX) && (dir != I2S_DIR_BOTH)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (dir == I2S_DIR_BOTH) {
		ret = hpmicro_i2s_trigger_one(dev_cfg, data, &data->tx, I2S_DIR_TX, cmd);
		if (ret == 0) {
			ret = hpmicro_i2s_trigger_one(dev_cfg, data, &data->rx, I2S_DIR_RX, cmd);
		}
	} else {
		struct hpmicro_i2s_stream *stream = stream_from_dir(data, dir);

		if (stream == NULL) {
			ret = -EINVAL;
		} else {
			ret = hpmicro_i2s_trigger_one(dev_cfg, data, stream, dir, cmd);
		}
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static int hpmicro_i2s_init(const struct device *dev)
{
	const struct hpmicro_i2s_config *cfg = dev->config;
	struct hpmicro_i2s_data *data = dev->data;
	i2s_config_t default_cfg;
	int ret;

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	hpmicro_i2s_prepare_audio_root(cfg->clock_name);
	clock_set_source_divider(cfg->clock_name, cfg->clock_src, cfg->clock_div);
	clock_add_to_group(cfg->clock_name, 0);
	i2s_get_default_config(cfg->base, &default_cfg);
	default_cfg.enable_mclk_out = true;
	i2s_init(cfg->base, &default_cfg);

	data->dev = dev;
	data->tx.state = I2S_STATE_NOT_READY;
	data->rx.state = I2S_STATE_NOT_READY;
	data->tx.configured = false;
	data->rx.configured = false;
	k_mutex_init(&data->lock);

#ifdef CONFIG_I2S_HPMICRO_DMA
	k_sem_init(&data->dma_tx.sync, 0, 1);
	k_sem_init(&data->dma_rx.sync, 0, 1);
	k_work_init(&data->tx_dma_work, hpmicro_i2s_tx_dma_work_handler);
	k_work_init(&data->rx_dma_work, hpmicro_i2s_rx_dma_work_handler);
	k_work_init_delayable(&data->tx_dma_watchdog_work,
			      hpmicro_i2s_tx_dma_watchdog_handler);
	k_work_init_delayable(&data->rx_dma_watchdog_work,
			      hpmicro_i2s_rx_dma_watchdog_handler);

	if ((data->dma_tx.dma_dev != NULL) && !device_is_ready(data->dma_tx.dma_dev)) {
		return -ENODEV;
	}
	if ((data->dma_rx.dma_dev != NULL) && !device_is_ready(data->dma_rx.dma_dev)) {
		return -ENODEV;
	}
	data->dma_tx.owner = data;
	data->dma_tx.is_tx = true;
	data->dma_tx.done = false;
	data->dma_rx.owner = data;
	data->dma_rx.is_tx = false;
	data->dma_rx.done = false;
	data->dma_tx.available = (data->dma_tx.dma_dev != NULL);
	data->dma_rx.available = (data->dma_rx.dma_dev != NULL);
	data->tx_dma_active = false;
	data->tx_inflight_block = NULL;
	data->tx_inflight_size = 0U;
	data->tx_last_err = 0;
	hpmicro_i2s_txq_reset(data);
	data->rx_dma_active = false;
	data->rx_inflight_block = NULL;
	data->rx_inflight_size = 0U;
	data->rx_last_err = 0;
	hpmicro_i2s_rxq_reset(data);
#endif

	return 0;
}

static const struct i2s_driver_api hpmicro_i2s_api = {
	.configure = hpmicro_i2s_configure,
	.config_get = hpmicro_i2s_config_get,
	.read = hpmicro_i2s_read,
	.write = hpmicro_i2s_write,
	.trigger = hpmicro_i2s_trigger,
};

#ifdef CONFIG_I2S_HPMICRO_DMA
#define I2S_DMA_CONFIG_INIT(n)						\
	.dma_tx = {							\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),\
		.channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),	\
		.slot = DT_INST_DMAS_CELL_BY_NAME(n, tx, source),	\
	},								\
	.dma_rx = {							\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)),\
		.channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),	\
		.slot = DT_INST_DMAS_CELL_BY_NAME(n, rx, source),	\
	},
#else
#define I2S_DMA_CONFIG_INIT(n)
#endif

#define HPM_I2S_INIT(idx)							    \
	PINCTRL_DT_INST_DEFINE(idx);						    \
										    \
	static const struct hpmicro_i2s_config hpmicro_i2s_cfg_##idx = {	    \
		.base = (I2S_Type *)DT_INST_REG_ADDR(idx),			    \
		.clock_name = DT_INST_CLOCKS_CELL(idx, name),			    \
		.clock_src = DT_INST_CLOCKS_CELL(idx, src),			    \
		.clock_div = DT_INST_CLOCKS_CELL(idx, div),			    \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),			    \
	};									    \
										    \
	static struct hpmicro_i2s_data hpmicro_i2s_data_##idx = {		    \
		I2S_DMA_CONFIG_INIT(idx)					    \
	};									    \
										    \
	DEVICE_DT_INST_DEFINE(idx, hpmicro_i2s_init, NULL,			    \
			      &hpmicro_i2s_data_##idx,			    \
			      &hpmicro_i2s_cfg_##idx, POST_KERNEL,		    \
			      CONFIG_I2S_INIT_PRIORITY, &hpmicro_i2s_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_I2S_INIT)
