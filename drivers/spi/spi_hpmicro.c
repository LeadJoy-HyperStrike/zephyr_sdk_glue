/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_spi

#include <errno.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <hpm_common.h>
#include <hpm_spi_drv.h>
#include <hpm_clock_drv.h>

#ifdef CONFIG_SPI_HPM_SPI_DMA
#include <zephyr/drivers/dma.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_hpmicro);

#ifdef CONFIG_SPI_HPM_SPI_DMA
#include <hpm_l1c_drv.h>
#endif

#include "spi/spi_context.h"

#define MAX_DATA_WIDTH		32  /* data width 1-32 bits */

struct spi_hpm_config {
	SPI_Type *base;
	uint32_t clock_name;
	uint32_t clock_src;
	uint32_t clock_div;
	void (*irq_config_func)(const struct device *dev);
	uint32_t cs2sclk;
	uint32_t csht;
	const struct pinctrl_dev_config *pincfg;
};

#ifdef CONFIG_SPI_HPM_SPI_DMA
#define SPI_HPM_SPI_DMA_ERROR_FLAG		0x01
#define SPI_HPM_SPI_DMA_RX_DONE_FLAG	0x02
#define SPI_HPM_SPI_DMA_TX_DONE_FLAG	0x04
#define SPI_HPM_SPI_DMA_DONE_FLAG		(SPI_HPM_SPI_DMA_RX_DONE_FLAG | SPI_HPM_SPI_DMA_TX_DONE_FLAG)

struct stream {
	const struct device *dma_dev;
	uint32_t channel; /* stores the channel for dma */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};
#endif

struct spi_hpm_data {
	const struct device *dev;
	spi_control_config_t control_config;
	struct spi_context ctx;
	size_t transfer_len;
	uint8_t slave_last_cmd;
#ifdef CONFIG_SPI_HPM_INTERRUPT
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	uint32_t tx_count;
	uint32_t rx_count;
#endif
#ifdef CONFIG_SPI_HPM_SPI_DMA
	volatile uint32_t status_flags;
	uint32_t dma_expected_flags;
	size_t dma_chunk_tx;
	size_t dma_chunk_rx;
	struct stream dma_rx;
	struct stream dma_tx;
	/* dummy value used for transferring NOP when tx buf is null */
	uint32_t dummy_tx_buffer;
	/* dummy value used to read RX data into when rx buf is null */
	uint32_t dummy_rx_buffer;
	uint8_t dma_slave_cmd;
	uint32_t dma_slave_addr;
	/* Complete DMA xfer on SPI end IRQ (SDK DMA slave style). */
	bool dma_use_spi_end;
#endif
};

static void spi_hpm_master_transfer_callback(SPI_Type *base, void *userData);

#if defined(CONFIG_SPI_HPM_SPI_DMA) && defined(CONFIG_SPI_ASYNC)
static void spi_hpm_dma_async_finalize(struct spi_hpm_data *data, int status);
#endif

#ifdef CONFIG_SPI_HPM_SPI_DMA
/* Zephyr CACHE_MANAGEMENT is often off on HPM while L1 D-cache stays on
 * (start.S). Use SDK L1C helpers so DMA buffers stay coherent.
 */
static void spi_hpm_dma_cache_flush(void *addr, size_t len)
{
	uint32_t start;
	uint32_t end;

	if (addr == NULL || len == 0U || !l1c_dc_is_enabled()) {
		return;
	}
	start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)addr);
	end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)addr + len);
	l1c_dc_writeback(start, end - start);
}

static void spi_hpm_dma_cache_invd(void *addr, size_t len)
{
	uint32_t start;
	uint32_t end;

	if (addr == NULL || len == 0U || !l1c_dc_is_enabled()) {
		return;
	}
	start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)addr);
	end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)addr + len);
	l1c_dc_invalidate(start, end - start);
}
#endif

static inline bool spi_hpm_is_slave(const struct spi_config *spi_cfg)
{
	return IS_ENABLED(CONFIG_SPI_SLAVE) &&
	       (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_SLAVE);
}

static inline bool spi_hpm_slave_framed(void)
{
	return IS_ENABLED(CONFIG_SPI_HPM_SLAVE_FRAMED);
}

/* Align TX/RX counts for a slave chunk (framed or data-only). */
static void spi_hpm_slave_pad_sizes(uint32_t *tx_size, uint32_t *rx_size,
				   uint8_t **tx_data, uint8_t **rx_data)
{
	if (*tx_size == 0U && *rx_size == 0U) {
		return;
	}
	if (*tx_size == 0U) {
		*tx_size = *rx_size;
		*tx_data = NULL;
	} else if (*rx_size == 0U) {
		*rx_size = *tx_size;
		*rx_data = NULL;
	} else if (*tx_size > *rx_size) {
		*tx_size = *rx_size;
	} else if (*rx_size > *tx_size) {
		*rx_size = *tx_size;
	}
}

/* Build TRANSCTRL for this chunk.
 * Raw slave: SLVDATAONLY + write-read-together (Zephyr default).
 * Framed slave: cmd/addr phases + read-dummy-write (HPM SDK master samples).
 */
static void spi_hpm_prepare_control(spi_control_config_t *control_config,
				    bool slave,
				    uint32_t *tx_size, uint32_t *rx_size,
				    uint8_t **tx_data, uint8_t **rx_data,
				    struct spi_context *ctx)
{
	/* SDK samples always zero the struct first. slave_get_default_* does
	 * not clear master_config.cmd/addr_enable — garbage bits break framing.
	 */
	memset(control_config, 0, sizeof(*control_config));

	*tx_size = MIN(ctx->tx_len, SPI_SOC_TRANSFER_COUNT_MAX);
	*rx_size = MIN(ctx->rx_len, SPI_SOC_TRANSFER_COUNT_MAX);
	*tx_data = (uint8_t *)ctx->tx_buf;
	*rx_data = (uint8_t *)ctx->rx_buf;

	if (slave) {
		spi_slave_get_default_control_config(control_config);
		spi_hpm_slave_pad_sizes(tx_size, rx_size, tx_data, rx_data);

		if (spi_hpm_slave_framed()) {
			control_config->slave_config.slave_data_only = false;
			control_config->common_config.dummy_cnt = spi_dummy_count_1;
			control_config->common_config.trans_mode = spi_trans_read_dummy_write;
			control_config->common_config.data_phase_fmt = spi_single_io_mode;
		} else {
			control_config->slave_config.slave_data_only = true;
			control_config->common_config.trans_mode = spi_trans_write_read_together;
		}
		return;
	}

	spi_master_get_default_control_config(control_config);

	if (*tx_size == 0U) {
		*tx_data = NULL;
		control_config->common_config.trans_mode = spi_trans_read_only;
	} else if (*rx_size == 0U) {
		*rx_data = NULL;
		control_config->common_config.trans_mode = spi_trans_write_only;
	} else if (*tx_size == *rx_size) {
		control_config->common_config.trans_mode = spi_trans_write_read_together;
	} else if (*tx_size > *rx_size) {
		*tx_size = *rx_size;
		control_config->common_config.trans_mode = spi_trans_write_read_together;
	} else {
		*rx_size = *tx_size;
		control_config->common_config.trans_mode = spi_trans_write_read_together;
	}
}

static void spi_hpm_transfer_next_packet(const struct device *dev)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	struct spi_context *ctx = &data->ctx;
	spi_control_config_t control_config;
	hpm_stat_t stat;
	bool slave = spi_hpm_is_slave(ctx->config);
	spi_mode_selection_t mode = slave ? spi_slave_mode : spi_master_mode;
	ARG_UNUSED(mode);

	uint8_t *tx_data;
	uint8_t *rx_data;
	uint32_t tx_size;
	uint32_t rx_size;

	if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
		if (!slave) {
			spi_context_cs_control(&data->ctx, false);
		}
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	spi_hpm_prepare_control(&control_config, slave, &tx_size, &rx_size,
				&tx_data, &rx_data, ctx);

	data->control_config = control_config;
	data->transfer_len = MAX(rx_size, tx_size);

#ifdef CONFIG_SPI_HPM_INTERRUPT
	{
		uint8_t data_len_in_bytes = spi_get_data_length_in_bytes(base);
		uint32_t irq_mask;

		if (data_len_in_bytes == 0U) {
			data_len_in_bytes = 1U;
		}

		data->tx_buf = tx_data;
		data->rx_buf = rx_data;
		/* NULL tx_buf still needs clocked NOP frames when tx_size > 0. */
		data->tx_count = (tx_size > 0U) ? (tx_size / data_len_in_bytes) : 0U;
		/* Drain RX even when discarding (slave pad / NULL rx). */
		data->rx_count = (rx_size > 0U) ? (rx_size / data_len_in_bytes) : 0U;

		stat = spi_control_init(base, &control_config, tx_size, rx_size);
		if (stat != status_success) {
			LOG_ERR("SPI control init failed");
			return;
		}

		spi_set_tx_fifo_threshold(base, SPI_SOC_FIFO_DEPTH - 1U);
		spi_set_rx_fifo_threshold(base, 1U);

		if (slave && spi_hpm_slave_framed()) {
			/* SDK interrupt slave: wait for cmd phase, then enable FIFOs. */
			irq_mask = spi_slave_cmd_int;
			spi_enable_interrupt(base, irq_mask);
		} else {
			irq_mask = spi_end_int;
			if (data->tx_count > 0U) {
				irq_mask |= spi_tx_fifo_threshold_int;
			}
			if (data->rx_count > 0U) {
				irq_mask |= spi_rx_fifo_threshold_int;
			}
			spi_enable_interrupt(base, irq_mask);

			/* Master: writing CMD starts the frame. Slave data-only: no-op. */
			stat = spi_write_command(base, mode, &control_config, NULL);
			if (stat != status_success) {
				spi_disable_interrupt(base, irq_mask);
				LOG_ERR("SPI write command failed");
				return;
			}
		}
	}
#else
	{
		static uint8_t nop_buf[SPI_SOC_FIFO_DEPTH];
		static uint8_t discard_buf[SPI_SOC_FIFO_DEPTH];
		uint8_t cmd = 0;

		if ((tx_size > 0U) && (tx_data == NULL)) {
			memset(nop_buf, 0, sizeof(nop_buf));
			if (tx_size > sizeof(nop_buf)) {
				tx_size = sizeof(nop_buf);
				if (rx_size > tx_size) {
					rx_size = tx_size;
				}
				data->transfer_len = tx_size;
			}
			tx_data = nop_buf;
		}
		if ((rx_size > 0U) && (rx_data == NULL)) {
			if (rx_size > sizeof(discard_buf)) {
				rx_size = sizeof(discard_buf);
				if (tx_size > rx_size) {
					tx_size = rx_size;
				}
				data->transfer_len = rx_size;
			}
			rx_data = discard_buf;
		}

		do {
			stat = spi_transfer(base, &control_config, &cmd, NULL,
					    tx_data, tx_size, rx_data, rx_size);
			/* Slave waits for master CS/SCLK; SDK returns timeout until then. */
		} while (slave && (stat == status_timeout));

		if (stat != status_success) {
			LOG_ERR("Transfer failed");
			return;
		}

		if (slave && spi_hpm_slave_framed()) {
			data->slave_last_cmd = cmd;
			LOG_INF("slave cmd=0x%02x", cmd);
		}

		spi_hpm_master_transfer_callback(base, data);
	}
#endif

}

__attribute__((section(".isr")))static void spi_hpm_isr(const struct device *dev)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	volatile uint32_t irq_status;
#ifdef CONFIG_SPI_HPM_INTERRUPT
	uint8_t data_len_in_bytes;
	hpm_stat_t stat;
#endif

	irq_status = spi_get_interrupt_status(base);

#ifdef CONFIG_SPI_HPM_INTERRUPT
	data_len_in_bytes = spi_get_data_length_in_bytes(base);
	if (data_len_in_bytes == 0U) {
		data_len_in_bytes = 1U;
	}

	/* Framed slave: after cmd phase, enable data FIFOs (SDK interrupt slave). */
	if ((irq_status & spi_slave_cmd_int) && spi_hpm_slave_framed() &&
	    spi_hpm_is_slave(data->ctx.config)) {
		uint32_t fifo_mask = spi_end_int;
		uint8_t cmd = 0;

		spi_disable_interrupt(base, spi_slave_cmd_int);
		spi_clear_interrupt_status(base, spi_slave_cmd_int);

		(void)spi_read_command(base, spi_slave_mode, &data->control_config, &cmd);
		data->slave_last_cmd = cmd;
		LOG_INF("slave cmd=0x%02x", cmd);

		if (data->tx_count > 0U) {
			fifo_mask |= spi_tx_fifo_threshold_int;
		}
		if (data->rx_count > 0U) {
			fifo_mask |= spi_rx_fifo_threshold_int;
		}
		spi_enable_interrupt(base, fifo_mask);
	}

	/* Handle FIFO before end so the last unit is moved (SDK-aligned). */
	if ((irq_status & spi_rx_fifo_threshold_int) && (data->rx_count > 0U)) {
		if (data->rx_buf != NULL) {
			stat = spi_read_data(base, data_len_in_bytes, data->rx_buf, 1);
			if (stat != status_success) {
				LOG_ERR("spi read data failed");
			} else {
				data->rx_buf += data_len_in_bytes;
				data->rx_count--;
			}
		} else {
			uint8_t discard[4];

			stat = spi_read_data(base, data_len_in_bytes, discard, 1);
			if (stat != status_success) {
				LOG_ERR("spi discard rx failed");
			} else {
				data->rx_count--;
			}
		}
		spi_clear_interrupt_status(base, spi_rx_fifo_threshold_int);
		if (data->rx_count == 0U) {
			spi_disable_interrupt(base, spi_rx_fifo_threshold_int);
		}
	}

	if ((irq_status & spi_tx_fifo_threshold_int) && (data->tx_count > 0U)) {
		if (data->tx_buf != NULL) {
			stat = spi_write_data(base, data_len_in_bytes, data->tx_buf, 1);
			if (stat != status_success) {
				LOG_ERR("spi write data failed");
			} else {
				data->tx_buf += data_len_in_bytes;
				data->tx_count--;
			}
		} else {
			/* Zephyr NULL TX buffer: clock NOP (0) frames on MOSI. */
			uint8_t nop = 0;

			stat = spi_write_data(base, data_len_in_bytes, &nop, 1);
			if (stat != status_success) {
				LOG_ERR("spi write nop failed");
			} else {
				data->tx_count--;
			}
		}
		spi_clear_interrupt_status(base, spi_tx_fifo_threshold_int);
		if (data->tx_count == 0U) {
			spi_disable_interrupt(base, spi_tx_fifo_threshold_int);
		}
	}

	if (irq_status & spi_end_int) {
		spi_disable_interrupt(base, spi_end_int | spi_rx_fifo_threshold_int |
					     spi_tx_fifo_threshold_int);
		spi_clear_interrupt_status(base, spi_end_int);
#ifdef CONFIG_SPI_HPM_SPI_DMA
		if (data->dma_use_spi_end) {
			data->dma_use_spi_end = false;
			spi_disable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable);
			data->status_flags |= data->dma_expected_flags;
#ifdef CONFIG_SPI_ASYNC
			if (data->ctx.asynchronous) {
				if (data->ctx.rx_buf != NULL && data->dma_chunk_rx > 0U) {
					spi_hpm_dma_cache_invd(data->ctx.rx_buf,
							       data->dma_chunk_rx);
				}
				spi_hpm_dma_async_finalize(data, 0);
				return;
			}
#endif
			spi_context_complete(&data->ctx, data->dev, 0);
			return;
		}
#endif
		spi_hpm_master_transfer_callback(base, data);
	}
#else
	if (irq_status & spi_end_int) {
		spi_disable_interrupt(base, spi_end_int | spi_rx_fifo_threshold_int |
					     spi_tx_fifo_threshold_int);
		spi_clear_interrupt_status(base, spi_end_int);
#ifdef CONFIG_SPI_HPM_SPI_DMA
		if (data->dma_use_spi_end) {
			data->dma_use_spi_end = false;
			spi_disable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable);
			data->status_flags |= data->dma_expected_flags;
#ifdef CONFIG_SPI_ASYNC
			if (data->ctx.asynchronous) {
				if (data->ctx.rx_buf != NULL && data->dma_chunk_rx > 0U) {
					spi_hpm_dma_cache_invd(data->ctx.rx_buf,
							       data->dma_chunk_rx);
				}
				spi_hpm_dma_async_finalize(data, 0);
				return;
			}
#endif
			spi_context_complete(&data->ctx, data->dev, 0);
			return;
		}
#endif
		spi_hpm_master_transfer_callback(base, data);
	}
#endif
}

static void spi_hpm_master_transfer_callback(SPI_Type *base, void *userData)
{
	struct spi_hpm_data *data = userData;
	size_t tx_adv = MIN(data->transfer_len, data->ctx.tx_len);
	size_t rx_adv = MIN(data->transfer_len, data->ctx.rx_len);

	ARG_UNUSED(base);

	spi_context_update_tx(&data->ctx, 1, tx_adv);
	spi_context_update_rx(&data->ctx, 1, rx_adv);

	spi_hpm_transfer_next_packet(data->dev);
}

static int spi_hpm_configure(const struct device *dev,
			      const struct spi_config *spi_cfg)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	spi_timing_config_t timing_config = {0};
	spi_format_config_t format_config = {0};
	uint32_t word_size;
	bool slave;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		/* This configuration is already in use */
		return 0;
	}

	word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	if (word_size > MAX_DATA_WIDTH) {
		LOG_ERR("Word size %d is greater than %d",
			    word_size, MAX_DATA_WIDTH);
		return -EINVAL;
	}

	slave = spi_hpm_is_slave(spi_cfg);

	if (slave) {
		spi_slave_get_default_format_config(&format_config);
		format_config.common_config.mode = spi_slave_mode;
		/* ADDRLEN is still programmed from master_config; match 1-byte addr. */
		format_config.master_config.addr_len_in_bytes = 1;
	} else {
		/* Master drives SCLK — program divider from spi_cfg->frequency. */
		timing_config.master_config.clk_src_freq_in_hz =
			clock_get_frequency(config->clock_name);
		timing_config.master_config.sclk_freq_in_hz = spi_cfg->frequency;
		timing_config.master_config.cs2sclk = config->cs2sclk;
		timing_config.master_config.csht = config->csht;
		spi_master_timing_init(base, &timing_config);

		spi_master_get_default_format_config(&format_config);
		format_config.common_config.mode = spi_master_mode;
	}

	format_config.common_config.data_len_in_bits = word_size;
	format_config.common_config.data_merge = false;
	format_config.common_config.mosi_bidir = false;

	format_config.common_config.cpol =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL)
		? spi_sclk_high_idle
		: spi_sclk_low_idle;

	format_config.common_config.cpha =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA)
		? spi_sclk_sampling_even_clk_edges
		: spi_sclk_sampling_odd_clk_edges;

	format_config.common_config.lsb =
		(spi_cfg->operation & SPI_TRANSFER_LSB) ? true : false;

	spi_format_init(base, &format_config);

	data->ctx.config = spi_cfg;

	return 0;
}

#ifdef CONFIG_SPI_HPM_SPI_DMA

#ifdef CONFIG_SPI_ASYNC
static int spi_hpm_dma_start_chunk(const struct device *dev);

static void spi_hpm_dma_async_finalize(struct spi_hpm_data *data, int status)
{
	const struct spi_hpm_config *config = data->dev->config;
	SPI_Type *base = config->base;
	volatile uint32_t spi_status;
	int ret;

	do {
		spi_status = base->STATUS;
	} while (spi_status & SPI_STATUS_SPIACTIVE_MASK);

	spi_disable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable);

	if (status == 0) {
		spi_context_update_tx(&data->ctx, 1,
				      MIN(data->dma_chunk_tx, data->ctx.tx_len));
		spi_context_update_rx(&data->ctx, 1,
				      MIN(data->dma_chunk_rx, data->ctx.rx_len));
	}

	if (status == 0 && (data->ctx.rx_len > 0 || data->ctx.tx_len > 0)) {
		ret = spi_hpm_dma_start_chunk(data->dev);
		if (ret == 0) {
			return; /* next chunk in flight */
		}
		status = ret;
	}

	if (!spi_hpm_is_slave(data->ctx.config)) {
		spi_context_cs_control(&data->ctx, false);
	}
	spi_context_complete(&data->ctx, data->dev, status);
}
#endif /* CONFIG_SPI_ASYNC */

/* This function is executed in the interrupt context */
static void spi_hpm_dma_callback(const struct device *dev, void *arg, uint32_t channel, int status)
{
	/* arg directly holds the spi device */
	struct spi_hpm_data *data = arg;

	if (status != 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
		data->status_flags |= SPI_HPM_SPI_DMA_ERROR_FLAG;
	} else if (channel == data->dma_tx.channel) {
		data->status_flags |= SPI_HPM_SPI_DMA_TX_DONE_FLAG;
		LOG_DBG("DMA TX Block Complete");
	} else if (channel == data->dma_rx.channel) {
		data->status_flags |= SPI_HPM_SPI_DMA_RX_DONE_FLAG;
		LOG_DBG("DMA RX Block Complete");
	} else {
		LOG_ERR("DMA callback channel %d is not valid.", channel);
		data->status_flags |= SPI_HPM_SPI_DMA_ERROR_FLAG;
	}

	/* Only wake waiter when required direction(s) finished (or error). */
	if ((data->status_flags & SPI_HPM_SPI_DMA_ERROR_FLAG) ||
	    ((data->status_flags & data->dma_expected_flags) == data->dma_expected_flags)) {
		/* Slave framed: SDK waits on SPI end IRQ, not DMA TC alone. */
		if (data->dma_use_spi_end &&
		    !(data->status_flags & SPI_HPM_SPI_DMA_ERROR_FLAG)) {
			return;
		}
#ifdef CONFIG_SPI_ASYNC
		if (data->ctx.asynchronous) {
			spi_hpm_dma_async_finalize(
				data,
				(data->status_flags & SPI_HPM_SPI_DMA_ERROR_FLAG) ? -EIO : 0);
			return;
		}
#endif
		spi_context_complete(&data->ctx, data->dev, 0);
	}
}

static int spi_hpm_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct spi_hpm_config *cfg = dev->config;
	struct spi_hpm_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	SPI_Type *base = cfg->base;


	/* remember active TX DMA channel (used in callback) */
	struct stream *stream = &data->dma_tx;

	blk_cfg = &stream->dma_blk_cfg;

	/* prepare the block for this TX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	if (buf == NULL) {
		/* NOP TX: repeatedly fetch from dummy (keep addr fixed). */
		data->dummy_tx_buffer = 0U;
		blk_cfg->source_address = (uint32_t)&data->dummy_tx_buffer;
		stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		/* tx direction has memory as source and peripheral as dest. */
		blk_cfg->source_address = (uint32_t)buf;
		stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* Dest is SPI DATA reg */
	blk_cfg->dest_address = (uint32_t)&(base->DATA);
	blk_cfg->block_size = len;
	/* Transfer 1 byte each DMA loop */
	stream->dma_cfg.source_burst_length = 1;

	stream->dma_cfg.head_block = &stream->dma_blk_cfg;
	/* give the client dev as arg, as the callback comes from the dma */
	stream->dma_cfg.user_data = data;
	/* pass our client origin to the dma: data->dma_tx.dma_channel */
	return dma_config(data->dma_tx.dma_dev, data->dma_tx.channel, &stream->dma_cfg);
}

static int spi_hpm_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct spi_hpm_config *cfg = dev->config;
	struct spi_hpm_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	SPI_Type *base = cfg->base;


	/* retrieve active RX DMA channel (used in callback) */
	struct stream *stream = &data->dma_rx;

	blk_cfg = &stream->dma_blk_cfg;

	/* prepare the block for this RX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	if (buf == NULL) {
		/* Discard RX into dummy (fixed dest address). */
		blk_cfg->dest_address = (uint32_t)&data->dummy_rx_buffer;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		/* rx direction has peripheral as source and mem as dest. */
		blk_cfg->dest_address = (uint32_t)buf;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	blk_cfg->block_size = len;
	/* Source is SPI DATA reg */
	blk_cfg->source_address = (uint32_t)&(base->DATA);
	stream->dma_cfg.source_burst_length = 1;

	stream->dma_cfg.head_block = blk_cfg;
	stream->dma_cfg.user_data = data;

	/* pass our client origin to the dma: data->dma_rx.channel */
	return dma_config(data->dma_rx.dma_dev, data->dma_rx.channel, &stream->dma_cfg);
}

static int wait_dma_rx_tx_done(const struct device *dev)
{
	struct spi_hpm_data *data = dev->data;
	int ret = -1;

	while (1) {
		ret = spi_context_wait_for_completion(&data->ctx);
		if (ret) {
			LOG_DBG("Timed out waiting for SPI context to complete");
			return ret;
		}
		if (data->status_flags & SPI_HPM_SPI_DMA_ERROR_FLAG) {
			return -EIO;
		}

		if ((data->status_flags & data->dma_expected_flags) ==
		    data->dma_expected_flags) {
			LOG_DBG("DMA block completed");
			return 0;
		}
	}
}

static int spi_hpm_dma_start_chunk(const struct device *dev)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	spi_control_config_t control_config;
	uint8_t *tx_data;
	uint8_t *rx_data;
	uint32_t tx_dma_size;
	uint32_t rx_dma_size;
	bool slave = spi_hpm_is_slave(data->ctx.config);
	int ret;

	data->status_flags = 0U;
	data->dma_expected_flags = 0U;
	data->dma_use_spi_end = false;

	spi_hpm_prepare_control(&control_config, slave, &tx_dma_size, &rx_dma_size,
				&tx_data, &rx_data, &data->ctx);

	if (tx_dma_size == 0U && rx_dma_size == 0U) {
		return 0;
	}

	/* DMA sees physical memory; flush TX / invalidate RX before start. */
	if (tx_data != NULL) {
		spi_hpm_dma_cache_flush(tx_data, tx_dma_size);
	}
	if (rx_data != NULL) {
		spi_hpm_dma_cache_invd(rx_data, rx_dma_size);
	}

	/* Match SDK DMA slave: hold off SPI TX DMA req until setup completes. */
	spi_disable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable);

	if (slave || (tx_dma_size > 0U && rx_dma_size > 0U)) {
		/* Slave SLVDATAONLY always needs both directions. */
		ret = spi_hpm_dma_rx_load(dev, rx_data, rx_dma_size);
		if (ret != 0) {
			return ret;
		}
		ret = spi_hpm_dma_tx_load(dev, tx_data, tx_dma_size);
		if (ret != 0) {
			return ret;
		}
		control_config.common_config.tx_dma_enable = true;
		control_config.common_config.rx_dma_enable = true;
		data->dma_expected_flags = SPI_HPM_SPI_DMA_DONE_FLAG;
		ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
		if (ret != 0) {
			return ret;
		}
		ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
		if (ret != 0) {
			return ret;
		}
	} else if (tx_dma_size == 0U) {
		ret = spi_hpm_dma_rx_load(dev, rx_data, rx_dma_size);
		if (ret != 0) {
			return ret;
		}
		control_config.common_config.rx_dma_enable = true;
		data->dma_expected_flags = SPI_HPM_SPI_DMA_RX_DONE_FLAG;
		ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
		if (ret != 0) {
			return ret;
		}
	} else {
		ret = spi_hpm_dma_tx_load(dev, tx_data, tx_dma_size);
		if (ret != 0) {
			return ret;
		}
		control_config.common_config.tx_dma_enable = true;
		data->dma_expected_flags = SPI_HPM_SPI_DMA_TX_DONE_FLAG;
		ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
		if (ret != 0) {
			return ret;
		}
	}

	if (slave && spi_hpm_slave_framed()) {
		data->dma_slave_cmd = 0;
		data->dma_slave_addr = 0;
		if (status_success != spi_setup_dma_transfer(base, &control_config,
							     &data->dma_slave_cmd,
							     &data->dma_slave_addr,
							     tx_dma_size, rx_dma_size)) {
			LOG_ERR("SPI setup DMA Transfer failed");
			return -EIO;
		}
		/* SDK DMA slave completes on spi_end_int. */
		data->dma_use_spi_end = true;
		spi_enable_interrupt(base, spi_end_int);
	} else if (status_success != spi_setup_dma_transfer(base, &control_config,
							    NULL, NULL, tx_dma_size,
							    rx_dma_size)) {
		LOG_ERR("SPI setup DMA Transfer failed");
		return -EIO;
	}

	data->dma_chunk_tx = tx_dma_size;
	data->dma_chunk_rx = rx_dma_size;
	return 0;
}

static int transceive_dma(const struct device *dev,
		      const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
			  spi_callback_t cb,
			  void *userdata)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	int ret;
	volatile uint32_t status;

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);

	ret = spi_hpm_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (!spi_hpm_is_slave(spi_cfg)) {
		spi_context_cs_control(&data->ctx, true);
	}

	while (data->ctx.rx_len > 0 || data->ctx.tx_len > 0) {
		ret = spi_hpm_dma_start_chunk(dev);
		if (ret != 0) {
			goto out;
		}

#ifdef CONFIG_SPI_ASYNC
		if (asynchronous) {
			/* Further chunks / completion in DMA callback. */
			return 0;
		}
#endif

		ret = wait_dma_rx_tx_done(dev);
		if (ret != 0) {
			data->dma_use_spi_end = false;
			spi_disable_interrupt(base, spi_end_int);
			goto out;
		}

		do {
			status = base->STATUS;
		} while (status & SPI_STATUS_SPIACTIVE_MASK);

		spi_disable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable);

		if (data->ctx.rx_buf != NULL && data->dma_chunk_rx > 0U) {
			spi_hpm_dma_cache_invd(data->ctx.rx_buf, data->dma_chunk_rx);
		}

		spi_context_update_tx(&data->ctx, 1,
				      MIN(data->dma_chunk_tx, data->ctx.tx_len));
		spi_context_update_rx(&data->ctx, 1,
				      MIN(data->dma_chunk_rx, data->ctx.rx_len));
	}

	if (!spi_hpm_is_slave(spi_cfg)) {
		spi_context_cs_control(&data->ctx, false);
	}

out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

#endif

static int transceive(const struct device *dev,
		      const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
			  spi_callback_t cb,
			  void *userdata)
{
	struct spi_hpm_data *data = dev->data;
	int ret;

#if defined(CONFIG_SPI_ASYNC) && !defined(CONFIG_SPI_HPM_INTERRUPT)
	if (asynchronous) {
		/* Polling path blocks in spi_transfer(); not true async. */
		return -ENOTSUP;
	}
#endif

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);

	ret = spi_hpm_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (!spi_hpm_is_slave(spi_cfg)) {
		spi_context_cs_control(&data->ctx, true);
	}

	spi_hpm_transfer_next_packet(dev);

	ret = spi_context_wait_for_completion(&data->ctx);
out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_hpm_transceive(const struct device *dev,
			       const struct spi_config *spi_cfg,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_HPM_SPI_DMA
	return transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
#endif /* CONFIG_SPI_HPM_SPI_DMA */
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_hpm_transceive_async(const struct device *dev,
				     const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
					 spi_callback_t cb,
					 void *userdata)
{
#ifdef CONFIG_SPI_HPM_SPI_DMA
	return transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
#endif /* CONFIG_SPI_HPM_SPI_DMA */
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_hpm_release(const struct device *dev,
			    const struct spi_config *spi_cfg)
{
	struct spi_hpm_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_hpm_init(const struct device *dev)
{
	int err;
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;

	config->irq_config_func(dev);

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	data->dev = dev;

#ifdef CONFIG_SPI_HPM_SPI_DMA
	if (!device_is_ready(data->dma_tx.dma_dev)) {
		LOG_ERR("%s device is not ready", data->dma_tx.dma_dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(data->dma_rx.dma_dev)) {
		LOG_ERR("%s device is not ready", data->dma_rx.dma_dev->name);
		return -ENODEV;
	}
#endif /* CONFIG_SPI_HPM_SPI_DMA */

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	clock_set_source_divider(config->clock_name, config->clock_src, config->clock_div);
	clock_add_to_group(config->clock_name, 0);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_hpm_driver_api = {
	.transceive = spi_hpm_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_hpm_transceive_async,
#endif
	.release = spi_hpm_release,
};

#ifdef CONFIG_SPI_HPM_SPI_DMA
#define SPI_DMA_CHANNELS(n)		\
	.dma_tx = {						\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)), \
		.channel =					\
			DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),	\
		.dma_cfg = {					\
			.channel_direction = MEMORY_TO_PERIPHERAL,	\
			.dma_callback = spi_hpm_dma_callback,		\
			.source_data_size = 1,				\
			.dest_data_size = 1,				\
			.block_count = 1,		\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, tx, source) \
		}							\
	},								\
	.dma_rx = {						\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)), \
		.channel =					\
			DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),	\
		.dma_cfg = {				\
			.channel_direction = PERIPHERAL_TO_MEMORY,	\
			.dma_callback = spi_hpm_dma_callback,		\
			.source_data_size = 1,				\
			.dest_data_size = 1,				\
			.block_count = 1,		\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, rx, source) \
		}							\
	}
#else
#define SPI_DMA_CHANNELS(n)
#endif /* CONFIG_SPI_HPM_SPI_DMA */


#define SPI_HPM_DEVICE_INIT(n)						\
	PINCTRL_DT_INST_DEFINE(n);				\
									\
	static void spi_hpm_config_func_##n(const struct device *dev);	\
									\
	static const struct spi_hpm_config spi_hpm_config_##n = {	\
		.base = (SPI_Type *) DT_INST_REG_ADDR(n),		\
		.clock_name = DT_INST_CLOCKS_CELL(n, name),	\
		.clock_src = DT_INST_CLOCKS_CELL(n, src),	\
		.clock_div = DT_INST_CLOCKS_CELL(n, div),	\
		.irq_config_func = spi_hpm_config_func_##n,		\
		.cs2sclk = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, cs_sck_delay),	\
			DT_INST_PROP(n, cs_sck_delay)),		\
		.csht = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, cs_minimum_high_time),	\
			DT_INST_PROP(n, cs_minimum_high_time)),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)				\
	};								\
									\
	static struct spi_hpm_data spi_hpm_data_##n = {		\
		SPI_CONTEXT_INIT_LOCK(spi_hpm_data_##n, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_hpm_data_##n, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)	\
		SPI_DMA_CHANNELS(n)			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, spi_hpm_init, NULL,			\
			    &spi_hpm_data_##n,				\
			    &spi_hpm_config_##n, POST_KERNEL,		\
			    CONFIG_SPI_INIT_PRIORITY,			\
			    &spi_hpm_driver_api);			\
									\
	static void spi_hpm_config_func_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	\
			    spi_hpm_isr, DEVICE_DT_INST_GET(n), 0);	\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(SPI_HPM_DEVICE_INIT)
