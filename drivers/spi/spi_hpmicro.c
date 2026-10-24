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

/*
 * Two implementations share this file since the 2026-10-12 vendor HPM5100
 * drop (sdk_glue vendor/hpm5100-2026-10-12):
 *
 * - CONFIG_SOC_SERIES_HPM5100: the vendor driver, verbatim -- slave mode
 *   (SPI_SLAVE / SPI_HPM_SLAVE_FRAMED), CONFIG_SPI_HPM_INTERRUPT, reworked
 *   DMA and async paths.
 * - every other SoC: the pre-merge driver, verbatim. HS2 (HPM6E80) images
 *   must not change: its interrupt-driven FIFO pump is what the BMI423
 *   reads ride on, and spi_hpm_master_timing_init() is what the P4/P5 LED
 *   timing depends on (hs2_hpm_firmware tests/static/test_led_spi_timing.py
 *   executes that function out of this file).
 *
 * The config struct, DMA stream type, init/release and the instance macros
 * below are identical in both and shared.
 */

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

/* Shared by both halves of this file (the HPM5100 vendor branch below and
 * the HS2 branch after #else). Zephyr's frequency is an upper bound. The SDK timing initializer only
 * accepts exact integer ratios, so asking for e.g. 3.125 MHz from 80 MHz
 * otherwise fails without programming TIMING. Select the nearest supported
 * rate at or below the request, including fractional-Hz hardware rates.
 * Exact ratios (e.g. the BMI423's 80 / 8 = 10 MHz) keep the same register. */
static int spi_hpm_master_timing_init(SPI_Type *base,
				    const spi_timing_config_t *timing)
{
	uint32_t source_hz = timing->master_config.clk_src_freq_in_hz;
	uint32_t requested_hz = timing->master_config.sclk_freq_in_hz;
	uint32_t divider = 1U;
	spi_timing_config_t hw_timing = *timing;

	if (source_hz == 0U || requested_hz == 0U) {
		return -EINVAL;
	}
	if (requested_hz < source_hz) {
		divider = (source_hz - 1U) / requested_hz + 1U;
		if (divider > 510U) {
			return -EINVAL;
		}
		divider += divider & 1U;
	}

	/* Use the SDK for CS timing, then its divider setter for the exact
	 * register ratio. 0xff is bypass; 0..0xfe mean 2..510. */
	hw_timing.master_config.sclk_freq_in_hz = source_hz;
	if (spi_master_timing_init(base, &hw_timing) != status_success) {
		return -EINVAL;
	}
	if (divider != 1U) {
		spi_master_set_sclk_div(base, (divider / 2U) - 1U);
	}
	return 0;
}

#if defined(CONFIG_SOC_SERIES_HPM5100)

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
		/* The vendor branch called spi_master_timing_init() directly and
		 * ignored its status: any request that is not an exact even
		 * divisor of the source (the WS2812's ~3.1 MHz of 80 MHz) left
		 * TIMING unprogrammed and the bus at whatever rate it had. */
		{
			int terr = spi_hpm_master_timing_init(base, &timing_config);

			if (terr != 0) {
				LOG_ERR("SCLK %u Hz not reachable from %u Hz",
					spi_cfg->frequency,
					timing_config.master_config.clk_src_freq_in_hz);
				return terr;
			}
		}

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

/* Undo spi_context_lock() after spi_hpm_configure() failed. Not
 * spi_context_release(): it decides through ctx->config->operation, and a
 * configure that fails on first use returns before ctx->config is set - the
 * NULL->operation load then reads ILM at 0x4, and a garbage SPI_LOCK_ON in it
 * keeps the bus locked for good. Same decision as release() on a negative
 * status, made on the request itself. */
static void spi_hpm_unlock_unconfigured(struct spi_context *ctx,
					const struct spi_config *spi_cfg)
{
#ifndef CONFIG_SPI_ASYNC
	if (spi_cfg->operation & SPI_LOCK_ON) {
		return;
	}
#else
	ARG_UNUSED(spi_cfg);
#endif
	ctx->owner = NULL;
	k_sem_give(&ctx->lock);
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
		spi_hpm_unlock_unconfigured(&data->ctx, spi_cfg);
		return ret;
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
		spi_hpm_unlock_unconfigured(&data->ctx, spi_cfg);
		return ret;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (!spi_hpm_is_slave(spi_cfg)) {
		spi_context_cs_control(&data->ctx, true);
	}

	spi_hpm_transfer_next_packet(dev);

	ret = spi_context_wait_for_completion(&data->ctx);
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

#else /* !CONFIG_SOC_SERIES_HPM5100 */

/*
 * Keep the interrupt-driven path. The vendor HPM5100 drop flipped this to 0 to
 * sidestep the broken base ISR, but 020893a/fbd01a0 fixed that ISR (bounded
 * FIFO pump, thresholds, IRQs armed before start, drain on end). 0 is not a
 * clean polling fallback either: four guards use #ifdef (always true) and one
 * uses #if, which leaves a half-compiled hybrid.
 */
#define CONFIG_SPI_INTERRUPT_DRIVEN 1

struct spi_hpm_data {
	const struct device *dev;
	spi_control_config_t control_config;
	struct spi_context ctx;
	size_t transfer_len;
#ifdef CONFIG_SPI_INTERRUPT_DRIVEN
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	/* FIFO units left to pump this packet. The threshold interrupts are
	 * level conditions (TX: entries <= TXTHRES, RX: entries >= RXTHRES);
	 * without these bounds the ISR keeps pushing past the buffer end
	 * until the TX FIFO jams full of overrun bytes and spi_write_data
	 * times out (one "transfer failed" per packet, wire data unharmed
	 * only because WRTRANCNT caps what actually shifts out). */
	size_t tx_remaining;
	size_t rx_remaining;
#endif
#ifdef CONFIG_SPI_HPM_SPI_DMA
	volatile uint32_t status_flags;
	struct stream dma_rx;
	struct stream dma_tx;
	/* dummy value used for transferring NOP when tx buf is null */
	uint32_t dummy_tx_buffer;
	/* dummy value used to read RX data into when rx buf is null */
	uint32_t dummy_rx_buffer;
#endif
};

static void spi_hpm_master_transfer_callback(SPI_Type *base, void *userData);

static void spi_hpm_transfer_next_packet(const struct device *dev)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	struct spi_context *ctx = &data->ctx;
	spi_control_config_t control_config;
	hpm_stat_t stat;

	uint8_t *tx_data;
	uint8_t *rx_data;
	uint32_t tx_size;
	uint32_t rx_size;

	/* get SPI control config for master */
	/* addr and cmd phase are disabled */
    spi_master_get_default_control_config(&control_config);

	if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
		/* nothing left to rx or tx, we're done! */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	tx_size = MIN(data->ctx.tx_len, SPI_SOC_TRANSFER_COUNT_MAX);
	rx_size = MIN(data->ctx.rx_len, SPI_SOC_TRANSFER_COUNT_MAX);

	if (tx_size == 0) {
		/* rx only, nothing to tx */
		tx_data = NULL;
		rx_data = (uint8_t *) ctx->rx_buf;
		control_config.common_config.trans_mode = spi_trans_read_only;
	} else if (rx_size == 0) {
		/* tx only, nothing to rx */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = NULL;
		control_config.common_config.trans_mode = spi_trans_write_only;
	} else if (tx_size == rx_size) {
		/* rx and tx are the same length */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = (uint8_t *) ctx->rx_buf;
		control_config.common_config.trans_mode = spi_trans_write_read_together;
	} else if (tx_size > rx_size) {
		/* Break up the tx into multiple transfers so we don't have to
		 * rx into a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = (uint8_t *) ctx->rx_buf;
		tx_size = rx_size;
		control_config.common_config.trans_mode = spi_trans_write_read_together;
	} else {
		/* Break up the rx into multiple transfers so we don't have to
		 * tx from a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = (uint8_t *) ctx->rx_buf;
		rx_size = tx_size;
		control_config.common_config.trans_mode = spi_trans_write_read_together;
	}

	data->control_config = control_config;
	data->transfer_len = MAX(rx_size, tx_size);

#if CONFIG_SPI_INTERRUPT_DRIVEN
	data->tx_buf = (uint8_t *) ctx->tx_buf;
	data->rx_buf = (uint8_t *) ctx->rx_buf;
	data->tx_remaining = tx_size;
	data->rx_remaining = rx_size;
    stat = spi_control_init(base, &control_config, tx_size, rx_size);
	if (stat != status_success) {
        LOG_ERR("SPI control init failed");
		return;
    }

	/* FIFO thresholds, per the SDK interrupt sample (samples/drivers/spi/
	 * interrupt/master): TX refills whenever the FIFO is not full (keeps
	 * SCLK continuous), RX drains per entry. The reset-default thresholds
	 * are 0, which makes the RX condition (entries >= RXTHRES) permanently
	 * true and storms the ISR on write-only transfers. */
	spi_set_tx_fifo_threshold(base, SPI_SOC_FIFO_DEPTH - 1U);
	spi_set_rx_fifo_threshold(base, 1U);

	/* enable end int + only the FIFO directions this packet uses */
	{
		uint32_t irq_mask = spi_end_int;

		if (rx_size > 0) {
			irq_mask |= spi_rx_fifo_threshold_int;
		}
		if (tx_size > 0) {
			irq_mask |= spi_tx_fifo_threshold_int;
		}
		spi_enable_interrupt(base, irq_mask);
	}

	/* Command phase LAST: spi_write_command() is what starts the
	 * transfer, and the SDK interrupt sample arms thresholds and
	 * interrupts before it (samples/drivers/spi/interrupt/master).
	 * Starting first meant the leading TX bytes were clocked out as
	 * FIFO-underflow zeros before the ISR pump was armed - the first
	 * SPI_HOLD-less read user (BMI423 bring-up) saw its command byte
	 * never reach the wire.
	 */
    stat = spi_write_command(base, spi_master_mode, &control_config, NULL);
    if (stat != status_success) {
        LOG_ERR("SPI write command failed");
		return;
    }
#else
	stat = spi_transfer(base, &control_config, NULL, NULL,
                (uint8_t *)tx_data, tx_size, (uint8_t *)rx_data, rx_size);

	if (stat != status_success) {
		LOG_ERR("Transfer failed");
		return;
	}

	spi_hpm_master_transfer_callback(base, data);
#endif

}

__attribute__((section(".isr")))static void spi_hpm_isr(const struct device *dev)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	volatile uint32_t irq_status;
#ifdef CONFIG_SPI_INTERRUPT_DRIVEN
	uint8_t data_len_in_bytes;
	hpm_stat_t stat;
#endif

	irq_status = spi_get_interrupt_status(base);

	if (irq_status & spi_end_int) {
#ifdef CONFIG_SPI_INTERRUPT_DRIVEN
		/* The end interrupt can coincide with the last rx-threshold
		 * condition (the SDK sample handles end/rx/tx as parallel ifs,
		 * not else-if). Drain what the pump has not consumed before
		 * declaring completion, or the rx tail is silently dropped
		 * (first seen as a one-byte-short read on BMI423 bring-up).
		 */
		data_len_in_bytes = spi_get_data_length_in_bytes(base);
		while (data->rx_remaining > 0 && data->rx_buf != NULL &&
		       spi_get_rx_fifo_valid_data_size(base) > 0) {
			stat = spi_read_data(base, data_len_in_bytes,
					     data->rx_buf, 1);
			if (stat != status_success) {
				LOG_ERR("spi rx drain failed");
				break;
			}
			data->rx_buf += data_len_in_bytes;
			data->rx_remaining--;
		}
#endif
		spi_disable_interrupt(base, spi_end_int | spi_rx_fifo_threshold_int | spi_tx_fifo_threshold_int);
		spi_hpm_master_transfer_callback(base, data);
		spi_clear_interrupt_status(base, spi_end_int);
#ifdef CONFIG_SPI_INTERRUPT_DRIVEN
	} else if (irq_status & (spi_rx_fifo_threshold_int | spi_tx_fifo_threshold_int)) {
		/* Per-direction pump with explicit bounds (the SDK interrupt
		 * sample's sent_count/receive_count pattern): the threshold
		 * interrupts are level conditions, so each direction must stop
		 * itself once its packet quota is done - the interrupt fires
		 * again for FIFO states the transfer no longer cares about. */
		data_len_in_bytes = spi_get_data_length_in_bytes(base);

		if (irq_status & spi_rx_fifo_threshold_int) {
			if (data->rx_remaining > 0 && data->rx_buf != NULL) {
				stat = spi_read_data(base, data_len_in_bytes,
						     data->rx_buf, 1);
				if (stat == status_success) {
					data->rx_buf += data_len_in_bytes;
					data->rx_remaining--;
					if (data->rx_remaining == 0) {
						spi_disable_interrupt(base,
							spi_rx_fifo_threshold_int);
					}
				} else {
					LOG_ERR("spi rx failed");
				}
			} else {
				spi_disable_interrupt(base,
						      spi_rx_fifo_threshold_int);
			}
			spi_clear_interrupt_status(base,
						   spi_rx_fifo_threshold_int);
		}

		if (irq_status & spi_tx_fifo_threshold_int) {
			if (data->tx_remaining > 0 && data->tx_buf != NULL) {
				stat = spi_write_data(base, data_len_in_bytes,
						      data->tx_buf, 1);
				if (stat == status_success) {
					data->tx_buf += data_len_in_bytes;
					data->tx_remaining--;
					if (data->tx_remaining == 0) {
						spi_disable_interrupt(base,
							spi_tx_fifo_threshold_int);
					}
				} else {
					LOG_ERR("spi tx failed");
				}
			} else {
				spi_disable_interrupt(base,
						      spi_tx_fifo_threshold_int);
			}
			spi_clear_interrupt_status(base,
						   spi_tx_fifo_threshold_int);
		}
#endif
	}
}

static void spi_hpm_master_transfer_callback(SPI_Type *base, void *userData)
{
	struct spi_hpm_data *data = userData;

	spi_context_update_tx(&data->ctx, 1, data->transfer_len);
	spi_context_update_rx(&data->ctx, 1, data->transfer_len);

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
	int ret;

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

	/* TODO support slave mode */
	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("slave not supported");
		return -ENOTSUP;
	}

	/* set SPI sclk frequency for master */
	timing_config.master_config.clk_src_freq_in_hz = clock_get_frequency(config->clock_name);
	timing_config.master_config.sclk_freq_in_hz = spi_cfg->frequency;
	timing_config.master_config.cs2sclk = config->cs2sclk;
	timing_config.master_config.csht    = config->csht;
	ret = spi_hpm_master_timing_init(base, &timing_config);
	if (ret != 0) {
		return ret;
	}

	spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = word_size;
    format_config.common_config.data_merge = false;
    format_config.common_config.mosi_bidir = false;
    format_config.common_config.mode = spi_master_mode;

	/* CPOL=0. Active-high LPSPI clock (idles low) */
	/* CPOL=1. Active-low LPSPI clock (idles high) */
	format_config.common_config.cpol =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL)
		? spi_sclk_high_idle
		: spi_sclk_low_idle;

	/* CPHA=0. Data is captured on the leading edge of the SCK and changed on the following edge. */
	/* CPHA=1. Data is changed on the leading edge of the SCK and captured on the following edge. */
	format_config.common_config.cpha =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA)
		? spi_sclk_sampling_even_clk_edges
		: spi_sclk_sampling_odd_clk_edges;

	format_config.common_config.lsb =
		(spi_cfg->operation & SPI_TRANSFER_LSB)
		? true
		: false;

	spi_format_init(base, &format_config);

	data->ctx.config = spi_cfg;

	return 0;
}

#ifdef CONFIG_SPI_HPM_SPI_DMA

/* This function is executed in the interrupt context */
static void spi_hpm_dma_callback(const struct device *dev, void *arg, uint32_t channel, int status)
{
	/* arg directly holds the spi device */
	struct spi_hpm_data *data = arg;

	if (status != 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
		data->status_flags |= SPI_HPM_SPI_DMA_ERROR_FLAG;
	} else {
		/* identify the origin of this callback */
		if (channel == data->dma_tx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= SPI_HPM_SPI_DMA_TX_DONE_FLAG;
			LOG_DBG("DMA TX Block Complete");
		} else if (channel == data->dma_rx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= SPI_HPM_SPI_DMA_RX_DONE_FLAG;
			LOG_DBG("DMA RX Block Complete");
		} else {
			LOG_ERR("DMA callback channel %d is not valid.",
								channel);
			data->status_flags |= SPI_HPM_SPI_DMA_ERROR_FLAG;
		}
	}
	spi_context_complete(&data->ctx, dev, 0);
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
		/* Treat the transfer as a peripheral to peripheral one, so that DMA
		 * reads from this address each time
		 */
		blk_cfg->source_address = (uint32_t)&data->dummy_tx_buffer;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		/* tx direction has memory as source and peripheral as dest. */
		blk_cfg->source_address = (uint32_t)buf;
		stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
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
		/* Treat the transfer as a peripheral to peripheral one, so that DMA
		 * reads from this address each time
		 */
		blk_cfg->dest_address = (uint32_t)&data->dummy_rx_buffer;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		/* rx direction has peripheral as source and mem as dest. */
		blk_cfg->dest_address = (uint32_t)buf;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_INCREMENT;
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

		if ((data->status_flags & SPI_HPM_SPI_DMA_DONE_FLAG)) {
			LOG_DBG("DMA block completed");
			return 0;
		}
	}
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
	size_t tx_dma_size, rx_dma_size;
	volatile uint32_t status;
	spi_control_config_t control_config;

	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.slave_config.slave_data_only = false;

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);

	ret = spi_hpm_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	/* Send each spi buf via DMA, updating context as DMA completes */
	while (data->ctx.rx_len > 0 || data->ctx.tx_len > 0) {
		/* Clear status flags */
		data->status_flags = 0U;

		tx_dma_size = MIN(data->ctx.tx_len, SPI_SOC_TRANSFER_COUNT_MAX);
		rx_dma_size = MIN(data->ctx.rx_len, SPI_SOC_TRANSFER_COUNT_MAX);

		if (tx_dma_size == 0) {
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_read_only;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
			
		} else if (rx_dma_size == 0) {
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_only;
			control_config.common_config.tx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
		} else if (tx_dma_size == rx_dma_size) {
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_read_together;
			control_config.common_config.tx_dma_enable = true;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
		} else if (tx_dma_size > rx_dma_size) {
			tx_dma_size = rx_dma_size;
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_read_together;
			control_config.common_config.tx_dma_enable = true;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
		} else {
			rx_dma_size = tx_dma_size;
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_read_together;
			control_config.common_config.tx_dma_enable = true;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
		}

		if (status_success != spi_setup_dma_transfer(base, &control_config,
                                NULL, NULL, tx_dma_size, rx_dma_size)) {
			LOG_ERR("SPI setup DMA Transfer failed");
			goto out;
		}

		/* Enable DMA Requests, already enabled in spi_setup_dma_transfer */
		/* spi_enable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable); */

		/* Wait for DMA to finish */
		ret = wait_dma_rx_tx_done(dev);
		if (ret != 0) {
			goto out;
		}

		/* wait until module is idle */
		do {
			status = base->STATUS;
		} while(status & SPI_STATUS_SPIACTIVE_MASK);


		/* Disable DMA */
		spi_disable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable);

		/* Update SPI contexts with amount of data we just sent */
		spi_context_update_tx(&data->ctx, 1, tx_dma_size);
		spi_context_update_rx(&data->ctx, 1, rx_dma_size);
	}

	spi_context_cs_control(&data->ctx, false);

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

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);

	ret = spi_hpm_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

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

#endif /* CONFIG_SOC_SERIES_HPM5100 */

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
