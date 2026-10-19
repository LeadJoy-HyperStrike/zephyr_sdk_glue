/*
 * Copyright (c) 2022 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_uart

#include <errno.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <soc.h>
#include <hpm_common.h>
#include <hpm_uart_drv.h>
#include <hpm_clock_drv.h>
#ifdef CONFIG_UART_ASYNC_API
#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
#define UART_HPM_HW_RX_IDLE 1
#else
#define UART_HPM_HW_RX_IDLE 0
#endif
#if !UART_HPM_HW_RX_IDLE
#include <hpm_trgmmux_src.h>
#include <hpm_trgm_drv.h>
#if defined(CONFIG_SOC_SERIES_HPM5100)
#include <hpm_gptmrv2_drv.h>
#else
#include <hpm_gptmr_drv.h>
#endif
#endif /* !UART_HPM_HW_RX_IDLE */
#endif
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_hpmicro);
#ifdef CONFIG_UART_ASYNC_API

typedef struct uart_info {
    UART_Type *ptr;
    uint32_t baudrate;
    uint32_t *buff_addr;
} uart_info_t;

typedef struct dma_info {
	struct device *dev;
    uint8_t ch;
    uint8_t *dst_addr;
    uint32_t buff_size;
	uint8_t *next_dst_addr;
	uint32_t next_buff_size;
	uint8_t	current_mode; /*0: need config dma, 1: needn't config dma*/
	uint8_t *current_buf;
	uint32_t current_offset;
	uint32_t current_size; /* current release buff size */
	uint32_t current_total_size;
} dma_info_t;

typedef struct dmamux_info {
    uint8_t ch;
    uint8_t src;
} dmamux_info_t;

#if !UART_HPM_HW_RX_IDLE
typedef struct trgm_info {
    TRGM_Type *ptr;
    uint32_t pin_index;
    uint8_t input_src;
    uint8_t output_gptmr_in;
    uint8_t output_gptmr_synci;
} trgm_info_t;

typedef struct gptmr_info {
    GPTMR_Type *ptr;
    clock_name_t clock_name;
    uint8_t cmp_ch;
    uint8_t cap_ch;
    uint8_t irq_index;
} gptmr_info_t;
#endif

struct stream {
	const struct device *dma_dev;
	uint32_t channel; /* stores the channel for dma */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};
#endif

struct uart_hpm_cfg {
	UART_Type *base;
	uint32_t clock_name;
	uint32_t clock_src;
	uint32_t clock_div;
	uint32_t parity;
	uint32_t baud_rate;
	uint8_t flow_ctrl;
	bool loopback_en;
	bool dma_enabled;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
	const struct pinctrl_dev_config *pincfg;
};

struct uart_hpm_data {
	struct uart_config uart_config;
	uint32_t pm_ier;
	bool pm_suspended;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	uart_callback_t idle_user_callback;
	void *idle_user_data;
    volatile bool uart_rx_idle;
    volatile uint32_t uart_receive_data_size;
    uart_info_t uart_info;
    dma_info_t dma_rxinfo;
	dma_info_t dma_txinfo;
#if !UART_HPM_HW_RX_IDLE
    trgm_info_t trgm_info;
    gptmr_info_t gptmr_info;
#endif
	uint32_t idle_time_out_us;
	uint32_t rx_idle_last_got;
	struct k_work_delayable rx_timeout_work;
	volatile uint32_t status_flags;
	struct stream dma_rx;
	struct stream dma_tx;
#endif
};

#if defined(CONFIG_UART_ASYNC_API) && UART_HPM_HW_RX_IDLE
static void uart_hpm_async_hw_idle_isr(const struct device *dev);
#else
static inline void uart_hpm_async_hw_idle_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif

#ifdef CONFIG_UART_ASYNC_API
#if !UART_HPM_HW_RX_IDLE
static void config_gptmr_to_detect_uart_rx_idle(gptmr_info_t *gptmr_info, trgm_info_t *trgm_info, uint32_t time_out)
{
    uint32_t gptmr_freq, bits;
    gptmr_channel_config_t config;

    bits = 12+6; /* use max value: 1 start bit, 8 data bits, 2 stop bits, 1 parity bit */
    clock_add_to_group(gptmr_info->clock_name, 0);
    gptmr_freq = clock_get_frequency(gptmr_info->clock_name);
    gptmr_channel_get_default_config(gptmr_info->ptr, &config);
    config.cmp[0] = (gptmr_freq / 1000000) * time_out; /* Time to transmit a byte */
    config.cmp[1] = 0xffffffff;
    config.synci_edge = gptmr_synci_edge_both;
    gptmr_channel_config(gptmr_info->ptr, gptmr_info->cmp_ch, &config, false);
    gptmr_channel_reset_count(gptmr_info->ptr, gptmr_info->cmp_ch);

    /* config uart rx -> TRGM INPUT IO -> GPTMR SYNCI */
    trgm_output_update_source(trgm_info->ptr, trgm_info->output_gptmr_synci, trgm_info->input_src);

    gptmr_enable_irq(gptmr_info->ptr, GPTMR_CH_CMP_IRQ_MASK(gptmr_info->cmp_ch, 0)); /* enable compare irq */
    /* not start gptmr cmp channel counter, start this channel counter when rx reception start */
}

static void config_gptmr_to_detect_uart_rx_start(gptmr_info_t *gptmr_info, trgm_info_t *trgm_info)
{
    gptmr_channel_config_t config;
    clock_add_to_group(gptmr_info->clock_name, 0);
    gptmr_channel_get_default_config(gptmr_info->ptr, &config);
    config.mode = gptmr_work_mode_capture_at_falling_edge;

    gptmr_channel_config(gptmr_info->ptr, gptmr_info->cap_ch, &config, false);
    gptmr_channel_reset_count(gptmr_info->ptr, gptmr_info->cap_ch);

    /* configure uart_rx -> TRGM INPUT IO -> GPTMR IN */
    trgm_output_update_source(trgm_info->ptr, trgm_info->output_gptmr_in, trgm_info->input_src);
    gptmr_enable_irq(gptmr_info->ptr, GPTMR_CH_CAP_IRQ_MASK(gptmr_info->cap_ch)); /* enable capture irq */
    gptmr_start_counter(gptmr_info->ptr, gptmr_info->cap_ch);
}
#endif /* !UART_HPM_HW_RX_IDLE */

static int uart_hpm_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct uart_hpm_cfg *cfg = dev->config;
	struct uart_hpm_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	UART_Type *base = cfg->base;

	assert(buf != NULL);

	/* remember active TX DMA channel (used in callback) */
	struct stream *stream = &data->dma_tx;

	blk_cfg = &stream->dma_blk_cfg;

	/* prepare the block for this TX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	/* tx direction has memory as source and peripheral as dest. */
	blk_cfg->source_address = (uint32_t)buf;
	stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;

	/* Dest is SPI DATA reg */
	blk_cfg->dest_address = (uint32_t)&(base->THR);
	blk_cfg->block_size = len;
	/* Transfer 1 byte each DMA loop */
	stream->dma_cfg.source_burst_length = 1;

	stream->dma_cfg.head_block = &stream->dma_blk_cfg;
	/* give the client dev as arg, as the callback comes from the dma */
	stream->dma_cfg.user_data = data;
	/* pass our client origin to the dma: data->dma_tx.dma_channel */
	return dma_config(data->dma_tx.dma_dev, data->dma_tx.channel, &stream->dma_cfg);
}

static int uart_hpm_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct uart_hpm_cfg *cfg = dev->config;
	struct uart_hpm_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	UART_Type *base = cfg->base;

	assert(buf != NULL);

	/* retrieve active RX DMA channel (used in callback) */
	struct stream *stream = &data->dma_rx;

	blk_cfg = &stream->dma_blk_cfg;

	/* prepare the block for this RX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	/* rx direction has peripheral as source and mem as dest. */
	blk_cfg->dest_address = (uint32_t)buf;
	stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_INCREMENT;

	blk_cfg->block_size = len;
	/* Source is SPI DATA reg */
	blk_cfg->source_address = (uint32_t)&(base->RBR);
	stream->dma_cfg.source_burst_length = 1;

	stream->dma_cfg.head_block = blk_cfg;
	stream->dma_cfg.user_data = data;

	/* pass our client origin to the dma: data->dma_rx.channel */
	return dma_config(data->dma_rx.dma_dev, data->dma_rx.channel, &stream->dma_cfg);
}

#endif

#ifdef CONFIG_UART_ASYNC_API
static void uart_hpm_rx_timeout_handler(struct k_work *work);
#endif

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_hpm_configure_get(const struct device *dev, struct uart_config *cfg)
{
	struct uart_hpm_data *data = dev->data;
	*cfg = data->uart_config;
	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_hpm_configure_init(const struct device *dev, const struct uart_config *cfg)
{
	const struct uart_hpm_cfg *const config = dev->config;
	// struct uart_hpm_data * data = dev->data;
	parity_setting_t parity;
	hpm_stat_t stat = status_success;
	uart_config_t uart_config = {0};
	UART_Type * base = (UART_Type *)config->base;

#ifdef CONFIG_UART_ASYNC_API
	// if (!device_is_ready(data->dma_tx.dma_dev)) {
	// 	LOG_ERR("%s device is not ready", data->dma_tx.dma_dev->name);
	// 	return -ENODEV;
	// }

	// if (!device_is_ready(data->dma_rx.dma_dev)) {
	// 	LOG_ERR("%s device is not ready", data->dma_rx.dma_dev->name);
	// 	return -ENODEV;
	// }
#endif
	uart_default_config(base, &uart_config);
	clock_set_source_divider(config->clock_name, config->clock_src,
				 config->clock_div);
	clock_add_to_group(config->clock_name, 0);
	uart_config.src_freq_in_hz = clock_get_frequency(config->clock_name);
	uart_config.baudrate = cfg->baudrate;
#ifdef CONFIG_UART_ASYNC_API
	uart_config.fifo_enable = true;
    uart_config.dma_enable = true;
	uart_config.tx_fifo_level = uart_tx_fifo_trg_not_full;
	uart_config.rx_fifo_level = uart_rx_fifo_trg_not_empty; /* this config should not change */
#endif
	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		parity = parity_none;
		break;
	case UART_CFG_PARITY_ODD:
		parity = parity_odd;
		break;
	case UART_CFG_PARITY_EVEN:
		parity = parity_even;
		break;
	default:
		return -ENOTSUP;
	}
	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		uart_config.word_length  = word_length_5_bits;
		break;
	case UART_CFG_DATA_BITS_6:
		uart_config.word_length  = word_length_6_bits;
		break;
	case UART_CFG_DATA_BITS_7:
		uart_config.word_length  = word_length_7_bits;
		break;
	case UART_CFG_DATA_BITS_8:
		uart_config.word_length  = word_length_8_bits;
		break;
	default:
		return -ENOTSUP;
	}
	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		uart_config.num_of_stop_bits = stop_bits_1;
		break;
	case UART_CFG_STOP_BITS_1_5:
		uart_config.num_of_stop_bits = stop_bits_1_5;
		break;
	case UART_CFG_STOP_BITS_2:
		uart_config.num_of_stop_bits = stop_bits_2;
		break;
	default:
		return -ENOTSUP;
	}
	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		uart_config.modem_config.auto_flow_ctrl_en = false;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		uart_config.modem_config.auto_flow_ctrl_en = true;
		break;
	default:
		return -ENOTSUP;
	}
	uart_config.parity = parity;
	if (config->loopback_en) {
		uart_modem_enable_loopback(base);
	} else {
		uart_modem_disable_loopback(base);
	}
	stat = uart_init(base, &uart_config);
	return stat;
}

static int uart_hpm_init(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;
	struct uart_hpm_data *const data = dev->data;
	struct uart_config *uart_api_config = &data->uart_config;
	int ret;

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}
	uart_api_config->baudrate = cfg->baud_rate;
	uart_api_config->parity = UART_CFG_PARITY_NONE;
	uart_api_config->stop_bits = UART_CFG_STOP_BITS_1;
	uart_api_config->data_bits = UART_CFG_DATA_BITS_8;
	uart_api_config->flow_ctrl = cfg->flow_ctrl;
	uart_hpm_configure_init(dev, uart_api_config);
#ifdef CONFIG_UART_ASYNC_API
	k_work_init_delayable(&data->rx_timeout_work, uart_hpm_rx_timeout_handler);
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static bool uart_hpm_async_busy(const struct device *dev)
{
#ifdef CONFIG_UART_ASYNC_API
	struct uart_hpm_data *data = dev->data;
	const struct uart_hpm_cfg *cfg = dev->config;
	struct dma_status status;

	if (!cfg->dma_enabled) {
		return false;
	}

	if (data->dma_rxinfo.current_buf != NULL) {
		return true;
	}

	if ((dma_get_status(data->dma_tx.dma_dev, data->dma_tx.channel,
			    &status) != 0) || status.busy) {
		return true;
	}

	if ((dma_get_status(data->dma_rx.dma_dev, data->dma_rx.channel,
			    &status) != 0) || status.busy) {
		return true;
	}
#else
	ARG_UNUSED(dev);
#endif

	return false;
}

static int uart_hpm_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
	const struct uart_hpm_cfg *cfg = dev->config;
	struct uart_hpm_data *data = dev->data;
	UART_Type *base = cfg->base;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		if (data->pm_suspended) {
			return 0;
		}

		if (uart_hpm_async_busy(dev)) {
			return -EBUSY;
		}

		/* Do not gate the clock while a polled/IRQ TX byte is pending. */
		for (uint32_t retry = 0U; retry < 100000U; retry++) {
			if (uart_check_status(base, uart_stat_transmitter_empty) != 0U) {
				break;
			}
			if (retry == 99999U) {
				return -EBUSY;
			}
		}

		data->pm_ier = base->IER;
		base->IER = 0U;
		clock_remove_from_group(cfg->clock_name, 0);
		data->pm_suspended = true;
		return 0;

	case PM_DEVICE_ACTION_RESUME:
		if (!data->pm_suspended) {
			return 0;
		}

		if (pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT) < 0) {
			return -EIO;
		}

		if (uart_hpm_configure_init(dev, &data->uart_config) !=
		    status_success) {
			return -EIO;
		}

		base->IER = data->pm_ier;
		data->pm_suspended = false;
		return 0;

	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_PM_DEVICE */

static int uart_hpm_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	if (status_success != uart_receive_byte((UART_Type *)cfg->base, c)) {
		return 1;
	} else {
		return 0;
	}
}

static void uart_hpm_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	uart_send_byte((UART_Type *)cfg->base, c);
}

static int uart_hpm_err_check(const struct device *dev)
{
	int errors = 0;

	return errors;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

int uart_hpm_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	const struct uart_hpm_cfg *const cfg = dev->config;
	uint8_t num_tx = 0U;

	if (uart_check_status((UART_Type *)cfg->base, uart_stat_transmitter_empty) != 0) {
		while ((len - num_tx > 0) && (num_tx < 16)) {
			uart_send_byte((UART_Type *)cfg->base, tx_data[num_tx++]);
		}
	}

	return num_tx;
}

int uart_hpm_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	const struct uart_hpm_cfg *const cfg = dev->config;
	uint8_t num_rx = 0U;

	while ((size - num_rx > 0) &&
		(uart_check_status((UART_Type *)cfg->base, uart_stat_data_ready) != 0)) {
		uart_receive_byte((UART_Type *)cfg->base, &rx_data[num_rx++]);
	}

	return num_rx;
}

void uart_hpm_irq_tx_enable(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	uart_enable_irq((UART_Type *)cfg->base, uart_intr_tx_slot_avail);
}

void uart_hpm_irq_tx_disable(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	uart_disable_irq((UART_Type *)cfg->base, uart_intr_tx_slot_avail);
}

int uart_hpm_irq_tx_ready(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	if (uart_check_status((UART_Type *)cfg->base, uart_stat_transmitter_empty) != 0) {
		return 1;
	} else {
		return 0;
	}
}

int uart_hpm_irq_tx_complete(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	if (uart_check_status((UART_Type *)cfg->base, uart_stat_transmitter_empty) != 0) {
		return 1;
	} else {
		return 0;
	}
}

void uart_hpm_irq_rx_enable(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	uart_enable_irq((UART_Type *)cfg->base, uart_intr_rx_data_avail_or_timeout);
}

void uart_hpm_irq_rx_disable(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	uart_disable_irq((UART_Type *)cfg->base, uart_intr_rx_data_avail_or_timeout);
}

int uart_hpm_irq_rx_ready(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;

	if (uart_check_status((UART_Type *)cfg->base, uart_stat_data_ready) != 0) {
		return 1;
	} else {
		return 0;
	}
}

void uart_hpm_irq_err_enable(const struct device *dev)
{
	/*This feature is not supported by the hpmicro driver */
}

void uart_hpm_irq_err_disable(const struct device *dev)
{
	/*This feature is not supported by the hpmicro driver */
}

int uart_hpm_irq_is_pending(const struct device *dev)
{
	const struct uart_hpm_cfg *const cfg = dev->config;
	uint32_t irq_id;

	irq_id = uart_get_irq_id((UART_Type *)cfg->base);
	if ((irq_id & uart_intr_id_rx_data_avail) ||
	(irq_id & uart_intr_id_tx_slot_avail)) {
		return 1;
	} else {
		return 0;
	}
}

void uart_hpm_irq_callback_set(const struct device *dev,
				 uart_irq_callback_user_data_t cb,
				 void *user_data)
{
	struct uart_hpm_data *const dev_data = dev->data;

	dev_data->user_cb = cb;
	dev_data->user_data = user_data;
}

__attribute__((section(".isr"))) static void uart_hpm_isr(const struct device *dev)
{
	const struct uart_hpm_cfg *cfg = dev->config;
	struct uart_hpm_data * const dev_data = dev->data;

	if (cfg->dma_enabled) {
		uart_hpm_async_hw_idle_isr(dev);
	}

	if (dev_data->user_cb) {
		dev_data->user_cb(dev, dev_data->user_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
static void async_evt_rx_rdy(const struct device *dev);
static void uart_hpm_rx_timeout_handler(struct k_work *work);

static inline void async_timer_start(struct k_work_delayable *work, size_t timeout_us)
{
	if ((timeout_us != SYS_FOREVER_US) && (timeout_us != 0)) {
		LOG_DBG("async timer started for %d us", timeout_us);
		k_work_reschedule(work, K_USEC(timeout_us));
	}
}

static inline void async_timer_stop(struct k_work_delayable *work)
{
	(void)k_work_cancel_delayable(work);
}

static size_t uart_hpm_async_rx_peek(const struct device *dev)
{
	struct uart_hpm_data *data = dev->data;
	struct dma_status stat;
	int ret;

	if (data->dma_rxinfo.current_buf == NULL) {
		return 0;
	}

	ret = dma_get_status(data->dma_rx.dma_dev, data->dma_rx.channel, &stat);
	if (ret) {
		return 0;
	}

	if (data->dma_rxinfo.current_total_size <
	    (data->dma_rxinfo.current_offset + stat.pending_length)) {
		return 0;
	}

	return data->dma_rxinfo.current_total_size -
	       data->dma_rxinfo.current_offset -
	       stat.pending_length;
}

static void uart_hpm_async_rx_idle_flush(const struct device *dev)
{
	struct uart_hpm_data *data = dev->data;
	size_t rx_rcv_len = uart_hpm_async_rx_peek(dev);

	if (rx_rcv_len == 0U) {
		return;
	}

	dma_suspend(data->dma_rx.dma_dev, data->dma_rx.channel);
	data->dma_rxinfo.current_size = rx_rcv_len;
	async_evt_rx_rdy(dev);
	dma_resume(data->dma_rx.dma_dev, data->dma_rx.channel);
	data->rx_idle_last_got = data->dma_rxinfo.current_offset;
}

static void uart_hpm_rx_timeout_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_hpm_data *data =
		CONTAINER_OF(dwork, struct uart_hpm_data, rx_timeout_work);
	const struct device *dev = data->dma_rxinfo.dev;
	size_t got;

	if ((dev == NULL) || (data->dma_rxinfo.current_buf == NULL)) {
		return;
	}

	got = uart_hpm_async_rx_peek(dev);
	if (got > data->rx_idle_last_got) {
		/* Still receiving — wait another idle window. */
		data->rx_idle_last_got = got;
	} else if (got > 0U) {
		uart_hpm_async_rx_idle_flush(dev);
	}

	async_timer_start(&data->rx_timeout_work, data->idle_time_out_us);
}

static void async_user_callback(const struct device *dev, struct uart_event *evt)
{
	const struct uart_hpm_data *data = dev->data;

	if (data->idle_user_callback) {
		data->idle_user_callback(dev, evt, data->idle_user_data);
	}
}

static void async_evt_tx_done(const struct device *dev)
{
	struct uart_hpm_data *data = dev->data;

	LOG_DBG("TX done: %d", data->dma_txinfo.buff_size);
	struct uart_event event = {
		.type = UART_TX_DONE,
		.data.tx.buf = data->dma_txinfo.dst_addr,
		.data.tx.len = data->dma_txinfo.buff_size
	};

	/* Reset TX Buffer */
	data->dma_txinfo.dst_addr = NULL;
	data->dma_txinfo.buff_size = 0U;

	async_user_callback(dev, &event);
}

static void async_evt_rx_rdy(const struct device *dev)
{
	struct uart_hpm_data * const data = dev->data;
    dma_info_t dma_rxinfo = data->dma_rxinfo;

	struct uart_event event = {
		.type = UART_RX_RDY,
		.data.rx.buf = dma_rxinfo.current_buf,
		.data.rx.len = dma_rxinfo.current_size,
		.data.rx.offset = dma_rxinfo.current_offset
	};

	/* Update the current pos for new data */
	data->dma_rxinfo.current_offset += dma_rxinfo.current_size;

	/* Only send event for new data */
	if (event.data.rx.len > 0) {
		async_user_callback(dev, &event);
	}
}

static void async_evt_rx_buf_request(const struct device *dev)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	async_user_callback(dev, &evt);
}

static void async_evt_rx_buf_release(const struct device *dev)
{
	struct uart_hpm_data *data = dev->data;
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = data->dma_rxinfo.current_buf,
	};
	async_user_callback(dev, &evt);
}

static void uart_hpm_async_rx_flush(const struct device *dev)
{
	struct uart_hpm_data *data = dev->data;
	size_t rx_rcv_len;
	struct dma_status stat;
	int ret;

	ret = dma_get_status(data->dma_rx.dma_dev, data->dma_rx.channel, &stat);
	if (ret) {
		LOG_ERR("ERR dma get status.\r\n");
	}
	rx_rcv_len = data->dma_rxinfo.current_total_size - 
				 data->dma_rxinfo.current_offset - 
				 stat.pending_length;
	if (rx_rcv_len > 0) {
		data->dma_rxinfo.current_size = rx_rcv_len;
		async_evt_rx_rdy(dev);
	} else {
		data->dma_rxinfo.current_size = 0;
	}
}

static uint32_t uart_hpm_async_tx_flush(const struct device *dev)
{
	struct uart_hpm_data *data = dev->data;
	size_t rx_rcv_len;
	struct dma_status stat;
	int ret;

	ret = dma_get_status(dev, data->dma_tx.channel, &stat);
	if (ret) {
		LOG_ERR("ERR dma get status.\r\n");
	}
	if (!ret) {
		rx_rcv_len = data->dma_txinfo.buff_size - 
					stat.pending_length;
	} else {
		rx_rcv_len = 0;
	}
	return rx_rcv_len;
}

static int uart_hpm_rx_disable(const struct device *dev)
{
	struct uart_hpm_data *data = dev->data;
	const struct uart_hpm_cfg *config = dev->config;
	UART_Type *uart_base = config->base;
	unsigned int key;

	if (!config->dma_enabled) {
		return -ENOTSUP;
	}

	key = irq_lock();

	async_timer_stop(&data->rx_timeout_work);

#if UART_HPM_HW_RX_IDLE
	{
		uart_rxline_idle_config_t idle_cfg = {0};

		idle_cfg.detect_enable = false;
		idle_cfg.detect_irq_enable = false;
		idle_cfg.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
		idle_cfg.threshold = 20U;
		(void)uart_init_rxline_idle_detection(uart_base, idle_cfg);
		uart_clear_rxline_idle_flag(uart_base);
	}
#else
	ARG_UNUSED(uart_base);
	gptmr_clear_status(data->gptmr_info.ptr, GPTMR_CH_CMP_STAT_MASK(data->gptmr_info.cmp_ch, 0)); /* clear cmp status */
	gptmr_stop_counter(data->gptmr_info.ptr, data->gptmr_info.cmp_ch); /* stop counter */
	gptmr_channel_reset_count(data->gptmr_info.ptr, data->gptmr_info.cmp_ch); /* clear counter */
	gptmr_disable_irq(data->gptmr_info.ptr, GPTMR_CH_CMP_IRQ_MASK(data->gptmr_info.cmp_ch, 0)); /* disable cmp irq */
#endif

	dma_suspend(data->dma_rx.dma_dev, data->dma_rx.channel);
	uart_hpm_async_rx_flush(dev);
	async_evt_rx_buf_release(dev);

		/* disable dma channel */
	dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);

	data->dma_rxinfo.next_dst_addr = NULL;
	data->dma_rxinfo.next_buff_size = 0;
	data->dma_rxinfo.current_mode = 0;
	data->dma_rxinfo.current_offset = 0;
	data->dma_rxinfo.current_buf = 0;
	data->dma_rxinfo.current_total_size = 0;
	data->dma_rxinfo.current_size = 0;

	struct uart_event disabled_event = {
		.type = UART_RX_DISABLED
	};

	async_user_callback(dev, &disabled_event);
	irq_unlock(key);
	return 0;
}

#if UART_HPM_HW_RX_IDLE
static void uart_hpm_async_hw_idle_isr(const struct device *dev)
{
	const struct uart_hpm_cfg *cfg = dev->config;
	struct uart_hpm_data *data = dev->data;
	UART_Type *uart_base = cfg->base;

	if (!uart_is_rxline_idle(uart_base)) {
		return;
	}

	uart_clear_rxline_idle_flag(uart_base);
	uart_hpm_async_rx_idle_flush(dev);
	async_timer_start(&data->rx_timeout_work, data->idle_time_out_us);
}
#else
/* this handler used to detect uart idle status */
/* when gptmr cap irq occurs, think as uart rx start, enable gptmr cmp irq for rx idle */
/* when gptmr cmp irq occurs, think as uart rx idle status, close gptmr cmp irq, close uart dma */
void gptmr_detect_uart_rx_idle_handler(const struct device *dev)
{
	struct uart_hpm_data * const data = dev->data;
    gptmr_info_t gptmr_info = data->gptmr_info;

    /* cmp */
    if (gptmr_check_status(gptmr_info.ptr, GPTMR_CH_CMP_STAT_MASK(gptmr_info.cmp_ch, 0))) {
        gptmr_clear_status(gptmr_info.ptr, GPTMR_CH_CMP_STAT_MASK(gptmr_info.cmp_ch, 0)); /* clear cmp status */
        gptmr_stop_counter(gptmr_info.ptr, gptmr_info.cmp_ch); /* stop counter */
        gptmr_channel_reset_count(gptmr_info.ptr, gptmr_info.cmp_ch); /* clear counter */
        gptmr_disable_irq(gptmr_info.ptr, GPTMR_CH_CMP_IRQ_MASK(gptmr_info.cmp_ch, 0)); /* disable cmp irq */

        /* disable dma channel */
        dma_suspend(data->dma_rx.dma_dev, data->dma_rx.channel);

        /* uart rx idle callback */
        uart_hpm_async_rx_flush(dev);
		dma_resume(data->dma_rx.dma_dev, data->dma_rx.channel);
		gptmr_enable_irq(gptmr_info.ptr, GPTMR_CH_CAP_IRQ_MASK(gptmr_info.cap_ch));
		gptmr_start_counter(gptmr_info.ptr, gptmr_info.cap_ch);
    }

    /* capture */
    if (gptmr_check_status(gptmr_info.ptr, GPTMR_CH_CAP_STAT_MASK(gptmr_info.cap_ch))) {
        gptmr_clear_status(gptmr_info.ptr, GPTMR_CH_CAP_STAT_MASK(gptmr_info.cap_ch)); /* clear capture status */
        gptmr_stop_counter(gptmr_info.ptr, gptmr_info.cap_ch);        /* stop counter */
        gptmr_channel_reset_count(gptmr_info.ptr, gptmr_info.cap_ch); /* clear counter */
        gptmr_disable_irq(gptmr_info.ptr, GPTMR_CH_CAP_IRQ_MASK(gptmr_info.cap_ch)); /* disable capture irq */
		if (data->dma_rxinfo.current_mode == 0) {
			/* config dma to transfer uart rx data */
			uart_hpm_dma_rx_load(dev, data->dma_rxinfo.current_buf, data->dma_rxinfo.current_total_size);
			dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			data->dma_rxinfo.current_mode = 1;
		}
        /* start gptmr cmp for rx idle detection */
        gptmr_enable_irq(gptmr_info.ptr, GPTMR_CH_CMP_IRQ_MASK(gptmr_info.cmp_ch, 0));
        gptmr_start_counter(gptmr_info.ptr, gptmr_info.cmp_ch);
    }
}

static void uart_hpm_timer_isr(const struct device *dev)
{
	gptmr_detect_uart_rx_idle_handler(dev);
}
#endif /* UART_HPM_HW_RX_IDLE */

static int uart_hpm_dma_replace_rx_buffer(const struct device *dev)
{
	struct uart_hpm_data * data = dev->data;
	unsigned int key = irq_lock();

	if (data->dma_rxinfo.next_dst_addr != NULL) {
#if !UART_HPM_HW_RX_IDLE
		gptmr_info_t gptmr_info = data->gptmr_info;

		gptmr_clear_status(gptmr_info.ptr, GPTMR_CH_CMP_STAT_MASK(gptmr_info.cmp_ch, 0)); /* clear cmp status */
		gptmr_stop_counter(gptmr_info.ptr, gptmr_info.cmp_ch); /* stop counter */
		gptmr_channel_reset_count(gptmr_info.ptr, gptmr_info.cmp_ch); /* clear counter */
		gptmr_disable_irq(gptmr_info.ptr, GPTMR_CH_CMP_IRQ_MASK(gptmr_info.cmp_ch, 0)); /* disable cmp irq */
#endif
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
		data->dma_rxinfo.current_buf = data->dma_rxinfo.next_dst_addr;
		data->dma_rxinfo.current_total_size = data->dma_rxinfo.next_buff_size;
		data->dma_rxinfo.current_size = 0;
		data->dma_rxinfo.next_dst_addr = NULL;
		data->dma_rxinfo.next_buff_size = 0;
		data->dma_rxinfo.current_mode = 0;
		data->dma_rxinfo.current_offset = 0;
		async_evt_rx_buf_request(dev);
#if UART_HPM_HW_RX_IDLE
		uart_hpm_dma_rx_load(dev, data->dma_rxinfo.current_buf,
				     data->dma_rxinfo.current_total_size);
		dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
		data->dma_rxinfo.current_mode = 1;
#else
		gptmr_enable_irq(gptmr_info.ptr, GPTMR_CH_CAP_IRQ_MASK(gptmr_info.cap_ch));
		gptmr_start_counter(gptmr_info.ptr, gptmr_info.cap_ch);
#endif
		irq_unlock(key);
		return 0;
	} else {
		irq_unlock(key);
		return -EFAULT;
	}
}

static int uart_hpm_callback_set(const struct device *dev, uart_callback_t callback,
				    void *user_data)
{
	struct uart_hpm_data *data = dev->data;

	data->idle_user_callback = callback;
	data->idle_user_data = user_data;

	return 0;
}

static int uart_hpm_tx(const struct device *dev, const uint8_t *buf, size_t len,
			  int32_t timeout_us)
{
	int stat;
	struct uart_hpm_data *data = dev->data;
	const struct uart_hpm_cfg *config = dev->config;
	UART_Type *uart_base = config->base;

	if (!config->dma_enabled) {
		return -ENOTSUP;
	}

	uart_reset_tx_fifo(uart_base);
	data->dma_txinfo.dst_addr = (uint8_t *)buf;
	data->dma_txinfo.buff_size = len;
	data->dma_txinfo.dev = (struct device*)dev;
    /* Sending data for the first time */
	stat = uart_hpm_dma_tx_load(dev, buf, len);
	if (stat) {
		return stat;
	}
	stat = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
	return stat;
}

static int uart_hpm_tx_abort(const struct device *dev)
{
	int stat;
	struct uart_hpm_data *data = dev->data;
	const struct uart_hpm_cfg *config = dev->config;
	uint32_t len;

	if (!config->dma_enabled) {
		return -ENOTSUP;
	}

	stat = dma_suspend(dev, data->dma_tx.channel);
	if (stat) {
		LOG_ERR("ERR dma suspend error.\r\n");
		return stat;
	}
	len = uart_hpm_async_tx_flush(dev);
	struct uart_event tx_aborted_event = {
			.type = UART_TX_ABORTED,
			.data.tx.buf = data->dma_txinfo.dst_addr,
			.data.tx.len = len
	};
	stat = dma_stop(dev, data->dma_tx.channel);
	if (stat) {
		LOG_ERR("ERR dma stop error.\r\n");
		return stat;
	}
	async_user_callback(dev, &tx_aborted_event);

	return 0;
}

static int uart_hpm_rx_enable(const struct device *dev, uint8_t *buf, const size_t len,
				 const int32_t timeout_us)
{
	struct uart_hpm_data *data = dev->data;
	const struct uart_hpm_cfg *config = dev->config;
	UART_Type *uart_base = config->base;
	volatile uint8_t buff;

	unsigned int key;

	if (!config->dma_enabled) {
		return -ENOTSUP;
	}

	key = irq_lock();

	data->dma_rxinfo.dst_addr = buf;
	data->dma_rxinfo.buff_size = len;
	data->dma_rxinfo.next_dst_addr = NULL;
	data->dma_rxinfo.next_buff_size = 0;
	data->dma_rxinfo.current_mode = 0;
	data->idle_time_out_us = timeout_us;
	data->dma_rxinfo.current_offset = 0;
	data->dma_rxinfo.current_buf = data->dma_rxinfo.dst_addr;
	data->dma_rxinfo.current_total_size = data->dma_rxinfo.buff_size;
	data->dma_rxinfo.current_size = 0;
	data->dma_rxinfo.dev = (struct device *)dev;
	uart_reset_rx_fifo(uart_base);
	while ((uart_base->LSR & UART_LSR_OE_MASK) || \
			(uart_base->LSR & UART_LSR_DR_MASK)) {
		buff = uart_base->RBR & UART_RBR_RBR_MASK;
	}
	async_evt_rx_buf_request(dev);
#if UART_HPM_HW_RX_IDLE
	{
		uart_rxline_idle_config_t idle_cfg = {0};
		uint32_t threshold = 20U;

		if ((timeout_us != SYS_FOREVER_US) && (timeout_us > 0)) {
			threshold = (uint32_t)(((uint64_t)timeout_us *
						data->uart_config.baudrate) / 1000000ULL);
			if (threshold < 1U) {
				threshold = 1U;
			} else if (threshold > 255U) {
				threshold = 255U;
			}
		}

		idle_cfg.detect_enable = true;
		idle_cfg.detect_irq_enable = true;
		idle_cfg.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
		idle_cfg.threshold = (uint8_t)threshold;
		(void)uart_init_rxline_idle_detection(uart_base, idle_cfg);
		uart_clear_rxline_idle_flag(uart_base);
	}
#else
	config_gptmr_to_detect_uart_rx_start(&data->gptmr_info, &data->trgm_info);
	config_gptmr_to_detect_uart_rx_idle(&data->gptmr_info, &data->trgm_info,
					    data->idle_time_out_us);
#endif
	uart_hpm_dma_rx_load(dev, data->dma_rxinfo.current_buf, data->dma_rxinfo.current_total_size);
	dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
	data->dma_rxinfo.current_mode = 1;
	data->rx_idle_last_got = 0;
	async_timer_start(&data->rx_timeout_work, data->idle_time_out_us);
	irq_unlock(key);

	return 0;
}

static int mcux_lpuart_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct uart_hpm_data *data = dev->data;
	const struct uart_hpm_cfg *config = dev->config;

	if (!config->dma_enabled) {
		return -ENOTSUP;
	}

	assert(data->dma_rxinfo.next_dst_addr == NULL);
	assert(data->dma_rxinfo.next_buff_size == 0);

	data->dma_rxinfo.next_dst_addr = buf;
	data->dma_rxinfo.next_buff_size = len;

	return 0;
}

static void uart_hpm_dma_callback(const struct device *dev, void *arg, uint32_t channel, int status)
{
	struct uart_hpm_data *data = arg;

	if (status != 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
	} else {
		/* identify the origin of this callback */
		if (channel == data->dma_tx.channel) {
			/* this part of the transfer ends */
			async_evt_tx_done(data->dma_txinfo.dev);
		} else if (channel == data->dma_rx.channel) {
			/* this part of the transfer ends */
			uart_hpm_async_rx_flush(data->dma_rxinfo.dev);
			if (data->dma_rxinfo.current_size > 0) {
				async_evt_rx_buf_release(data->dma_rxinfo.dev);
			}
			if (uart_hpm_dma_replace_rx_buffer(data->dma_rxinfo.dev) != 0) {
				uart_hpm_rx_disable(data->dma_rxinfo.dev);
			}
		} else {
			LOG_ERR("DMA callback channel %d is not valid.",
								channel);
		}
	}
}

#endif /* CONFIG_UART_ASYNC_API */

static const struct uart_driver_api uart_hpm_driver_api = {
	.poll_in = uart_hpm_poll_in,
	.poll_out = uart_hpm_poll_out,
	.err_check = uart_hpm_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_hpm_configure_init,
	.config_get = uart_hpm_configure_get,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_hpm_callback_set,
	.tx = uart_hpm_tx,
	.tx_abort = uart_hpm_tx_abort,
	.rx_enable = uart_hpm_rx_enable,
	.rx_buf_rsp = mcux_lpuart_rx_buf_rsp,
	.rx_disable = uart_hpm_rx_disable,
#endif /* CONFIG_UART_ASYNC_API */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill		  = uart_hpm_fifo_fill,
	.fifo_read		  = uart_hpm_fifo_read,
	.irq_tx_enable	  = uart_hpm_irq_tx_enable,
	.irq_tx_disable	  = uart_hpm_irq_tx_disable,
	.irq_tx_ready	  = uart_hpm_irq_tx_ready,
	.irq_tx_complete  = uart_hpm_irq_tx_complete,
	.irq_rx_enable	  = uart_hpm_irq_rx_enable,
	.irq_rx_disable	  = uart_hpm_irq_rx_disable,
	.irq_rx_ready	  = uart_hpm_irq_rx_ready,
	.irq_err_enable	  = uart_hpm_irq_err_enable,
	.irq_err_disable  = uart_hpm_irq_err_disable,
	.irq_is_pending	  = uart_hpm_irq_is_pending,
	.irq_callback_set = uart_hpm_irq_callback_set,
#endif
};


#define UART_HPMICRO_IRQ_FLAGS_SENSE0(n) 0
#define UART_HPMICRO_IRQ_FLAGS_SENSE1(n) DT_INST_IRQ(n, sense)
#define UART_HPMICRO_IRQ_FLAGS(n) \
	_CONCAT(UART_HPMICRO_IRQ_FLAGS_SENSE, \
	DT_INST_IRQ_HAS_CELL(n, sense))(n)

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define DEV_CONFIG_IRQ_FUNC_INIT(n) \
	.irq_config_func = irq_config_func##n,
#define UART_HPMICRO_IRQ_FUNC_DECLARE(n) \
	static void irq_config_func##n(const struct device *dev);
#ifndef CONFIG_UART_ASYNC_API
#define UART_HPMICRO_IRQ_FUNC_DEFINE(n)	\
	static void irq_config_func##n(const struct device *dev)	\
	{	\
		ARG_UNUSED(dev);	\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	\
		uart_hpm_isr, DEVICE_DT_INST_GET(n),	\
		UART_HPMICRO_IRQ_FLAGS(n));		\
		irq_enable(DT_INST_IRQN(n));	\
	}
#else /* CONFIG_UART_ASYNC_API */
#if UART_HPM_HW_RX_IDLE
#define UART_HPMICRO_IRQ_FUNC_DEFINE(n)	\
	static void irq_config_func##n(const struct device *dev)	\
	{	\
		ARG_UNUSED(dev);	\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, uart, irq), \
		DT_INST_IRQ_BY_NAME(n, uart, priority),	\
		uart_hpm_isr, DEVICE_DT_INST_GET(n),	\
		UART_HPMICRO_IRQ_FLAGS(n));		\
		irq_enable(DT_INST_IRQ_BY_NAME(n, uart, irq));	\
	}
#else
#define UART_HPMICRO_IRQ_FUNC_DEFINE(n)	\
	static void irq_config_func##n(const struct device *dev)	\
	{	\
		ARG_UNUSED(dev);	\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, uart, irq), \
		DT_INST_IRQ_BY_NAME(n, uart, priority),	\
		uart_hpm_isr, DEVICE_DT_INST_GET(n),	\
		UART_HPMICRO_IRQ_FLAGS(n));		\
		irq_enable(DT_INST_IRQ_BY_NAME(n, uart, irq));	\
		\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, idletimer, irq), \
		DT_INST_IRQ_BY_NAME(n, idletimer, priority),	\
		uart_hpm_timer_isr, DEVICE_DT_INST_GET(n),	\
		UART_HPMICRO_IRQ_FLAGS(n));		\
		irq_enable(DT_INST_IRQ_BY_NAME(n, idletimer, irq));	\
	}
#endif /* UART_HPM_HW_RX_IDLE */

#endif /* !CONFIG_UART_ASYNC_API */

#else /* !CONFIG_UART_INTERRUPT_DRIVEN */
#define DEV_CONFIG_IRQ_FUNC_INIT(n)
#define UART_HPMICRO_IRQ_FUNC_DECLARE(n)
#define UART_HPMICRO_IRQ_FUNC_DEFINE(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
#if UART_HPM_HW_RX_IDLE
#define UART_HPMICRO_ASYNC_DATA_INIT(n)	\
		.uart_rx_idle = false,	\
		.uart_receive_data_size = 0,	\
		.uart_info = {	\
					.ptr = (UART_Type *)DT_INST_REG_ADDR(n),	\
					.baudrate = DT_INST_PROP(n, current_speed),	\
		},
#else
#define UART_HPMICRO_ASYNC_DATA_INIT(n)	\
		.uart_rx_idle = false,	\
		.uart_receive_data_size = 0,	\
		.uart_info = {	\
					.ptr = (UART_Type *)DT_INST_REG_ADDR(n),	\
					.baudrate = DT_INST_PROP(n, current_speed),	\
		},	\
		.trgm_info = {	\
                .ptr = (TRGM_Type *)DT_INST_PROP(n, uart_idle_trgm_reg),	\
                .pin_index = DT_INST_PROP(n, uart_idle_trgm_pin),	\
                .input_src = DT_INST_PROP(n, uart_idle_trgm_input_src),	\
                .output_gptmr_in = DT_INST_PROP(n, uart_idle_trgm_output_gptmr_in),	\
                .output_gptmr_synci = DT_INST_PROP(n, uart_idle_trgm_output_gptmr_sync),	\
    	},	\
		.gptmr_info = {	\
                .ptr = (GPTMR_Type *)DT_INST_PROP(n, uart_idle_gptmr_reg),	\
                .clock_name = DT_INST_PROP(n, uart_idle_gptmr_clock_name),	\
                .cmp_ch = 0,	\
                .cap_ch = 2,	\
    	},
#endif /* UART_HPM_HW_RX_IDLE */

#define UART_DMA_CHANNELS_DATA_INIT_1(n)		\
		.dma_tx = {						\
			.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)), \
			.channel =					\
				DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),	\
			.dma_cfg = {					\
				.channel_direction = MEMORY_TO_PERIPHERAL,	\
				.dma_callback = uart_hpm_dma_callback,		\
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
				.dma_callback = uart_hpm_dma_callback,		\
				.source_data_size = 1,				\
				.dest_data_size = 1,				\
				.block_count = 1,		\
				.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, rx, source) \
			}							\
		},
#define UART_DMA_CHANNELS_DATA_INIT_0(n)
#define UART_DMA_CHANNELS_DATA_INIT(n) \
	_CONCAT(UART_DMA_CHANNELS_DATA_INIT_, \
		DT_INST_NODE_HAS_PROP(n, dmas))(n)

#else
#define UART_HPMICRO_ASYNC_DATA_INIT(n)
#define UART_DMA_CHANNELS_DATA_INIT(n)
#endif

#define PINCTRL_INIT(n) .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),
#define PINCTRL_DEFINE(n) PINCTRL_DT_INST_DEFINE(n);

#define HPM_UART_INIT(n)	\
		PINCTRL_DEFINE(n)	\
		UART_HPMICRO_IRQ_FUNC_DECLARE(n)	\
		PM_DEVICE_DT_INST_DEFINE(n, uart_hpm_pm_action);	\
		static struct uart_hpm_data uart_hpm_data_##n = {	\
		UART_HPMICRO_ASYNC_DATA_INIT(n)	\
		UART_DMA_CHANNELS_DATA_INIT(n)			\
		};									\
		static const struct uart_hpm_cfg uart_hpm_config_##n = {	\
			.base = (UART_Type *)DT_INST_REG_ADDR(n),	\
			.clock_name = DT_INST_CLOCKS_CELL(n, name),	\
			.clock_src = DT_INST_CLOCKS_CELL(n, src),	\
			.clock_div = DT_INST_CLOCKS_CELL(n, div),	\
			.parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),	\
			.loopback_en = DT_INST_PROP(n, loopback),				\
			.dma_enabled = DT_INST_NODE_HAS_PROP(n, dmas),			\
			.baud_rate = DT_INST_PROP(n, current_speed),	\
			.flow_ctrl = DT_INST_PROP(n, hw_flow_control) ?				\
			UART_CFG_FLOW_CTRL_RTS_CTS : UART_CFG_FLOW_CTRL_NONE,		\
			DEV_CONFIG_IRQ_FUNC_INIT(n)		\
			PINCTRL_INIT(n)		\
		};	\
		DEVICE_DT_INST_DEFINE(n, uart_hpm_init,	\
					PM_DEVICE_DT_INST_GET(n),	\
					&uart_hpm_data_##n,	\
					&uart_hpm_config_##n, PRE_KERNEL_1,	\
					CONFIG_SERIAL_INIT_PRIORITY,	\
					&uart_hpm_driver_api);	\
		UART_HPMICRO_IRQ_FUNC_DEFINE(n)

DT_INST_FOREACH_STATUS_OKAY(HPM_UART_INIT)
