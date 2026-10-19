/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI3 master loopback: short MOSI (PC13) to MISO (PC15) on HPM5100EVK.
 * With CONFIG_SPI_ASYNC: spi_transceive_signal + k_poll (IRQ or DMA path).
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SPI_NODE DT_NODELABEL(spi3)

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

/* Mode 3: match SDK polling master (CPOL=1, CPHA=1). HW CS on PC06. */
static struct spi_config spi_cfg = {
	.frequency = 1000000,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
		     SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
};

#if defined(CONFIG_SPI_ASYNC) && defined(CONFIG_POLL)
static struct k_poll_signal async_sig;
static struct k_poll_event async_evt;
#endif

int main(void)
{
	uint8_t tx[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
	uint8_t rx[8];
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret;
	uint32_t n = 0;

	if (!device_is_ready(spi_dev)) {
		printk("SPI3 not ready\n");
		return 0;
	}

#if defined(CONFIG_SPI_ASYNC)
	printk("SPI3 loopback (async");
#if defined(CONFIG_SPI_HPM_SPI_DMA)
	printk("+dma");
#elif defined(CONFIG_SPI_HPM_INTERRUPT)
	printk("+irq");
#endif
	printk(")");
#elif defined(CONFIG_SPI_HPM_SPI_DMA)
	printk("SPI3 loopback (dma)");
#elif defined(CONFIG_SPI_HPM_INTERRUPT)
	printk("SPI3 loopback (irq)");
#else
	printk("SPI3 loopback (polling)");
#endif
#if DT_NODE_HAS_PROP(SPI_NODE, cs_gpios)
	printk(" + gpio-cs");
#else
	printk(" + hw-cs");
#endif
	printk(": short MOSI(PC13)<->MISO(PC15)\n");

#if defined(CONFIG_SPI_ASYNC) && defined(CONFIG_POLL)
	k_poll_signal_init(&async_sig);
	k_poll_event_init(&async_evt, K_POLL_TYPE_SIGNAL,
			  K_POLL_MODE_NOTIFY_ONLY, &async_sig);
#endif

	while (1) {
		memset(rx, 0, sizeof(rx));

#if defined(CONFIG_SPI_ASYNC) && defined(CONFIG_POLL)
		k_poll_signal_reset(&async_sig);
		async_evt.signal->signaled = 0;
		async_evt.state = K_POLL_STATE_NOT_READY;

		ret = spi_transceive_signal(spi_dev, &spi_cfg, &tx_set, &rx_set,
					    &async_sig);
		if (ret == 0) {
			ret = k_poll(&async_evt, 1, K_MSEC(1000));
			if (ret == 0) {
				unsigned int signaled = 0;
				int spi_result = 0;

				k_poll_signal_check(&async_sig, &signaled,
						    &spi_result);
				ret = spi_result;
			} else {
				printk("[%u] async poll timeout/fail: %d\n", n, ret);
				n++;
				k_msleep(1000);
				continue;
			}
		}
#else
		ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
#endif
		if (ret) {
			printk("[%u] spi_transceive failed: %d\n", n, ret);
		} else if (memcmp(tx, rx, sizeof(tx)) != 0) {
			printk("[%u] MISMATCH tx:", n);
			for (size_t i = 0; i < sizeof(tx); i++) {
				printk(" %02x", tx[i]);
			}
			printk(" rx:");
			for (size_t i = 0; i < sizeof(rx); i++) {
				printk(" %02x", rx[i]);
			}
			printk("\n");
		} else {
#if defined(CONFIG_SPI_ASYNC)
			printk("[%u] OK async loopback 8 bytes @ 1MHz\n", n);
#else
			printk("[%u] OK loopback 8 bytes @ 1MHz\n", n);
#endif
		}
		n++;
		k_msleep(1000);
	}

	return 0;
}
