/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI3 master loopback: short MOSI (PC13) to MISO (PC15) on HPM5100EVK.
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

	printk("SPI3 loopback (polling): short MOSI(PC13)<->MISO(PC15)\n");

	while (1) {
		memset(rx, 0, sizeof(rx));
		ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
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
			printk("[%u] OK loopback 8 bytes @ 1MHz\n", n);
		}
		n++;
		k_msleep(1000);
	}

	return 0;
}
