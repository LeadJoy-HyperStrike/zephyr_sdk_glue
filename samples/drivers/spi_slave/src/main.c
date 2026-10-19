/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI3 Zephyr API framed DMA slave — CONFIG_SPI_HPM_SLAVE_FRAMED.
 *
 * Peer: HPM SDK DMA master, Mode3, cmd=0x1A, addr=0x10, 128B (i % 0xFF).
 * Pins: CS=PC06 SCLK=PC10 MOSI=PC13 MISO=PC15 (same-name wiring).
 * Start this slave before the master.
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <hpm_clock_drv.h>

#define SPI_NODE DT_NODELABEL(spi3)
#define XFER_LEN 128U

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

/* Mode3 + slave; frequency unused while waiting on master. */
static const struct spi_config spi_cfg = {
	.frequency = 1000000,
	.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
		     SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
};

static uint8_t tx_buff[XFER_LEN] __aligned(64);
static uint8_t rx_buff[XFER_LEN] __aligned(64);

static int run_one_xfer(uint32_t n)
{
	const struct spi_buf tx_buf = {.buf = tx_buff, .len = sizeof(tx_buff)};
	const struct spi_buf rx_buf = {.buf = rx_buff, .len = sizeof(rx_buff)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	uint32_t err = 0;
	int ret;

	for (uint32_t i = 0; i < XFER_LEN; i++) {
		tx_buff[i] = (uint8_t)(i % 0xFF);
		rx_buff[i] = 0;
	}

	printk("[%u] waiting for master (Zephyr API framed DMA)...\n", n);
	ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		printk("[%u] spi_transceive fail %d\n", n, ret);
		return ret;
	}

	for (uint32_t i = 0; i < XFER_LEN; i++) {
		if (rx_buff[i] != (uint8_t)(i % 0xFF)) {
			err++;
		}
	}

	printk("[%u] frames=%d err=%u\n", n, ret, err);
	if (err == 0U) {
		printk("[%u] OK framed 128B (Zephyr API)\n", n);
		return 0;
	}

	printk(" rx:");
	for (uint32_t i = 0; i < 16; i++) {
		printk(" %02x", rx_buff[i]);
	}
	printk("\n");
	return -EIO;
}

int main(void)
{
	uint32_t n = 0;

	if (!device_is_ready(spi_dev)) {
		printk("SPI3 not ready\n");
		return 0;
	}

	if (!IS_ENABLED(CONFIG_SPI_HPM_SLAVE_FRAMED)) {
		printk("Need CONFIG_SPI_HPM_SLAVE_FRAMED=y\n");
		return 0;
	}

	printk("SPI3 Zephyr API slave framed DMA 128B Mode3 @ %u Hz\n",
	       (unsigned)clock_get_frequency(clock_spi3));
	printk("CS=PC06 SCLK=PC10 MOSI=PC13 MISO=PC15\n");
	printk("Peer: SDK DMA master cmd=0x1A addr=0x10\n");

	while (1) {
		(void)run_one_xfer(n++);
		k_msleep(50);
	}

	return 0;
}
