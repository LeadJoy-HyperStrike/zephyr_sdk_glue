/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART0 async DMA smoke:
 *  1) TX DMA
 *  2) RX DMA fill via internal loopback
 *  3) RX DMA + HW RX-line idle via host bytes on COM (real RX pin)
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <hpm_uart_drv.h>

#define UART_NODE DT_CHOSEN(zephyr_console)

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static UART_Type *const uart_base = (UART_Type *)DT_REG_ADDR(UART_NODE);

static uint8_t tx_buf[64] __aligned(32);
static uint8_t rx_buf0[64] __aligned(32);
static uint8_t rx_buf1[64] __aligned(32);
static uint8_t rx_got[64];

static struct k_sem tx_done;
static struct k_sem rx_rdy;
static struct k_sem rx_disabled;

static volatile int tx_status;
static volatile size_t rx_got_len;

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_TX_DONE:
		tx_status = 0;
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		tx_status = -EIO;
		k_sem_give(&tx_done);
		break;
	case UART_RX_BUF_REQUEST:
		(void)uart_rx_buf_rsp(uart_dev, rx_buf1, sizeof(rx_buf1));
		break;
	case UART_RX_RDY:
		if ((evt->data.rx.len > 0) &&
		    (rx_got_len + evt->data.rx.len <= sizeof(rx_got))) {
			memcpy(&rx_got[rx_got_len],
			       &evt->data.rx.buf[evt->data.rx.offset],
			       evt->data.rx.len);
			rx_got_len += evt->data.rx.len;
		}
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

static int async_tx(const void *msg, size_t len)
{
	int ret;

	memcpy(tx_buf, msg, len);
	k_sem_reset(&tx_done);
	ret = uart_tx(uart_dev, tx_buf, len, SYS_FOREVER_US);
	if (ret) {
		return ret;
	}

	ret = k_sem_take(&tx_done, K_MSEC(1000));
	if (ret) {
		return ret;
	}

	return tx_status;
}

static int check_rx(const char *expect, size_t len)
{
	if ((rx_got_len != len) || (memcmp(rx_got, expect, len) != 0)) {
		printk("RX FAIL len=%u\n", (unsigned int)rx_got_len);
		return -EIO;
	}

	return 0;
}

int main(void)
{
	static const char tx_msg[] = "ASYNC_TX_OK\r\n";
	static const char probe[] = "RX_PROBE";
	static const char idle_probe[] = "IDLEOK";
	const size_t probe_len = sizeof(probe) - 1;
	const size_t idle_len = sizeof(idle_probe) - 1;
	int ret;

	k_sem_init(&tx_done, 0, 1);
	k_sem_init(&rx_rdy, 0, 1);
	k_sem_init(&rx_disabled, 0, 1);

	if (!device_is_ready(uart_dev)) {
		printk("UART not ready\n");
		return 0;
	}

	printk("UART0 async DMA smoke\n");

	ret = uart_callback_set(uart_dev, uart_cb, NULL);
	if (ret) {
		printk("callback_set %d\n", ret);
		return 0;
	}

	ret = async_tx(tx_msg, sizeof(tx_msg) - 1);
	if (ret) {
		printk("TX FAIL %d\n", ret);
		return 0;
	}

	k_msleep(20);
	printk("TX OK (async DMA)\n");

	/* --- Phase A: RX DMA fill (internal loopback) --- */
	memset(rx_got, 0, sizeof(rx_got));
	rx_got_len = 0;
	k_sem_reset(&rx_rdy);
	k_sem_reset(&rx_disabled);

	uart_modem_enable_loopback(uart_base);
	k_msleep(2);

	ret = uart_rx_enable(uart_dev, rx_buf0, probe_len, 50000);
	if (ret) {
		uart_modem_disable_loopback(uart_base);
		printk("rx_enable %d\n", ret);
		return 0;
	}

	ret = async_tx(probe, probe_len);
	if (ret) {
		uart_modem_disable_loopback(uart_base);
		(void)uart_rx_disable(uart_dev);
		printk("RX probe TX FAIL %d\n", ret);
		return 0;
	}

	ret = k_sem_take(&rx_rdy, K_MSEC(500));
	uart_modem_disable_loopback(uart_base);
	(void)uart_rx_disable(uart_dev);
	(void)k_sem_take(&rx_disabled, K_MSEC(200));

	if (ret || check_rx(probe, probe_len)) {
		printk("RX FAIL timeout/data\n");
		return 0;
	}

	printk("ASYNC_RX_OK\n");
	printk("RX OK (async DMA fill)\n");

	/* --- Phase B: HW RX-line idle (host must send IDLEOK) --- */
	memset(rx_got, 0, sizeof(rx_got));
	rx_got_len = 0;
	k_sem_reset(&rx_rdy);
	k_sem_reset(&rx_disabled);

	ret = uart_rx_enable(uart_dev, rx_buf0, sizeof(rx_buf0), 30000);
	if (ret) {
		printk("rx_enable idle %d\n", ret);
		return 0;
	}

	printk("SEND_IDLEOK\n");

	ret = k_sem_take(&rx_rdy, K_MSEC(3000));
	(void)uart_rx_disable(uart_dev);
	(void)k_sem_take(&rx_disabled, K_MSEC(200));

	if (ret) {
		printk("RX IDLE FAIL timeout\n");
		return 0;
	}

	if (check_rx(idle_probe, idle_len)) {
		return 0;
	}

	printk("ASYNC_RX_IDLE_OK\n");
	printk("RX OK (async DMA + HW idle)\n");
	return 0;
}
