/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * MCAN0 internal loopback smoke (no transceiver required).
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define CAN_NODE DT_CHOSEN(zephyr_canbus)
#define TEST_ID  0x123
#define TEST_DLC 8

static const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
static struct k_sem rx_sem;
static struct k_sem tx_sem;
static struct can_frame rx_frame;
static volatile int tx_err;

static void tx_cb(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	tx_err = error;
	k_sem_give(&tx_sem);
}

static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	memcpy(&rx_frame, frame, sizeof(rx_frame));
	k_sem_give(&rx_sem);
}

int main(void)
{
	struct can_frame tx = {0};
	int filter_id;
	int ret;
	const uint8_t payload[TEST_DLC] = {
		0xA5, 0x5A, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55
	};

	k_sem_init(&rx_sem, 0, 1);
	k_sem_init(&tx_sem, 0, 1);

	if (!device_is_ready(can_dev)) {
		printk("CAN not ready\n");
		return 0;
	}

	printk("MCAN0 loopback smoke\n");

	ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	if (ret) {
		printk("set_mode %d\n", ret);
		return 0;
	}
	printk("mode OK\n");

	ret = can_start(can_dev);
	if (ret) {
		printk("start %d\n", ret);
		return 0;
	}
	printk("start OK\n");

	filter_id = can_add_rx_filter(can_dev, rx_cb, NULL,
				      &(const struct can_filter){
					      .id = TEST_ID,
					      .mask = CAN_STD_ID_MASK,
				      });
	if (filter_id < 0) {
		printk("add_filter %d\n", filter_id);
		return 0;
	}
	printk("filter OK id=%d\n", filter_id);

	tx.id = TEST_ID;
	tx.dlc = TEST_DLC;
	memcpy(tx.data, payload, TEST_DLC);

	printk("sending...\n");
	ret = can_send(can_dev, &tx, K_MSEC(500), tx_cb, NULL);
	printk("submit ret=%d\n", ret);
	if (ret) {
		return 0;
	}

	ret = k_sem_take(&tx_sem, K_MSEC(500));
	printk("tx_done ret=%d err=%d\n", ret, tx_err);
	if (ret) {
		return 0;
	}

	ret = k_sem_take(&rx_sem, K_MSEC(500));
	if (ret) {
		printk("RX FAIL timeout\n");
		return 0;
	}

	if ((rx_frame.id != TEST_ID) || (rx_frame.dlc != TEST_DLC) ||
	    (memcmp(rx_frame.data, payload, TEST_DLC) != 0)) {
		printk("RX FAIL mismatch id=0x%x dlc=%u\n",
		       rx_frame.id, rx_frame.dlc);
		return 0;
	}

	printk("CAN_LOOPBACK_OK\n");
	printk("RX OK (MCAN0 internal loopback)\n");
	return 0;
}
