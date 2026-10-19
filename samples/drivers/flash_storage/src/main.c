/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Erase / write / read the board "storage" partition (safe; not XIP code).
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/printk.h>

#define STORAGE_ID FIXED_PARTITION_ID(storage_partition)
#define TEST_OFF   0
#define TEST_LEN   16

int main(void)
{
	const struct flash_area *fa;
	uint8_t wr[TEST_LEN];
	uint8_t rd[TEST_LEN];
	int ret;
	int i;

	ret = flash_area_open(STORAGE_ID, &fa);
	if (ret) {
		printk("open storage %d\n", ret);
		return 0;
	}

	printk("Flash storage smoke\n");
	printk("  off=0x%x size=0x%x erase=%u\n",
	       (unsigned)fa->fa_off, (unsigned)fa->fa_size,
	       (unsigned)fa->fa_size >= 4096 ? 4096U : (unsigned)fa->fa_size);

	ret = flash_area_erase(fa, TEST_OFF, 4096);
	if (ret) {
		printk("erase %d\n", ret);
		return 0;
	}
	printk("erase OK\n");

	for (i = 0; i < TEST_LEN; i++) {
		wr[i] = (uint8_t)(0xA0 + i);
	}

	ret = flash_area_write(fa, TEST_OFF, wr, TEST_LEN);
	if (ret) {
		printk("write %d\n", ret);
		return 0;
	}
	printk("write OK\n");

	memset(rd, 0, sizeof(rd));
	ret = flash_area_read(fa, TEST_OFF, rd, TEST_LEN);
	if (ret) {
		printk("read %d\n", ret);
		return 0;
	}

	if (memcmp(wr, rd, TEST_LEN) != 0) {
		printk("FLASH_FAIL mismatch\n");
		for (i = 0; i < TEST_LEN; i++) {
			printk("  [%d] wr=%02x rd=%02x\n", i, wr[i], rd[i]);
		}
		return 0;
	}

	printk("FLASH_OK\n");
	printk("storage R/W OK (XPI NOR)\n");
	return 0;
}
