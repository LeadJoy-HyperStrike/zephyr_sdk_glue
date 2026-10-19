/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Distinguish images with -DEXTRA_CFLAGS="-DSWAP_SMOKE_LABEL=\\\"SLOT-B\\\""
 */

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifndef SWAP_SMOKE_LABEL
#define SWAP_SMOKE_LABEL "SLOT-A"
#endif

int main(void)
{
	int rc;

	printk("Hello %s! %s\n", SWAP_SMOKE_LABEL, CONFIG_BOARD_TARGET);

	if (boot_is_img_confirmed()) {
		printk("img already confirmed\n");
	} else {
		rc = boot_write_img_confirmed();
		printk("img confirm %s (%d)\n", rc ? "FAIL" : "OK", rc);
	}

	return 0;
}
