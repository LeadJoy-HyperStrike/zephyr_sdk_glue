/*
 * Copyright (c) 2023-2025 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#define DT_DRV_COMPAT hpmicro_xpi
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_NOR_CFG_OPT_HDR DT_PROP(SOC_NV_FLASH_NODE, nor_cfg_opt_hdr)
#define FLASH_NOR_CFG_OPT_OPT0 DT_PROP(SOC_NV_FLASH_NODE, nor_cfg_opt_opt0)
#define FLASH_NOR_CFG_OPT_OPT1 DT_PROP(SOC_NV_FLASH_NODE, nor_cfg_opt_opt1)

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include "hpm_romapi.h"
#include "hpm_l1c_drv.h"
#ifdef ARRAY_SIZE
#undef ARRAY_SIZE
#endif
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
LOG_MODULE_REGISTER(flash_hpmicro, CONFIG_FLASH_LOG_LEVEL);

#define HPM_STATUS_ZEPHYR_RET(x)    (x)

static xpi_nor_config_t s_xpi_nor_config;

static uint32_t flash_size;
static uint32_t sector_size;
static uint32_t page_size;
static uint32_t block_size;

struct flash_hpmicro_dev_config {
	void *controller;
};
struct flash_hpmicro_dev_data {
	XPI_Type *controller;
};

static const struct flash_parameters flash_hpmicro_parameters = {
    .write_block_size = 4,
    .erase_value = 0xff,
};

static int flash_hpmicro_init(const struct device *dev);
static bool initted = false;

/*
 * XIP hazard, and why every entry point below is ATTR_RAMFUNC (.fast -> ILM).
 *
 * This driver programs the same NOR the code is executing from. The ROM API
 * itself lives in BootROM, so the erase/program primitives are safe -- but the
 * loop AROUND them was not: after each rom_xpi_nor_erase_sector() returned,
 * the next instruction of this loop had to be fetched from a NOR that had just
 * been busy, through an XPI whose AHB read path the erase had disturbed. In
 * the application that mostly worked because the loop body sat in I-cache. In
 * MCUboot (375 MHz, cold cache, first flash op at boot) it hung: on hs2prod,
 * 2026-08-27, both a test swap and its revert stopped dead after "Starting
 * swap using scratch algorithm." with no further output, no assert, no fault
 * -- and never came back until reset.
 *
 * HPM's own wrappers (components/eeprom_emulation/port/hpm_nor_flash.c,
 * samples/tinyuf2/src/board_api.c) mark every such function ATTR_RAMFUNC and
 * invalidate D-cache over the touched range afterwards; the flashstress
 * sample goes further and refuses to run "on flash_xip build" at all. This
 * file follows the wrappers. The linker script already collects .fast into
 * the ITCM output section whenever the board provides zephyr,itcm (hs2prod
 * and the EVK both do) -- the section was simply empty because nothing here
 * asked to go there.
 *
 * The D-cache invalidate matters for a different reason: rom_xpi_nor_read()
 * bypasses the cache, but XIP code and any memcpy from the flash window do
 * not, and a line cached before an erase would keep serving the old bytes.
 */
#define FLASH_HPM_CACHELINE 64u

ATTR_RAMFUNC
static void flash_hpm_invalidate(off_t offset, size_t size)
{
    uint32_t start = ((uint32_t)CONFIG_FLASH_BASE_ADDRESS + (uint32_t)offset) &
                     ~(FLASH_HPM_CACHELINE - 1u);
    uint32_t end = ((uint32_t)CONFIG_FLASH_BASE_ADDRESS + (uint32_t)offset +
                    (uint32_t)size + FLASH_HPM_CACHELINE - 1u) &
                   ~(FLASH_HPM_CACHELINE - 1u);

    if (l1c_dc_is_enabled()) {
        l1c_dc_invalidate(start, end - start);
    }
}

/*
 * rom_xpi_nor_read() is a static inline with a chunking loop; gcc outlines it
 * into a plain-.text `rom_xpi_nor_read.constprop.0` that does NOT inherit the
 * caller's .fast placement, which put the read path back on XIP flash. Call
 * the ROM table directly from here instead, keeping the 32 KiB chunk rule the
 * SDK wrapper enforces.
 */
ATTR_RAMFUNC
static hpm_stat_t flash_hpm_rom_read(XPI_Type *base, uint32_t *dst,
                                     uint32_t start, uint32_t length)
{
    const uint32_t max_chunk = 32u * 1024u;
    uint8_t *p = (uint8_t *)dst;
    hpm_stat_t st = status_success;

    while (length > 0u) {
        uint32_t chunk = length > max_chunk ? max_chunk : length;

        st = ROM_API_TABLE_ROOT->xpi_nor_driver_if->read(base, xpi_xfer_channel_auto,
                                                         &s_xpi_nor_config,
                                                         (uint32_t *)p, start, chunk);
        if (st != status_success) {
            break;
        }
        p += chunk;
        start += chunk;
        length -= chunk;
    }
    return st;
}

ATTR_RAMFUNC
static int flash_hpmicro_read(const struct device *dev, off_t offset,
                void *data,
                size_t size)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
    hpm_stat_t status = 0;
	unsigned int key;
    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    key = irq_lock();
    if (size < 4) {
        uint32_t temp;
        status = flash_hpm_rom_read(dev_data->controller, &temp, offset, 4);
        memcpy(data, &temp, size);
    } else {
        status = flash_hpm_rom_read(dev_data->controller, data, offset, size);
    }
    irq_unlock(key);

    return HPM_STATUS_ZEPHYR_RET(status);
}

ATTR_RAMFUNC
static int flash_hpmicro_write(const struct device *dev, off_t offset,
                 const void *data, size_t size)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
    hpm_stat_t status = 0;
	unsigned int key;
    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    key = irq_lock();
    status = rom_xpi_nor_program(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                        data, offset, size);
    flash_hpm_invalidate(offset, size);
    irq_unlock(key);
    return HPM_STATUS_ZEPHYR_RET(status);
}

ATTR_RAMFUNC
static int flash_hpmicro_erase(const struct device *dev, off_t offset,
                 size_t size)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
    hpm_stat_t status = 0;
	unsigned int key;
    uint32_t sector;
    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    /* Used to be `while (1) {}`. A caller error is not a reason to hang the
     * part silently; report it like every other driver does. */
    if (size < 4) {
        return -EINVAL;
    }
    sector = s_xpi_nor_config.device_info.sector_size_kbytes * 1024u;
    if (sector == 0u) {
        return -EIO;
    }
    key = irq_lock();
    for (size_t i = 0; i < size; i += sector) {
        status = rom_xpi_nor_erase_sector(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                   offset + i);
        if (status != status_success) {
            break;
        }
    }
    flash_hpm_invalidate(offset, size);
    irq_unlock(key);
    return HPM_STATUS_ZEPHYR_RET(status);
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_hpm_pages_layout[] = {
    {
        .pages_count = FIXED_PARTITION_OFFSET(boot_partition) / KB(4),
        .pages_size = KB(4),
    },
    {
        .pages_count = FIXED_PARTITION_SIZE(boot_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = FIXED_PARTITION_SIZE(slot0_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = FIXED_PARTITION_SIZE(slot1_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = FIXED_PARTITION_SIZE(scratch_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = FIXED_PARTITION_SIZE(storage_partition) / KB(4),
        .pages_size = KB(4)
    }
};

void flash_hpmicro_page_layout(const struct device *dev,
                 const struct flash_pages_layout **layout,
                 size_t *layout_size)
{
    *layout = flash_hpm_pages_layout;
    *layout_size = ARRAY_SIZE(flash_hpm_pages_layout);
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_hpmicro_get_parameters(const struct device *dev)
{
    return &flash_hpmicro_parameters;
}

ATTR_RAMFUNC
static int flash_hpmicro_init(const struct device *dev)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
	unsigned int key;

    xpi_nor_config_option_t option;
    option.header.U = FLASH_NOR_CFG_OPT_HDR;
    option.option0.U = FLASH_NOR_CFG_OPT_OPT0;
    option.option1.U = FLASH_NOR_CFG_OPT_OPT1;

    key = irq_lock();
    hpm_stat_t status = rom_xpi_nor_auto_config(dev_data->controller, &s_xpi_nor_config, &option);
    if (status != status_success) {
        irq_unlock(key);
        return status;
    }

    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_total_size,
                             &flash_size);
    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_sector_size,
                             &sector_size);
    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_block_size,
                             &block_size);
    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_page_size, &page_size);
    irq_unlock(key);

    initted = true;
    return 0;
}

static const struct flash_driver_api flash_hpmicro_driver_api = {
    .read = flash_hpmicro_read,
    .write = flash_hpmicro_write,
    .erase = flash_hpmicro_erase,
    .get_parameters = flash_hpmicro_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
    .page_layout = flash_hpmicro_page_layout,
#endif
};

static struct flash_hpmicro_dev_data flash_hpmicro_data = {
	.controller = (XPI_Type *)DT_INST_REG_ADDR(0),
};

static const struct flash_hpmicro_dev_config flash_hpmicro_config = {
	.controller = (XPI_Type *)DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, flash_hpmicro_init,
		      NULL,
		      &flash_hpmicro_data, &flash_hpmicro_config,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_hpmicro_driver_api);