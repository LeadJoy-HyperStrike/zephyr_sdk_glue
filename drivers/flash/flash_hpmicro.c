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

/* Drop any D-cache lines covering [offset, offset + size) of the XIP window
 * after the ROM programmed or erased it, so a later read through the window
 * (see flash_hpmicro_read) cannot serve the old bytes. */
static void flash_hpm_dc_drop(off_t offset, size_t size)
{
    uint32_t addr = (uint32_t)CONFIG_FLASH_BASE_ADDRESS + (uint32_t)offset;
    uint32_t start = addr & ~(HPM_L1C_CACHELINE_SIZE - 1u);
    uint32_t end = (addr + (uint32_t)size + HPM_L1C_CACHELINE_SIZE - 1u) &
                   ~(HPM_L1C_CACHELINE_SIZE - 1u);

    if (l1c_dc_is_enabled() && end > start) {
        l1c_dc_invalidate(start, end - start);
    }
}
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
 * IP-command read (physical offsets, bypasses EXIP). The ROM's read wants a
 * word-aligned destination and works in whole words; callers of flash_read()
 * do not promise either (NVS reads ATEs and payloads at arbitrary offsets and
 * lengths), so anything not word-aligned on both ends goes through a small
 * aligned bounce buffer. Aligned callers (slot_state, hs2img) take the direct
 * path. Each ROM call runs under irq_lock like program/erase do.
 */
static int flash_hpmicro_ip_read(const struct device *dev, off_t offset, void *data, size_t size)
{
    struct flash_hpmicro_dev_data *const dev_data = dev->data;
    uint8_t *out = data;
    uint32_t bounce[64];
    hpm_stat_t status;
    unsigned int key;

    if ((((uintptr_t)data) & 3u) == 0u && ((uint32_t)offset & 3u) == 0u && (size & 3u) == 0u) {
        key = irq_lock();
        status = rom_xpi_nor_read(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                  (uint32_t *)data, (uint32_t)offset, (uint32_t)size);
        irq_unlock(key);
        return status == status_success ? 0 : -EIO;
    }
    while (size > 0u) {
        uint32_t start = (uint32_t)offset & ~3u;
        uint32_t skip = (uint32_t)offset - start;
        uint32_t take = (uint32_t)size;

        if (take > sizeof(bounce) - skip) {
            take = sizeof(bounce) - skip;
        }
        key = irq_lock();
        status = rom_xpi_nor_read(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                  bounce, start, (skip + take + 3u) & ~3u);
        irq_unlock(key);
        if (status != status_success) {
            return -EIO;
        }
        memcpy(out, (const uint8_t *)bounce + skip, take);
        out += take;
        offset += take;
        size -= take;
    }
    return 0;
}

static int flash_hpmicro_read(const struct device *dev, off_t offset,
                void *data,
                size_t size)
{
    /*
     * Read through the memory-mapped XIP window with a plain memcpy, the way
     * HPM's own eeprom_emulation port does (components/eeprom_emulation/port/
     * hpm_nor_flash.c) -- not through the ROM's IP-mode xpi_nor_read().
     *
     * Context (hs2prod bench, 2026-08-27): during the OTA investigation the
     * ROM read returned wrong data / a garbage status / hung right after a
     * swap, and "invalidate then memcpy" read transiently wrong bytes, while
     * a plain memcpy was right in every probe run. The root cause of that day
     * turned out to be the bench USB supply collapsing under the swap's
     * sustained erase load (VPMC POR; see backlog #31) -- those read paths
     * were the first victims of a sagging rail, not broken in themselves.
     * This implementation is kept because it matches HPM's own practice, has
     * no ROM/XPI state dependence, degraded most gracefully on a marginal
     * rail, and is the configuration the final passing OTA rounds validated.
     *
     * Coherence with this driver's own erase/program comes from those paths:
     * they drop the D-cache lines they touched once the ROM call returns.
     *
     * One exception (2026-09-02, ROM-native dual image): when the BootROM
     * booted the second image it enabled XPI address remapping, and the
     * window at the flash base then shows the *other* slot's bytes -- and
     * through EXIP, decrypted ones. A window memcpy is only right for the
     * running image, so with remap on we go through the ROM's IP-command
     * read, which addresses physical flash offsets and bypasses EXIP: raw
     * bytes, which is what a readback check against a catalog sha256 wants.
     * The IP read's destination is a uint32_t *; callers on that path
     * (hs2img_mgmt, slot_state) declare their buffers 4-byte aligned.
     */
    struct flash_hpmicro_dev_data *const dev_data = dev->data;
    const uint8_t *src = (const uint8_t *)((uint32_t)CONFIG_FLASH_BASE_ADDRESS +
                                           (uint32_t)offset);

    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    if (size == 0u) {
        return 0;
    }
    if (rom_xpi_nor_is_remap_enabled(dev_data->controller)) {
        return flash_hpmicro_ip_read(dev, offset, data, size);
    }
    memcpy(data, src, size);

    return 0;
}

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
    flash_hpm_dc_drop(offset, size);
    irq_unlock(key);
    return HPM_STATUS_ZEPHYR_RET(status);
}

static int flash_hpmicro_erase(const struct device *dev, off_t offset,
                 size_t size)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
    hpm_stat_t status = 0;
	unsigned int key;
    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    if (size < 4) {
        while (1) {
        }
    }
    key = irq_lock();
    for (int i = 0; i < size; i += (s_xpi_nor_config.device_info.sector_size_kbytes * 1024)) {
        status = rom_xpi_nor_erase_sector(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                   offset + i);
        if (status != status_success) {
            break;
        }
    }
    flash_hpm_dc_drop(offset, size);
    irq_unlock(key);
    return HPM_STATUS_ZEPHYR_RET(status);
}

#if CONFIG_FLASH_PAGE_LAYOUT
/* One uniform layout for the whole part: every sector of the NOR erases as one
 * page of erase-block-size. It used to be a chain of per-partition entries with
 * the same page size, which tied the driver to the MCUboot partition labels
 * (boot/scratch) and broke the build the moment a board dropped them. */
static const struct flash_pages_layout flash_hpm_pages_layout[] = {
    {
        .pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
        .pages_size = FLASH_ERASE_BLK_SZ,
    },
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