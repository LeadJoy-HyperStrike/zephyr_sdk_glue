# HPM5100EVK Flash (XPI NOR) 适配说明

**状态：通过**（`storage` 分区 erase/write/read）

## 硬件

| 项 | 值 |
|----|-----|
| 控制器 | **XPI0** `@0xF00E0000` |
| Flash | 板载 NOR，映射 `0x80000000`，1MB |
| Chosen | `zephyr,flash=&flash0`，`zephyr,flash-controller=&xpi0` |

## 分区（MCUboot 风格）

| Label | Offset | Size |
|-------|--------|------|
| mcuboot | `0x3000` | 128KB |
| image-0 | `0x23000` | 384KB |
| image-1 | `0x83000` | 384KB |
| image-scratch | `0xE3000` | 64KB |
| **storage** | `0xF3000` | **52KB** (`0xD000`，到 1MB 末尾；勿写成 64KB 会越界) |

冒烟只操作 **storage**，不碰 XIP 代码区。

## 驱动要点

`sdk_glue/drivers/flash/flash_hpmicro.c` + `Kconfig.hpmicro`：

- ROM API：`rom_xpi_nor_auto_config` / `read` / `program` / `erase_sector`
- **必须** `select FLASH_HAS_EXPLICIT_ERASE`：否则 `flash_area_flatten()`（mcumgr `img_mgmt`）返回 `-ENOSYS` → `FLASH_ERASE_FAILED`
- erase：按 `sector_size` 对齐，逐扇区 `irq_lock`，失败返回 `-EIO`
- `write-block-size` 参数报告为 4

## 验证

```text
west build -b hpm5100evk sdk_glue/samples/drivers/flash_storage -d build_hpm5100evk_flash
west flash -d build_hpm5100evk_flash
```

期望：

```text
Flash storage smoke
erase OK
write OK
FLASH_OK
storage R/W OK (XPI NOR)
```
