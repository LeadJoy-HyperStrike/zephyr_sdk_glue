# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

if(CONFIG_UDC_HPMICRO)
  set(HPM_UDC_USB_DEVICE_DIR "${HPM_SDK_DIR}/components/usb/device")

  # HPM5100 HRAM is 32KB. Default 8 QTDs/EP * 16 EP * 2 dir ≈ 8KB QTDs.
  # CDC only needs short control/bulk chains; 2 QTDs/EP frees ~6KB for HS + stacks.
  zephyr_compile_definitions(USB_SOC_DCD_QTD_COUNT_EACH_ENDPOINT=2)

  zephyr_include_directories(${HPM_UDC_USB_DEVICE_DIR})
  zephyr_library_sources(${HPM_UDC_USB_DEVICE_DIR}/hpm_usb_device.c)
endif()

zephyr_library_sources_ifdef(CONFIG_UDC_HPMICRO udc_hpmicro.c)
