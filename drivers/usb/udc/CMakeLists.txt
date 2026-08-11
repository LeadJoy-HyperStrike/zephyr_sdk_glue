# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

if(CONFIG_UDC_HPMICRO)
  set(HPM_UDC_USB_DEVICE_DIR "${HPM_SDK_DIR}/components/usb/device")
  set(HPM_UDC_EXTERNAL_5100_DIR
      "${HPM_SDK_DIR}/../../sdk_env_v1.12.0_hpm5100_er_2026.06.30/hpm_sdk/components/usb/device")

  if(EXISTS "${HPM_UDC_EXTERNAL_5100_DIR}/hpm_usb_device.c")
    set(HPM_UDC_USB_DEVICE_DIR "${HPM_UDC_EXTERNAL_5100_DIR}")
  endif()

  # HPM5100 HRAM is 32KB. Default 8 QTDs/EP * 16 EP * 2 dir ≈ 8KB QTDs.
  # CDC only needs short control/bulk chains; 2 QTDs/EP frees ~6KB for HS + stacks.
  zephyr_compile_definitions(USB_SOC_DCD_QTD_COUNT_EACH_ENDPOINT=2)

  zephyr_include_directories(${HPM_UDC_USB_DEVICE_DIR})
  zephyr_library_sources(${HPM_UDC_USB_DEVICE_DIR}/hpm_usb_device.c)
endif()

zephyr_library_sources_ifdef(CONFIG_UDC_HPMICRO udc_hpmicro.c)
