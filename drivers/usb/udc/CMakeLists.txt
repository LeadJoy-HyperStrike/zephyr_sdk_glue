# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

if(CONFIG_UDC_HPMICRO)
  set(HPM_UDC_USB_DEVICE_DIR "${HPM_SDK_DIR}/components/usb/device")

  if(CONFIG_SOC_SERIES_HPM5100)
    # HPM5100 only, and it MUST stay gated: the -D overrides the #ifndef
    # default (8) in every SoC's hpm_soc_feature.h, so an ungated one would
    # shrink the HPM6E80 QTD pool from 256 to 64 in both the app and the
    # MCUboot recovery image.
    # HPM5100 HRAM is 32KB. Default 8 QTDs/EP * 16 EP * 2 dir ≈ 8KB QTDs.
    # CDC only needs short control/bulk chains; 2 QTDs/EP frees ~6KB.
    #
    # The vendor drop also redirected hpm_usb_device.c to a side-by-side
    # sdk_env_v1.12.0_hpm5100_er_* tree; sdk_env is v1.12.1 now, so
    # HPM_SDK_DIR already carries that exact file and the redirect is gone.
    zephyr_compile_definitions(USB_SOC_DCD_QTD_COUNT_EACH_ENDPOINT=2)
  endif()

  zephyr_include_directories(${HPM_UDC_USB_DEVICE_DIR})
  zephyr_library_sources(${HPM_UDC_USB_DEVICE_DIR}/hpm_usb_device.c)
endif()

zephyr_library_sources_ifdef(CONFIG_UDC_HPMICRO udc_hpmicro.c)
