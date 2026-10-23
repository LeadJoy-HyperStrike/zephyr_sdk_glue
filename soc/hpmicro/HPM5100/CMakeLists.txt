# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_sources(
  start.S
  soc.c
)

# The vendor file overrides arch_cpu_idle even with CONFIG_PM=n. Respect
# the explicit no-clock-gating bring-up/real-time configuration instead
# of silently enabling Wait mode (and touching WKUP state) in that case.
if(CONFIG_PM OR CONFIG_POWEROFF OR CONFIG_SOC_HPM5100_CPU_IDLE_WAIT)
  zephyr_sources(power.c)
endif()

if(CONFIG_SOC_ANDES_V5_HWDSP)
  zephyr_cc_option(-mext-dsp)
endif()

if(CONFIG_SOC_SERIES_HPM5100)
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
endif()
