# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_COUNTER_HPM_GPTMR counter_gptmr_hpmicro.c)
