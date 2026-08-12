# SPDX-License-Identifier: Apache-2.0

set(OPENOCD_CONFIG_RELATIVE ${ZEPHYR_BASE}/../sdk_env/hpm_sdk/boards/openocd)
set(HPM_TOOLS_RELATIVE ${ZEPHYR_BASE}/../sdk_env/tools)
get_filename_component(OPENOCD_CONFIG_ABSOLUTE ${OPENOCD_CONFIG_RELATIVE} ABSOLUTE)
get_filename_component(HPM_TOOLS_ABSOLUTE ${HPM_TOOLS_RELATIVE} ABSOLUTE)
set(OPENOCD_CONFIG_DIR ${OPENOCD_CONFIG_ABSOLUTE} CACHE PATH "hpmicro openocd cfg root directory")
set(HPM_TOOLS_DIR ${HPM_TOOLS_ABSOLUTE} CACHE PATH "hpmicro win tools root directory")
set(HPM_OPENOCD_PROBE "cmsis_dap" CACHE STRING "openocd probe cfg name without .cfg")
set(HPM_OPENOCD_CMSIS_DAP_SERIAL "" CACHE STRING "optional cmsis-dap serial")
set(HPM_OPENOCD_BIN "${HPM_TOOLS_DIR}/openocd/openocd.exe" CACHE FILEPATH "openocd executable path")

if(NOT CONFIG_XIP)
    board_runner_args(openocd "--use-elf")
endif()

set(ENV_PATH $ENV{PATH})
unset(OPENOCD CACHE)

if(${BOARD} STREQUAL "hpm5100evk")
    board_runner_args(openocd "--config=${OPENOCD_CONFIG_DIR}/probes/${HPM_OPENOCD_PROBE}.cfg"
                                    "--config=${OPENOCD_CONFIG_DIR}/soc/hpm5100.cfg"
                                "--config=${OPENOCD_CONFIG_DIR}/boards/hpm5100evk.cfg"
                                "--cmd-pre-init=adapter speed 2000"
                                "--openocd-search=${OPENOCD_CONFIG_DIR}")
    if(HPM_OPENOCD_CMSIS_DAP_SERIAL)
        board_runner_args(openocd "--cmd-pre-init=cmsis_dap_serial ${HPM_OPENOCD_CMSIS_DAP_SERIAL}")
    endif()
    board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)
else()
    message(FATAL_ERROR "${BOARD} is not supported now")
endif()

if("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    set(SEPARATOR ":")
    string(REPLACE ${SEPARATOR} ";" ZPATH ${ENV_PATH})
    find_program(OPENOCD openocd PATHS ${ZPATH} NO_DEFAULT_PATH)
elseif("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Windows")
    set(OPENOCD "${HPM_OPENOCD_BIN}" CACHE FILEPATH "" FORCE)
    get_filename_component(HPM_OPENOCD_DIR "${HPM_OPENOCD_BIN}" DIRECTORY)
    set(OPENOCD_DEFAULT_PATH "${HPM_OPENOCD_DIR}/tcl")
else()
    message(WARNING "${CMAKE_HOST_SYSTEM_NAME} openocd is not support")
endif()
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
