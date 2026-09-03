# SPDX-License-Identifier: Apache-2.0
#
# CI-only copy of alp-sdk's vendors/gd32_firmware_library/CMakeLists.txt
# wrapper.  This file is Apache-2.0 alp-sdk glue code, NOT GigaDevice IP --
# the licensed vendor sources it points at (SLA-GD0001 v1.1) are fetched
# separately by .github/workflows/ci.yml from the pinned public mirror
# https://github.com/alplabai/gd32g5x3-firmware-library, never committed
# here.
#
# The `gd32 backend build` CI job stages this file at
# <2 levels above the checkout>/vendors/gd32_firmware_library/CMakeLists.txt
# so CMakeLists.txt:93's hardcoded fallback path
# (../../vendors/gd32_firmware_library, the pre-split alp-sdk layout) finds
# it with no CMakeLists.txt change and no dependency on PR #67's
# GD32_VENDOR_DIR override.  Keep this in sync by hand with alp-sdk's copy
# if that wrapper changes (glob pattern, include dirs, compile options);
# there is no automation tying the two today.
#
# Not used by any non-CI build -- a local `-DBRIDGE_HAL_BACKEND=gd32` build
# still resolves the real alp-sdk checkout (or -DGD32_VENDOR_DIR once #67
# lands).

cmake_minimum_required(VERSION 3.20)

set(_GD32_UPSTREAM ${CMAKE_CURRENT_SOURCE_DIR}/upstream)
set(_GD32_FW_ROOT  ${_GD32_UPSTREAM}/Firmware)
set(_GD32_CMSIS    ${_GD32_FW_ROOT}/CMSIS)
set(_GD32_STDPER   ${_GD32_FW_ROOT}/GD32G5x3_standard_peripheral)
set(_GD32_DEVICE_H ${_GD32_CMSIS}/GD/GD32G5x3/Include/gd32g5x3.h)

if(NOT EXISTS ${_GD32_DEVICE_H})
    message(FATAL_ERROR
        "GD32 firmware library not found.\n"
        "Expected gd32g5x3.h under:\n"
        "  ${_GD32_CMSIS}/GD/GD32G5x3/Include/\n"
        "(CI stages this from https://github.com/alplabai/gd32g5x3-firmware-library)")
endif()

message(STATUS "vendors/gd32_firmware_library/upstream: found GD32G5x3 firmware library "
               "(${_GD32_DEVICE_H})")

# Collect every peripheral source the standard-peripheral library
# publishes.  Mirrors alp-sdk's glob -- see that repo's
# vendors/gd32_firmware_library/CMakeLists.txt for the authoritative copy.
file(GLOB _GD32_STDPER_SOURCES
    ${_GD32_STDPER}/Source/gd32g5x3_*.c)

# No E1M-V2M101 HXTAL clock override here (alp-sdk's overrides/system_gd32g5x3.c):
# this job compiles and links only, never runs on silicon, so the stock
# vendor SystemInit() is fine.
set(_GD32_SYSTEM_SOURCES
    ${_GD32_CMSIS}/GD/GD32G5x3/Source/system_gd32g5x3.c
    ${_GD32_CMSIS}/GD/GD32G5x3/Source/GCC/startup_gd32g5x3.S)

add_library(gd32_firmware_library STATIC
    ${_GD32_SYSTEM_SOURCES}
    ${_GD32_STDPER_SOURCES})

target_include_directories(gd32_firmware_library PUBLIC
    ${_GD32_CMSIS}
    ${_GD32_CMSIS}/GD/GD32G5x3/Include
    ${_GD32_STDPER}/Include)

target_compile_options(gd32_firmware_library PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -Wno-unused-parameter
    -Wno-misleading-indentation)
