# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Alp Lab AB
#
# Build wrapper for the GigaDevice GD32G5x3 firmware library.
#
# The library is GigaDevice's code under GigaDevice's own terms (see
# THIRD_PARTY_NOTICES.md).  It is NOT part of this repository: run
# tools/fetch_gd32_library.sh to clone it, at a pinned commit with a
# verified tree hash, into ./upstream (gitignored).  This file is only
# Alp-authored build glue.

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
        "Run:  tools/fetch_gd32_library.sh\n"
        "(clones https://github.com/GigaDevice-GD32-MCU/GD32G5x3_Firmware_Library "
        "at the pinned commit)")
endif()

file(GLOB _GD32_STDPER_SOURCES ${_GD32_STDPER}/Source/gd32g5x3_*.c)

# The stock system_gd32g5x3.c selects the HXTAL clock, which never starts on
# the E1M-V2M101 SoM, so SystemInit() hangs before main() (#127).  Apply our
# IRC8M patch to a COPY in the build tree; the fetched library stays pristine
# and no modified GigaDevice file is ever committed.  The copy lives under
# overrides/ because tools/check_clock_override.py verifies that path.
set(_GD32_SYSTEM_STOCK ${_GD32_CMSIS}/GD/GD32G5x3/Source/system_gd32g5x3.c)
set(_GD32_SYSTEM_PATCH ${CMAKE_CURRENT_SOURCE_DIR}/patches/system_gd32g5x3-irc8m.patch)
if(EXISTS ${_GD32_SYSTEM_PATCH})
    find_package(Git REQUIRED)
    set(_GD32_OVR_DIR ${CMAKE_CURRENT_BINARY_DIR}/overrides)
    file(MAKE_DIRECTORY ${_GD32_OVR_DIR})
    file(COPY ${_GD32_SYSTEM_STOCK} DESTINATION ${_GD32_OVR_DIR})
    # `git apply` works outside a repository and does not need GNU patch.
    execute_process(
        COMMAND ${GIT_EXECUTABLE} apply -p1 ${_GD32_SYSTEM_PATCH}
        WORKING_DIRECTORY ${_GD32_OVR_DIR}
        RESULT_VARIABLE _gd32_patch_rc
        ERROR_VARIABLE  _gd32_patch_err)
    if(NOT _gd32_patch_rc EQUAL 0)
        message(FATAL_ERROR
            "Could not apply ${_GD32_SYSTEM_PATCH} to the fetched library "
            "(version drift? re-run tools/fetch_gd32_library.sh):\n${_gd32_patch_err}")
    endif()
    set(_GD32_SYSTEM_C ${_GD32_OVR_DIR}/system_gd32g5x3.c)
    message(STATUS "gd32_firmware_library: IRC8M clock patch applied (216M-PLL-IRC8M)")
else()
    set(_GD32_SYSTEM_C ${_GD32_SYSTEM_STOCK})
endif()

add_library(gd32_firmware_library STATIC
    ${_GD32_SYSTEM_C}
    ${_GD32_CMSIS}/GD/GD32G5x3/Source/GCC/startup_gd32g5x3.S
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
