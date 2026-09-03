# SPDX-License-Identifier: Apache-2.0
#
# HOST unit tests for the bridge firmware.  Configured SEPARATELY from the
# top-level firmware build, which cross-compiles for the Cortex-M33 and must
# not have a host toolchain spliced into it:
#
#   cmake -S tests/unit -B build-tests
#   cmake --build build-tests
#   ctest --test-dir build-tests --output-on-failure
#
# The `ota` and `transport_spi` suites moved here from alp-sdk when the firmware
# split into its own repository (alp-sdk ADR 0031).  They, and `transport_i2c`,
# link the REAL firmware sources against hal/bridge_hw_stub.c -- not mocks -- so
# the bounds checks and the CS/register framing seams under test are the same
# code paths the device runs.
#
# `protocol` is the deliberate exception: it links tests/unit/fake/bridge_hw_fake.c
# INSTEAD of the stub.  The stub answers BRIDGE_HW_ERR_NOTIMPL for nearly
# everything, so only about two hardware opcodes can reach STATUS_OK through it,
# which left src/protocol.c's reply encoding and its BRIDGE_HW_ERR_* -> STATUS_*
# ladders unreachable from any host test.  The fake is injectable (a forced return
# code per function) and resettable, trading the real HAL for reachability.  It is
# test-only and lives under tests/unit/ so no firmware build can pick it up.
cmake_minimum_required(VERSION 3.20)
project(gd32-bridge-unit-tests LANGUAGES C)

enable_testing()

set(FW ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# The suites use `#include <zephyr/ztest.h>`; tests/unit/zephyr/ztest.h
# satisfies it and forwards to the shim, which keeps the suite sources
# byte-identical to the originals.
add_library(ztest_shim STATIC ztest_shim.c)
target_include_directories(ztest_shim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# The shim registers cases with __attribute__((constructor)), which MSVC does
# not implement -- so this build is GCC/Clang only.  Say that outright rather
# than letting cl.exe fail later on an unrecognised /Wextra.
if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR
        "The unit tests need GCC or Clang (got '${CMAKE_C_COMPILER_ID}'): the "
        "ztest shim registers cases via __attribute__((constructor)).
"
        "On Windows, configure with -DCMAKE_C_COMPILER=gcc or run them in WSL.")
endif()

# Warnings are errors here: these tests exist to catch bounds and framing bugs,
# and a silent -Wconversion in the firmware sources is exactly that class.
set(TEST_FLAGS -std=c11 -Wall -Wextra -Werror)

# --- ota: OTA state machine, partitioned path armed ------------------------
#
# The flash read/program seams are redirected into a host buffer by the weak
# ota_fmc_* overrides inside the test source, so no raw 32-bit flash address is
# ever dereferenced on a 64-bit host.
add_executable(test_ota
    ota/src/test_ota.c
    ${FW}/src/ota.c
    ${FW}/src/crc32.c
)
target_include_directories(test_ota PRIVATE ${FW}/src ${FW}/hal)
target_compile_definitions(test_ota PRIVATE BRIDGE_OTA_PARTITIONED)
target_compile_options(test_ota PRIVATE ${TEST_FLAGS})
target_link_libraries(test_ota PRIVATE ztest_shim)
add_test(NAME ota COMMAND test_ota)

# --- transport_spi: transport + dispatcher over the stub HAL ---------------
add_executable(test_transport_spi
    transport_spi/src/test_transport_spi.c
    ${FW}/src/transport_spi.c
    ${FW}/src/protocol.c
    ${FW}/src/bootloader/bootloader.c
    ${FW}/src/ota.c
    ${FW}/src/crc32.c
    ${FW}/hal/bridge_hw_stub.c
)
target_include_directories(test_transport_spi PRIVATE ${FW}/src ${FW}/hal)
target_compile_options(test_transport_spi PRIVATE ${TEST_FLAGS})
target_link_libraries(test_transport_spi PRIVATE ztest_shim)
add_test(NAME transport_spi COMMAND test_transport_spi)

# --- transport_i2c: transport + dispatcher over the stub HAL ---------------
add_executable(test_transport_i2c
    transport_i2c/src/test_transport_i2c.c
    ${FW}/src/transport_i2c.c
    ${FW}/src/protocol.c
    ${FW}/src/bootloader/bootloader.c
    ${FW}/src/ota.c
    ${FW}/src/crc32.c
    ${FW}/hal/bridge_hw_stub.c
)
target_include_directories(test_transport_i2c PRIVATE ${FW}/src ${FW}/hal)
target_compile_options(test_transport_i2c PRIVATE ${TEST_FLAGS})
target_link_libraries(test_transport_i2c PRIVATE ztest_shim)
add_test(NAME transport_i2c COMMAND test_transport_i2c)

# --- protocol: dispatcher over a TEST-ONLY injectable fake HAL -------------
#
# hal/bridge_hw_stub.c answers BRIDGE_HW_ERR_NOTIMPL for essentially every
# call, so it can only ever prove STATUS_OK on the opcodes with no hardware
# dependency (PING/GET_VERSION/GET_BUILD_ID/LINK_FEATURES) -- the rest of
# src/protocol.c's 1025 lines goes untouched by transport_spi/transport_i2c.
# fake/bridge_hw_fake.c stands in for hal/bridge_hw_stub.c here instead, so
# every opcode's success path AND its BRIDGE_HW_ERR_* -> STATUS_* mapping is
# reachable.  Not -DBRIDGE_OTA_PARTITIONED (matching transport_spi/
# transport_i2c, not the standalone `ota` target above): the 0xF0..0xFF OTA
# range is exercised here only as the inert, unpartitioned default.
add_executable(test_protocol
    protocol/src/test_protocol.c
    fake/bridge_hw_fake.c
    ${FW}/src/protocol.c
    ${FW}/src/bootloader/bootloader.c
    ${FW}/src/ota.c
    ${FW}/src/crc32.c
)
target_include_directories(test_protocol PRIVATE ${FW}/src ${FW}/hal ${CMAKE_CURRENT_SOURCE_DIR}/fake)
target_compile_options(test_protocol PRIVATE ${TEST_FLAGS})
target_link_libraries(test_protocol PRIVATE ztest_shim)
add_test(NAME protocol COMMAND test_protocol)
