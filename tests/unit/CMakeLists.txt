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
# These suites moved here from alp-sdk when the firmware split into its own
# repository (alp-sdk ADR 0031).  They link the REAL firmware sources -- not
# mocks -- so the bounds checks and CS-framing seams under test are the same
# code paths the device runs.
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

# --- adc_seq: GD32 ADC register-sequence gate -------------------------------
#
# #77 means no CI job compiles hal/gd32/*.c -- the stub build above links
# only hal/bridge_hw_stub.c.  This target links the REAL hal/gd32/adc.c +
# hal/gd32/adc_stream.c, unmodified, against a hand-written mock of the GD32
# vendor header (adc_seq/mock/gd32g5x3.h) so the two register sequences
# PR #80's review asked for by name -- ROVF cleared before adc_enable, and
# calibration run after every adc_enable on the request path -- are pinned
# by a real assertion instead of a diff read.  See adc_seq/mock/gd32g5x3.h's
# top comment for exactly what the mock does and does not model.
add_executable(test_adc_seq
    adc_seq/src/test_adc_seq.c
    adc_seq/mock/mock_gd32.c
    ${FW}/hal/gd32/adc.c
    ${FW}/hal/gd32/adc_stream.c
)
target_include_directories(test_adc_seq PRIVATE
    ${FW}/hal
    ${FW}/hal/gd32
    ${CMAKE_CURRENT_SOURCE_DIR}/adc_seq/mock
)
target_compile_options(test_adc_seq PRIVATE ${TEST_FLAGS})
# adc_stream.c's FFT-window math (__builtin_cosf/sqrtf) lowers to real libm
# calls on a host compiler with no -ffreestanding; the M33 build never needs
# this because CMSIS-DSP/arm-none-eabi's libm supplies them implicitly.
target_link_libraries(test_adc_seq PRIVATE ztest_shim m)
add_test(NAME adc_seq COMMAND test_adc_seq)
