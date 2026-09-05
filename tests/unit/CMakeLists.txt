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
# `crc32` (#26 C2, #104) pins the table-driven ota_crc32() bit-identical to
# the bit-serial implementation it replaced, against a reference copy of the
# old algorithm kept local to the test file, plus the standard CRC-32 check
# value and the streaming/chunked seed contract src/ota.c's h_write depends
# on.
#
# `crc16` (#26, #104) is the same proof for src/protocol.c's table-driven
# crc16_ccitt_false() -- the FRAMING CRC, called twice per round trip.  It
# links protocol.c itself (not a standalone crc16.c/.h -- see the comment
# ahead of crc16_table[] in protocol.c for why) plus protocol.c's other
# dependencies, the same set transport_spi/transport_i2c below link.
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
#
# Built TWICE off the same source (#3): ota.c derives the compile-time
# OTA_RUNNING_SLOT from -DBRIDGE_APP_SLOT_BASE (see ota.c), and that
# derivation is only exercised for the value it is actually built with --
# an A-resident-only test build would never compile the OTA_SLOT_B_BASE
# arm of the #elif chain.
#
# The two bases below are PARSED out of src/ota_layout.h -- the same
# header ota.c includes -- rather than hand-copied, so this copy cannot
# drift silently against it.  Two other independent copies of the same
# numbers remain and are NOT derived here: the top-level CMakeLists.txt's
# gd32-bridge-slot-a / gd32-bridge-slot-b BRIDGE_APP_SLOT_BASE compile
# defines (:197,202) and APP_SLOT_BASE linker-script sets (:187,190) --
# out of scope for this fix, and that file is explicitly not to be edited
# here -- and OTA_SLOT_SIZE's separate copy in tools/gen_ota_metadata.py:34.
# A mismatch in either of those still fails loudly: ota.c's own #error
# chain rejects any BRIDGE_APP_SLOT_BASE that doesn't equal
# OTA_SLOT_A_BASE/OTA_SLOT_B_BASE from the header.
set(_OTA_LAYOUT_H ${FW}/src/ota_layout.h)
file(STRINGS ${_OTA_LAYOUT_H} _OTA_SLOT_A_BASE_LINE
     REGEX "^#define[ \t]+OTA_SLOT_A_BASE[ \t]+0x[0-9A-Fa-f]+u")
file(STRINGS ${_OTA_LAYOUT_H} _OTA_SLOT_B_BASE_LINE
     REGEX "^#define[ \t]+OTA_SLOT_B_BASE[ \t]+0x[0-9A-Fa-f]+u")
string(REGEX MATCH "0x[0-9A-Fa-f]+u" _OTA_SLOT_A_BASE "${_OTA_SLOT_A_BASE_LINE}")
string(REGEX MATCH "0x[0-9A-Fa-f]+u" _OTA_SLOT_B_BASE "${_OTA_SLOT_B_BASE_LINE}")
if(NOT _OTA_SLOT_A_BASE OR NOT _OTA_SLOT_B_BASE)
    message(FATAL_ERROR
        "tests/unit/CMakeLists.txt: could not parse OTA_SLOT_A_BASE / "
        "OTA_SLOT_B_BASE out of ${_OTA_LAYOUT_H} -- has the #define layout changed?")
endif()
foreach(_slot A B)
    string(TOLOWER "${_slot}" _slot_lc)
    set(_tgt test_ota_slot_${_slot_lc})
    add_executable(${_tgt}
        ota/src/test_ota.c
        ${FW}/src/ota.c
        ${FW}/src/crc32.c
    )
    target_include_directories(${_tgt} PRIVATE ${FW}/src ${FW}/hal)
    target_compile_definitions(${_tgt} PRIVATE
        BRIDGE_OTA_PARTITIONED BRIDGE_APP_SLOT_BASE=${_OTA_SLOT_${_slot}_BASE})
    target_compile_options(${_tgt} PRIVATE ${TEST_FLAGS})
    target_link_libraries(${_tgt} PRIVATE ztest_shim)
    add_test(NAME ota_slot_${_slot_lc} COMMAND ${_tgt})
endforeach()

# --- crc32: table-driven CRC-32 pinned bit-identical to bit-serial ---------
add_executable(test_crc32
    crc32/src/test_crc32.c
    ${FW}/src/crc32.c
)
target_include_directories(test_crc32 PRIVATE ${FW}/src ${FW}/hal)
target_compile_options(test_crc32 PRIVATE ${TEST_FLAGS})
target_link_libraries(test_crc32 PRIVATE ztest_shim)
add_test(NAME crc32 COMMAND test_crc32)

# --- crc16: table-driven framing CRC-16 pinned bit-identical to bit-serial -
add_executable(test_crc16
    crc16/src/test_crc16.c
    ${FW}/src/protocol.c
    ${FW}/src/bootloader/bootloader.c
    ${FW}/src/ota.c
    ${FW}/src/crc32.c
    ${FW}/hal/bridge_hw_stub.c
)
target_include_directories(test_crc16 PRIVATE ${FW}/src ${FW}/hal)
target_compile_options(test_crc16 PRIVATE ${TEST_FLAGS})
target_link_libraries(test_crc16 PRIVATE ztest_shim)
add_test(NAME crc16 COMMAND test_crc16)

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

# --- adc_dsp: chain pool + chain_bind capability validation (#69, #70) -----
#
# Links ONLY hal/gd32/adc_dsp_chain.c -- the chain pool + bind-time validator
# split out of hal/gd32/adc_stream.c specifically because it touches ZERO
# GD32 vendor symbols (no "gd32g5x3.h").  adc_stream.c itself is NOT linked
# here: it pulls the vendor header for the FAC/FFT register-level pump, which
# this suite cannot exercise on a host.  Instead this target supplies its own
# definition of adc_streams[] (test_adc_dsp.c) so cases can drive `in_use` /
# `dsp_bound` / `dsp_terminal` directly -- the same "fake state instead of a
# real HAL" trade the `protocol` suite above makes, for the same reason.
add_executable(test_adc_dsp
    adc_dsp/src/test_adc_dsp.c
    ${FW}/hal/gd32/adc_dsp_chain.c
)
target_include_directories(test_adc_dsp PRIVATE ${FW}/hal ${FW}/hal/gd32)
target_compile_options(test_adc_dsp PRIVATE ${TEST_FLAGS})
target_link_libraries(test_adc_dsp PRIVATE ztest_shim)
add_test(NAME adc_dsp COMMAND test_adc_dsp)

# --- protocol_vectors: the firmware-side consumer of tests/protocol_vectors.txt
#     CONTRIBUTING.md claims (#31 finding E5) -------------------------------
#
# Feeds the canonical vectors' own bytes through the REAL transport_spi.c /
# transport_i2c.c -> protocol.c seams (same link set as test_transport_spi /
# test_transport_i2c above: hal/bridge_hw_stub.c, unpartitioned OTA) and
# asserts the emitted reply bytes.  Before this target, nothing under
# tests/unit/ read tests/protocol_vectors.txt at all -- the only automation
# touching the file was the generator's own regenerate-and-diff, a comparison
# with itself (see issue #68's postmortem: that comparison stayed green
# through a whole-file CRC byte-order inversion).
#
# The vectors are baked into a generated header (not parsed at test runtime)
# by tests/gen_protocol_vectors_header.py, mirroring cmake/gen_build_id.cmake's
# generate-don't-parse precedent -- see that script's docstring.  Regenerated
# on every build; never committed.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PV_TXT ${FW}/tests/protocol_vectors.txt)
set(PV_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/protocol_vectors_gen)
set(PV_GEN_HEADER ${PV_GEN_DIR}/protocol_vectors_gen.h)
add_custom_command(
    OUTPUT ${PV_GEN_HEADER}
    COMMAND ${Python3_EXECUTABLE} ${FW}/tests/gen_protocol_vectors_header.py
            ${PV_TXT} ${PV_GEN_HEADER}
    DEPENDS ${PV_TXT} ${FW}/tests/gen_protocol_vectors_header.py
    COMMENT "Generating protocol_vectors_gen.h from tests/protocol_vectors.txt")
add_custom_target(protocol-vectors-header DEPENDS ${PV_GEN_HEADER})

add_executable(test_protocol_vectors
    protocol_vectors/src/test_protocol_vectors.c
    ${FW}/src/transport_spi.c
    ${FW}/src/transport_i2c.c
    ${FW}/src/protocol.c
    ${FW}/src/bootloader/bootloader.c
    ${FW}/src/ota.c
    ${FW}/src/crc32.c
    ${FW}/hal/bridge_hw_stub.c
)
add_dependencies(test_protocol_vectors protocol-vectors-header)
target_include_directories(test_protocol_vectors PRIVATE ${FW}/src ${FW}/hal ${PV_GEN_DIR})
target_compile_options(test_protocol_vectors PRIVATE ${TEST_FLAGS})
target_link_libraries(test_protocol_vectors PRIVATE ztest_shim)
add_test(NAME protocol_vectors COMMAND test_protocol_vectors)
