/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway port glue for the mcuboot-size measurement. NOT production.
 *
 * bootutil_keys[] is normally generated from a real signing key's public
 * half by `imgtool.py getpub` (see boot/zephyr/CMakeLists.txt's
 * autogen-pubkey.c step) -- gd32-bridge has no factory-provisioned key yet
 * (src/ota_layout.h: "signature field is reserved... until factory key
 * provisioning exists"). This is a size-correct DUMMY DER SubjectPublicKeyInfo
 * (91 bytes: the standard encoding for an uncompressed P-256 point) with the
 * key material zeroed -- NOT a real key, never used to verify anything for
 * real.
 */
#include "bootutil/sign_key.h"

static const uint8_t dummy_ec256_pubkey_der[91] = {
	0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02, 0x01, 0x06,
	0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04,
	/* 64 bytes of X||Y follow; zeroed dummy, never a real point. */
};

static const unsigned int dummy_ec256_pubkey_len = sizeof(dummy_ec256_pubkey_der);

const struct bootutil_key bootutil_keys[] = {
	{
	    .key = dummy_ec256_pubkey_der,
	    .len = &dummy_ec256_pubkey_len,
	},
};

const int bootutil_key_cnt = 1;
