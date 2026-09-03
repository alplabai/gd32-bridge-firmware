/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway port glue for the mcuboot-size measurement. NOT production.
 *
 * bootutil/crypto/sha.h includes this unconditionally, but its macros are
 * only ever invoked when MCUBOOT_HAVE_LOGGING is defined (see
 * bootutil/bootutil_log.h) -- which this probe never sets, since the real
 * 32 KB bootloader has no room for a log formatter. Empty on purpose.
 */
#ifndef MCUBOOT_SIZE_PROBE_LOGGING_H
#define MCUBOOT_SIZE_PROBE_LOGGING_H
#endif
