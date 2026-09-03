/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway port glue for the mcuboot-size measurement. NOT production.
 *
 * MCUboot expects a generated sysflash.h mapping (image, slot) to a
 * flash_area ID; a real port generates this from board.yaml / a partition
 * table. One image, two slots (direct-XIP, no scratch).
 */
#ifndef SYSFLASH_H
#define SYSFLASH_H

#define PRIMARY_ID   0
#define SECONDARY_ID 1

#define FLASH_AREA_IMAGE_PRIMARY(x)   PRIMARY_ID
#define FLASH_AREA_IMAGE_SECONDARY(x) SECONDARY_ID

#endif /* SYSFLASH_H */
