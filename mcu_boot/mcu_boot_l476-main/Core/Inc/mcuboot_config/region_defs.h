#ifndef REGION_DEFS_H
#define REGION_DEFS_H
#include "mcuboot_config.h"

/*
 * STM32L476RG
 * Internal flash: 1 MB
 * Flash base:     0x08000000
 * Erase page:     2 KB
 *
 * MCUboot configuration:
 * - One application image
 * - A/B bank selection with integrity fallback
 * - Primary and secondary slots in internal flash
 */

/* Flash device */
#define FLASH_BASE_ADDRESS          0x08000000UL
#define FLASH_DEV_NAME              Driver_FLASH0
#define FLASH_AREA_IMAGE_SECTOR_SIZE 0x20000U
#define BOOT_IMAGE_HEADER_SIZE       0x200U
#define BOOT_RAM_START               0x20000000U
#define BOOT_RAM_SIZE                0x18000U
#define BOOT_RAM2_START              0x10000000U
#define BOOT_RAM2_SIZE               0x8000U
/* Keep the final page free for the A/B commit record. */
#define BOOT_UPLOAD_MAX_SIZE         (FLASH_AREA_2_SIZE - FLASH_AREA_IMAGE_SECTOR_SIZE)
#define BOOT_BANK_RECORD_OFFSET      (FLASH_AREA_2_SIZE - 16U)

/* Image counts used by the ST flash map */
#define MCUBOOT_APP_IMAGE_NUMBER    1
#define MCUBOOT_S_DATA_IMAGE_NUMBER 0
#define MCUBOOT_NS_DATA_IMAGE_NUMBER 0

/*
 * Flash layout:
 *
 * Bootloader:  0x08000000 - 0x0800FFFF (64 KB)
 * Primary:     0x08010000 - 0x0807FFFF (448 KB)
 * Bank 2 slot: 0x08090000 - 0x080FFFFF (448 KB)
 * Reserved:    0x08080000 - 0x0808FFFF (64 KB)
 *
 * Offsets are relative to FLASH_BASE_ADDRESS.
 */

/* Primary application slot */
#define FLASH_AREA_0_ID             1U
#define FLASH_AREA_0_OFFSET         0x00010000UL
#define FLASH_AREA_0_SIZE           0x00080000UL

/* Secondary application slot */
#define FLASH_AREA_2_ID             2U
#define FLASH_AREA_2_OFFSET         0x00090000UL
#define FLASH_AREA_2_SIZE           0x00070000UL

/* ST's flash map requires a scratch entry; A/B selection never writes it. */
#define FLASH_AREA_SCRATCH_ID       3U
#define FLASH_AREA_SCRATCH_OFFSET   0x00080000UL
#define FLASH_AREA_SCRATCH_SIZE     0x00002000UL

/* Shared-data code is excluded; these satisfy the ST header only. */
#define BOOT_SHARED_DATA_BASE       0U
#define BOOT_SHARED_DATA_SIZE       0U

#endif /* REGION_DEFS_H */
