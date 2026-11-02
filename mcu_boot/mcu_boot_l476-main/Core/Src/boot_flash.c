#include "boot_port.h"
#include "boot_update.h"
#include "mcuboot_config/region_defs.h"
#include "Driver_Flash.h"
#include <string.h>
#include "bootutil/bootutil_log.h"

static int ready;
static ARM_FLASH_INFO info = {NULL, 512, 2048, 8, 8, 0xff};

int boot_flash_init(void)
{
    /* This L476RG port requires 1 MiB with the normal, unswapped bank mapping. */
    ready = FLASH_SIZE == 0x100000U &&
            (SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) == 0 &&
            (FLASH->OPTR & FLASH_OPTR_BFB2) == 0 &&
            (FLASH->OPTR & FLASH_OPTR_DUALBANK) != 0;
    if (!ready) {
        boot_diag_error(ERROR_FLASH_CONFIG, 0, SOURCE_BOOT, FLASH->OPTR);
        BOOT_LOG_ERR("Flash requires 1 MiB, DUALBANK=1, BFB2=0, FB_MODE=0: size=%lu OPTR=0x%08lx MEMRMP=0x%08lx",
                     (unsigned long)FLASH_SIZE, (unsigned long)FLASH->OPTR, (unsigned long)SYSCFG->MEMRMP);
    } else
        BOOT_LOG_INF("Flash ready: dual bank, slots 0x%08lx/0x%08lx, %lu bytes each",
                     (unsigned long)(FLASH_BASE_ADDRESS + FLASH_AREA_0_OFFSET),
                     (unsigned long)(FLASH_BASE_ADDRESS + FLASH_AREA_2_OFFSET), (unsigned long)FLASH_AREA_0_SIZE);
    return ready ? 0 : -1;
}

static int writable(uint32_t off, uint32_t len)
{
    return ready &&
        ((off >= FLASH_AREA_0_OFFSET && off <= FLASH_AREA_0_OFFSET + FLASH_AREA_0_SIZE &&
          len <= FLASH_AREA_0_OFFSET + FLASH_AREA_0_SIZE - off) ||
         (off >= FLASH_AREA_2_OFFSET && off <= FLASH_AREA_2_OFFSET + FLASH_AREA_2_SIZE &&
          len <= FLASH_AREA_2_OFFSET + FLASH_AREA_2_SIZE - off));
}

static int32_t read_data(uint32_t off, void *dst, uint32_t len)
{
    if (!ready || !dst || off > 0x100000U || len > 0x100000U - off) {
        boot_diag_flash_error(ERROR_FLASH_READ, off, HAL_FLASH_GetError());
        return -1;
    }
    memcpy(dst, (const void *)(FLASH_BASE_ADDRESS + off), len);
    /* ST's flash map expects zero, unlike the generic CMSIS byte-count convention. */
    return 0;
}

static int32_t program(uint32_t off, const void *src, uint32_t len)
{
    if (!src || !writable(off, len) || (off & 7U) || (len & 7U)) {
        boot_diag_flash_error(ERROR_FLASH_WRITE, off, HAL_FLASH_GetError());
        return -1;
    }
    if (HAL_FLASH_Unlock() != HAL_OK) {
        boot_diag_flash_error(ERROR_FLASH_UNLOCK, off, HAL_FLASH_GetError());
        BOOT_LOG_ERR("Flash unlock failed: error 0x%08lx", (unsigned long)HAL_FLASH_GetError());
        return -1;
    }
    int rc = 0;
    for (uint32_t i = 0; i < len; i += 8) {
        uint64_t data;
        memcpy(&data, (const uint8_t *)src + i, 8);
        /* An erased doubleword must retain its unprogrammed ECC bits. */
        if (data == UINT64_MAX) {
            if (*(const uint64_t *)(FLASH_BASE_ADDRESS + off + i) != UINT64_MAX) {
                boot_diag_flash_error(ERROR_FLASH_WRITE, off + i, HAL_FLASH_GetError());
                BOOT_LOG_ERR("Flash padding target not erased: offset 0x%08lx", (unsigned long)(off + i));
                rc = -1; break;
            }
            continue;
        }
        if (*(const uint64_t *)(FLASH_BASE_ADDRESS + off + i) != UINT64_MAX ||
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              FLASH_BASE_ADDRESS + off + i, data) != HAL_OK ||
            memcmp((const void *)(FLASH_BASE_ADDRESS + off + i), &data, 8) != 0) {
            boot_diag_flash_error(ERROR_FLASH_WRITE, off + i, HAL_FLASH_GetError());
            BOOT_LOG_ERR("Flash program/readback failed: offset 0x%08lx, error 0x%08lx",
                         (unsigned long)(off + i), (unsigned long)HAL_FLASH_GetError());
            rc = -1; break;
        }
    }
    if (HAL_FLASH_Lock() != HAL_OK) {
        boot_diag_flash_error(ERROR_FLASH_LOCK, off, HAL_FLASH_GetError());
        BOOT_LOG_ERR("Flash lock failed: error 0x%08lx", (unsigned long)HAL_FLASH_GetError());
        rc = -1;
    }
    return rc;
}

static int32_t erase(uint32_t off)
{
    if (!writable(off, FLASH_PAGE_SIZE) || off % FLASH_PAGE_SIZE) {
        boot_diag_flash_error(ERROR_FLASH_ERASE, off, HAL_FLASH_GetError());
        return -1;
    }
    FLASH_EraseInitTypeDef e = {0};
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Banks = off < 0x80000U ? FLASH_BANK_1 : FLASH_BANK_2;
    e.Page = (off % 0x80000U) / FLASH_PAGE_SIZE;
    e.NbPages = 1;
    uint32_t error = UINT32_MAX;
    if (HAL_FLASH_Unlock() != HAL_OK) {
        boot_diag_flash_error(ERROR_FLASH_UNLOCK, off, HAL_FLASH_GetError());
        BOOT_LOG_ERR("Flash unlock failed: error 0x%08lx", (unsigned long)HAL_FLASH_GetError());
        return -1;
    }
    int rc = HAL_FLASHEx_Erase(&e, &error) == HAL_OK ? 0 : -1;
    if (rc != 0) {
        boot_diag_flash_error(ERROR_FLASH_ERASE, off, HAL_FLASH_GetError());
        BOOT_LOG_ERR("Flash erase failed: bank %u page %lu, failing page %lu, HAL error 0x%08lx",
                     off < 0x80000U ? 1 : 2, (unsigned long)e.Page,
                     (unsigned long)error, (unsigned long)HAL_FLASH_GetError());
    }
    if (HAL_FLASH_Lock() != HAL_OK) {
        boot_diag_flash_error(ERROR_FLASH_LOCK, off, HAL_FLASH_GetError());
        BOOT_LOG_ERR("Flash lock failed: error 0x%08lx", (unsigned long)HAL_FLASH_GetError());
        rc = -1;
    }
    return rc;
}

static ARM_FLASH_INFO *get_info(void) { return &info; }
ARM_DRIVER_FLASH Driver_FLASH0 = {
    .ReadData = read_data, .ProgramData = program,
    .EraseSector = erase, .GetInfo = get_info
};
