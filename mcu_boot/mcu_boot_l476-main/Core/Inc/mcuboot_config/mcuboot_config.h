#ifndef MCUBOOT_CONFIG_H
#define MCUBOOT_CONFIG_H



/* Increment for released bootloader changes; board type 2 is STM32F412RETx */
#define BOOT_VERSION_MAJOR 1U
#define BOOT_VERSION_MINOR 0U
#define BOOT_VERSION_PATCH 0U
#define BOOT_BOARD_TYPE 2U
#define BOOT_PROTOCOL_VERSION 1U

/* STM32F412RETx – 512 KB internal Flash */
#define FLASH_BASE_ADDRESS       0x08000000UL

/* Bootloader: 64 KB */
#define BOOTLOADER_OFFSET        0x00000000UL
#define BOOTLOADER_SIZE          0x00010000UL

/* MCUboot Slot 0 - Bank 1 */
#define FLASH_AREA_0_OFFSET      0x00010000UL
#define FLASH_AREA_0_SIZE        0x00070000UL

/* MCUboot Slot 1 - Bank 2 */
#define FLASH_AREA_2_OFFSET      0x00090000UL
#define FLASH_AREA_2_SIZE        0x00070000UL

/* Application vector tables */
#define APP_SLOT0_VECTOR_ADDR    0x08010200UL
#define APP_SLOT1_VECTOR_ADDR    0x08090200UL

/* One firmware image */
#define MCUBOOT_IMAGE_NUMBER 1

/* MCUboot supplies image validation only; boot_select() owns A/B activation.
 * Retain overwrite-only trailer geometry for ST's erased-padding validator. */
#define MCUBOOT_OVERWRITE_ONLY 1

/* Flash driver */
#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS 1

/* Cryptographic backend */
#define MCUBOOT_USE_MBED_TLS 1

/* Logging: MCUboot BOOT_LOG_* goes to printf(), retargeted to USART2
 * (ST-LINK VCP, 115200 8N1) via __io_putchar() in boot_port.c. */
#define MCUBOOT_HAVE_LOGGING 1
#define MCUBOOT_LOG_LEVEL 3

/* Watchdog */
#define MCUBOOT_WATCHDOG_FEED() ((void)0)

/* Fault injection hardening */
#define MCUBOOT_FIH_PROFILE_OFF

/* Maximum number of flash sectors */
#define MCUBOOT_MAX_IMG_SECTORS 224

#endif /* MCUBOOT_CONFIG_H */
