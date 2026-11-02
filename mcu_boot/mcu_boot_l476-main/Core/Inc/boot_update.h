#ifndef BOOT_UPDATE_H
#define BOOT_UPDATE_H
#include <stdint.h>
#include "bootloader.h"
#include "bootutil/image.h"
#define BOOT_CAN_REQUEST_ID  BOOTLOADER_BOOTLOADER_REQUEST_FRAME_ID
#define BOOT_CAN_RESPONSE_ID BOOTLOADER_BOOTLOADER_RESPONSE_FRAME_ID
#define BOOT_CAN_HEARTBEAT_ID BOOTLOADER_BCI_BOOT_HEARTBEAT_FRAME_ID
#define BOOT_CAN_DIAGNOSTIC_BASE_ID BOOTLOADER_BCI_BOOT_INFO_FRAME_ID
enum { UPDATE_INFO, UPDATE_BEGIN, UPDATE_DATA, UPDATE_END, UPDATE_RESET };
enum { UPDATE_OK, UPDATE_BAD_FRAME, UPDATE_BAD_STATE, UPDATE_BAD_SIZE,
       UPDATE_BAD_OFFSET, UPDATE_FLASH_ERROR, UPDATE_BAD_IMAGE };
enum { DIAG_INFO, DIAG_VERSION, DIAG_DECISION, DIAG_UPLOAD,
       DIAG_ERROR, DIAG_ROLLBACK, DIAG_FLASH, DIAG_REJECTED_VERSION };
enum { UPLOAD_IDLE, UPLOAD_ERASING, UPLOAD_RECEIVING, UPLOAD_VALIDATING,
       UPLOAD_COMMITTED, UPLOAD_FAILED };
enum { ERROR_NONE, ERROR_HEADER, ERROR_TLV, ERROR_HASH, ERROR_VECTORS,
       ERROR_CRC, ERROR_FRAME, ERROR_STATE, ERROR_SIZE, ERROR_OFFSET,
       ERROR_FLASH_READ, ERROR_FLASH_WRITE, ERROR_FLASH_ERASE,
       ERROR_FLASH_UNLOCK, ERROR_FLASH_LOCK, ERROR_CAN_QUEUE,
       ERROR_CAN_TIMEOUT, ERROR_HANDOFF, ERROR_FLASH_CONFIG, ERROR_INACTIVITY,
       ERROR_RECORD, ERROR_FLASH_OPERATION };
enum { SOURCE_VALIDATION = 1, SOURCE_UPLOAD, SOURCE_FLASH, SOURCE_CAN, SOURCE_BOOT };
enum { ENTRY_RAM = 1, ENTRY_BUTTON = 2, ENTRY_RECOVERY = 4, ENTRY_ROLLBACK = 8 };

/* Main-loop-owned snapshots; repeated changes coalesce while CAN is busy. */
struct boot_diagnostics {
    uint16_t pending;
    uint8_t selected_bank, valid_banks, rejected_bank, reject_reason;
    uint8_t rejected_version_known, entry_reason, upload_bank, upload_stage;
    uint8_t error_code, error_bank, error_command, error_source;
    uint8_t current_command;
    struct image_version selected_version, rejected_version;
    uint32_t reset_flags, rejected_generation, received, total;
    uint32_t error_detail, flash_offset, flash_error, error_sequence;
};
extern struct boot_diagnostics boot_diagnostics;
void boot_diag_error(uint8_t code, uint8_t bank, uint8_t source, uint32_t detail);
void boot_diag_flash_error(uint8_t code, uint32_t offset, uint32_t hal_error);
void boot_diag_snapshot(void);
void boot_diag_flush(void);

/* Each call consumes one classic CAN data frame and creates an 8-byte reply. */
void boot_update_frame(const uint8_t *data, uint8_t len, uint8_t reply[8]);
struct boot_rsp;
/* Validate both physical banks and select the newest committed, intact image. */
int boot_select(struct boot_rsp *rsp);
int boot_update_active(void);
int boot_image_vectors_valid(uint32_t sp, uint32_t pc, uint32_t image_size);
#endif
