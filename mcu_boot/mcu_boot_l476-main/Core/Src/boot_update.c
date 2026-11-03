#include "boot_update.h"
#include "mcuboot_config/region_defs.h"
#include "flash_map/flash_map.h"
#include "bootutil/bootutil.h"
#include "bootutil/image.h"
#include "bootutil/bootutil_log.h"
#include <string.h>

static const struct flash_area *slot;
static uint32_t length, received, crc, generation;
static uint8_t target_id = FLASH_AREA_2_ID;

/* Two separately programmed doublewords: END commits the complements. */
struct bank_record { uint32_t length, generation, length_inv, generation_inv; };
static uint8_t active, complete, word[8], last[8];

struct boot_diagnostics boot_diagnostics;

/* The board drains diagnostics between blocking update phases; tests need no CAN. */
__attribute__((weak)) void boot_diag_flush(void) {}

void boot_diag_error(uint8_t code, uint8_t bank, uint8_t source, uint32_t detail)
{
    ++boot_diagnostics.error_sequence;
    boot_diagnostics.error_code = code;
    boot_diagnostics.error_bank = bank;
    boot_diagnostics.error_command = boot_diagnostics.current_command;
    boot_diagnostics.error_source = source;
    boot_diagnostics.error_detail = detail;
    boot_diagnostics.pending |= 1U << DIAG_ERROR;
}

void boot_diag_flash_error(uint8_t code, uint32_t offset, uint32_t hal_error)
{
    boot_diag_error(code, offset < 0x80000U ? 1 : 2, SOURCE_FLASH, offset);
    boot_diagnostics.flash_offset = offset;
    boot_diagnostics.flash_error = hal_error;
    boot_diagnostics.pending |= 1U << DIAG_FLASH;
}

void boot_diag_snapshot(void)
{
    boot_diagnostics.pending |= (1U << DIAG_INFO) | (1U << DIAG_DECISION) | (1U << DIAG_UPLOAD);
    if (boot_diagnostics.selected_bank) boot_diagnostics.pending |= 1U << DIAG_VERSION;
    if (boot_diagnostics.error_code) boot_diagnostics.pending |= 1U << DIAG_ERROR;
    if (boot_diagnostics.error_source == SOURCE_FLASH) boot_diagnostics.pending |= 1U << DIAG_FLASH;
    if (boot_diagnostics.rejected_bank && boot_diagnostics.selected_bank) {
        boot_diagnostics.pending |= 1U << DIAG_ROLLBACK;
        if (boot_diagnostics.rejected_version_known)
            boot_diagnostics.pending |= 1U << DIAG_REJECTED_VERSION;
    }
}

static void upload_status(uint8_t stage)
{
    boot_diagnostics.upload_stage = stage;
    boot_diagnostics.upload_bank = target_id == FLASH_AREA_0_ID ? 1 : 2;
    boot_diagnostics.received = received;
    boot_diagnostics.total = length;
    boot_diagnostics.pending |= 1U << DIAG_UPLOAD;
    boot_diag_flush();
}

int boot_image_vectors_valid(uint32_t sp, uint32_t pc, uint32_t image_size)
{
    uint32_t start = FLASH_BASE_ADDRESS + FLASH_AREA_0_OFFSET + BOOT_IMAGE_HEADER_SIZE;
    int ram = (sp > BOOT_RAM_START && sp <= BOOT_RAM_START + BOOT_RAM_SIZE) ||
              (sp > BOOT_RAM2_START && sp <= BOOT_RAM2_START + BOOT_RAM2_SIZE);
    return ram && (sp & 7U) == 0 && (pc & 1U) != 0 &&
           (pc & ~1U) >= start && (pc & ~1U) - start < image_size;
}

static int validate_image(const struct flash_area *area, uint32_t expected,
                          struct image_header *hdr)
{
    struct image_tlv_info tlv;
    uint8_t tmp[256];
    uint32_t vectors[2], end;
    uint8_t bank = area->fa_id == FLASH_AREA_0_ID ? 1 : 2;
    if (flash_area_read(area, 0, hdr, sizeof *hdr) != 0) {
        boot_diag_error(ERROR_FLASH_READ, bank, SOURCE_VALIDATION, area->fa_off);
        return -1;
    }
    if (hdr->ih_magic != IMAGE_MAGIC || hdr->ih_hdr_size != BOOT_IMAGE_HEADER_SIZE ||
        hdr->ih_flags != 0 || hdr->ih_protect_tlv_size != 0 || hdr->ih_img_size < 8 ||
        hdr->ih_img_size > BOOT_UPLOAD_MAX_SIZE - BOOT_IMAGE_HEADER_SIZE - sizeof tlv) {
        boot_diag_error(ERROR_HEADER, bank, SOURCE_VALIDATION, area->fa_off);
        BOOT_LOG_WRN("Bank image at 0x%08lx: invalid header/size", (unsigned long)area->fa_off);
        return -1;
    }
    end = hdr->ih_hdr_size + hdr->ih_img_size;
    if (flash_area_read(area, end, &tlv, sizeof tlv) != 0 ||
        flash_area_read(area, hdr->ih_hdr_size, vectors, sizeof vectors) != 0) {
        boot_diag_error(ERROR_FLASH_READ, bank, SOURCE_VALIDATION, area->fa_off + end);
        return -1;
    }
    if (tlv.it_magic != IMAGE_TLV_INFO_MAGIC || tlv.it_tlv_tot < sizeof tlv ||
        tlv.it_tlv_tot > BOOT_UPLOAD_MAX_SIZE - end ||
        (expected && end + tlv.it_tlv_tot != expected)) {
        boot_diag_error(ERROR_TLV, bank, SOURCE_VALIDATION, area->fa_off + end);
        BOOT_LOG_WRN("Bank image at 0x%08lx: invalid TLV/length", (unsigned long)area->fa_off);
        return -1;
    }
    if (!boot_image_vectors_valid(vectors[0], vectors[1], hdr->ih_img_size)) {
        boot_diag_error(ERROR_VECTORS, bank, SOURCE_VALIDATION, vectors[1]);
        return -1;
    }
    if (!fih_eq(bootutil_img_validate(NULL, 0, hdr, area, tmp,
                                    sizeof tmp, NULL, 0, NULL), FIH_SUCCESS)) {
        boot_diag_error(ERROR_HASH, area->fa_id == FLASH_AREA_0_ID ? 1 : 2, SOURCE_VALIDATION, area->fa_off);
        BOOT_LOG_ERR("Bank image at 0x%08lx: SHA-256 validation failed", (unsigned long)area->fa_off);
        return -1;
    }
    return 0;
}

/* shortcut: rollback checks integrity at reset; add confirmation/watchdog for runtime health failures. */
int boot_select(struct boot_rsp *rsp)
{
    static struct image_header headers[2];
    uint32_t reset_flags = boot_diagnostics.reset_flags;
    memset(&boot_diagnostics, 0, sizeof boot_diagnostics);
    boot_diagnostics.reset_flags = reset_flags;
    boot_diagnostics.current_command = 0xff;
    uint32_t rejected_generation = 0;
    const uint8_t ids[] = {FLASH_AREA_0_ID, FLASH_AREA_2_ID};
    int selected = -1;
    uint32_t selected_generation = 0;
    generation = 0;
    active = complete = 0;
    received = length = 0;
    memset(headers, 0, sizeof headers);
    memset(rsp, 0, sizeof *rsp);
    for (unsigned i = 0; i < 2; ++i) {
        const struct flash_area *area;
        struct bank_record record;
        if (flash_area_open(ids[i], &area) != 0) continue;
        int read_ok = flash_area_read(area, BOOT_BANK_RECORD_OFFSET, &record, sizeof record) == 0;
        if (!read_ok) boot_diag_error(ERROR_FLASH_READ, i + 1, SOURCE_VALIDATION, area->fa_off + BOOT_BANK_RECORD_OFFSET);
        int committed = read_ok && record.generation != 0 && record.generation != UINT32_MAX &&
                        record.length >= BOOT_IMAGE_HEADER_SIZE + 8 + 40 &&
                        record.length <= BOOT_UPLOAD_MAX_SIZE &&
                        record.length_inv == ~record.length &&
                        record.generation_inv == ~record.generation;
        /* Accept an existing bank-1 image once, before its first A/B update. */
        int legacy = read_ok && i == 0 && record.length == UINT32_MAX &&
                     record.generation == UINT32_MAX && record.length_inv == UINT32_MAX &&
                     record.generation_inv == UINT32_MAX;
        if (committed && record.generation > generation) generation = record.generation;
        if (!committed && !legacy) {
            if (read_ok && record.length != UINT32_MAX)
                boot_diag_error(ERROR_RECORD, i + 1, SOURCE_BOOT, area->fa_off + BOOT_BANK_RECORD_OFFSET);
            BOOT_LOG_WRN("Bank %u: no committed image", i + 1);
            flash_area_close(area);
            continue;
        }
        uint32_t sequence = committed ? record.generation : 0;
        if (validate_image(area, committed ? record.length : 0, &headers[i]) == 0) {
            boot_diagnostics.valid_banks |= 1U << i;
            BOOT_LOG_INF("Bank %u: valid generation %lu, version %u.%u.%u+%lu", i + 1,
                         (unsigned long)sequence, headers[i].ih_ver.iv_major,
                         headers[i].ih_ver.iv_minor, headers[i].ih_ver.iv_revision,
                         (unsigned long)headers[i].ih_ver.iv_build_num);
            if (selected < 0 || sequence > selected_generation) {
                selected = (int)i;
                selected_generation = sequence;
                rsp->br_hdr = &headers[i];
                rsp->br_flash_dev_id = area->fa_device_id;
                rsp->br_image_off = area->fa_off;
            }
        } else if (committed && sequence >= rejected_generation) {
            rejected_generation = sequence;
            boot_diagnostics.rejected_bank = i + 1;
            boot_diagnostics.reject_reason = boot_diagnostics.error_code;
            boot_diagnostics.rejected_generation = sequence;
            boot_diagnostics.rejected_version_known = headers[i].ih_magic == IMAGE_MAGIC;
            boot_diagnostics.rejected_version = headers[i].ih_ver;
        }
        flash_area_close(area);
    }
    if (selected >= 0) {
        boot_diagnostics.selected_bank = selected + 1;
        boot_diagnostics.selected_version = headers[selected].ih_ver;
    }
    if (selected >= 0 && rejected_generation <= selected_generation) {
        boot_diagnostics.rejected_bank = 0;
        boot_diagnostics.reject_reason = 0;
    }
    target_id = selected == 1 ? FLASH_AREA_0_ID : FLASH_AREA_2_ID;
    upload_status(UPLOAD_IDLE);
    boot_diag_snapshot();
    if (selected < 0) {
        BOOT_LOG_ERR("No valid committed image; CAN recovery mode");
        return -1;
    }
    if (selected_generation < generation)
        BOOT_LOG_WRN("Rollback: newest image damaged; selecting retained bank %u", selected + 1);
    BOOT_LOG_INF("Selected bank %u; next upload targets bank %u", selected + 1,
                 target_id == FLASH_AREA_0_ID ? 1 : 2);
    return 0;
}

int boot_update_active(void) { return active || complete; }

void boot_update_frame(const uint8_t *data, uint8_t len, uint8_t reply[8])
{
    uint8_t status = UPDATE_OK;
    uint8_t cmd = len ? data[0] : 0x7f;
    uint32_t n, off, value = 0;
    struct bootloader_bootloader_request_t request = {0};
    boot_diagnostics.current_command = cmd;
    uint32_t previous_error_sequence = boot_diagnostics.error_sequence;
    memset(reply, 0, 8);
    if (len != 8 || bootloader_bootloader_request_unpack(&request, data, len) != 0) {
        status = UPDATE_BAD_FRAME; goto respond;
    }
    cmd = request.command;
    switch (cmd) {
    case UPDATE_INFO:
        boot_diag_snapshot();
        BOOT_LOG_INF("CAN INFO: max upload %lu bytes", (unsigned long)BOOT_UPLOAD_MAX_SIZE);
        value = BOOT_UPLOAD_MAX_SIZE;
        break;
    case UPDATE_BEGIN:
        if (complete) { status = UPDATE_BAD_STATE; break; }
        n = request.image_length;
        if (n < BOOT_IMAGE_HEADER_SIZE + 8 + 40 || n > BOOT_UPLOAD_MAX_SIZE) {
            status = UPDATE_BAD_SIZE; break;
        }
        if (active && received == 0 && n == length) break;
        if (generation >= UINT32_MAX - 1U) { status = UPDATE_BAD_STATE; break; }
        BOOT_LOG_INF("Upload BEGIN: bank %u, %lu bytes; erasing inactive slot",
                     target_id == FLASH_AREA_0_ID ? 1 : 2, (unsigned long)n);
        active = complete = 0;
        received = 0;
        length = n;
        upload_status(UPLOAD_ERASING);
        if (flash_area_open(target_id, &slot) != 0 ||
            /* Invalidate the old record before touching its payload. */
            flash_area_erase(slot, slot->fa_size - FLASH_AREA_IMAGE_SECTOR_SIZE,
                             FLASH_AREA_IMAGE_SECTOR_SIZE) != 0 ||
            flash_area_erase(slot, 0, slot->fa_size - FLASH_AREA_IMAGE_SECTOR_SIZE) != 0) {
            status = UPDATE_FLASH_ERROR; break;
        }
        struct bank_record record = {n, generation + 1U, UINT32_MAX, UINT32_MAX};
        if (flash_area_write(slot, BOOT_BANK_RECORD_OFFSET, &record, 8) != 0) {
            status = UPDATE_FLASH_ERROR; break;
        }
        length = n;
        crc = 0xffffffffU;
        memset(word, 0xff, sizeof word);
        active = 1;
        upload_status(UPLOAD_RECEIVING);
        BOOT_LOG_INF("Upload ready: generation %lu", (unsigned long)(generation + 1U));
        break;
    case UPDATE_DATA:
        if (!active) { status = UPDATE_BAD_STATE; break; }
        off = request.image_offset;
        /* A lost ACK must never cause a second program of the same ECC word. */
        if (received && memcmp(last, data, 8) == 0) break;
        if (off != received || received == length) { status = UPDATE_BAD_OFFSET; break; }
        n = length - received;
        if (n > 4) n = 4;
        const uint8_t chunk[] = {request.payload_byte0, request.payload_byte1,
                                 request.payload_byte2, request.payload_byte3};
        memcpy(word + (received & 7U), chunk, n);
        for (uint32_t i = 0; i < n; ++i) {
            crc ^= chunk[i];
            for (unsigned j = 0; j < 8; ++j)
                crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0xedb88320U);
        }
        off = received & ~7U;
        received += n;
        if ((received & 7U) == 0 || received == length) {
            if (flash_area_write(slot, off, word, sizeof word) != 0) {
                active = 0; status = UPDATE_FLASH_ERROR; break;
            }
            memset(word, 0xff, sizeof word);
        }
        memcpy(last, data, 8);
        boot_diagnostics.received = received;
        if (received % 16384U == 0 || received == length) {
            upload_status(UPLOAD_RECEIVING);
            BOOT_LOG_INF("Upload progress: %lu/%lu bytes", (unsigned long)received, (unsigned long)length);
        }
        break;
    case UPDATE_END:
        if (complete && request.image_crc32 == (crc ^ 0xffffffffU)) break;
        if (!active || received != length) { status = UPDATE_BAD_STATE; break; }
        upload_status(UPLOAD_VALIDATING);
        if (request.image_crc32 != (crc ^ 0xffffffffU)) {
            boot_diag_error(ERROR_CRC, boot_diagnostics.upload_bank, SOURCE_UPLOAD, crc ^ 0xffffffffU);
            BOOT_LOG_WRN("Upload CRC mismatch: host 0x%08lx, device 0x%08lx",
                         (unsigned long)request.image_crc32, (unsigned long)(crc ^ 0xffffffffU));
            status = UPDATE_BAD_IMAGE; break;
        }
        struct image_header hdr;
        if (validate_image(slot, length, &hdr) != 0) { status = UPDATE_BAD_IMAGE; break; }
        uint32_t commit[] = {~length, ~(generation + 1U)};
        if (flash_area_write(slot, BOOT_BANK_RECORD_OFFSET + 8U, commit, sizeof commit) != 0) {
            active = 0; status = UPDATE_FLASH_ERROR; break;
        }
        active = 0;
        complete = 1;
        upload_status(UPLOAD_COMMITTED);
        BOOT_LOG_INF("Upload committed: bank %u, generation %lu; reset to activate",
                     target_id == FLASH_AREA_0_ID ? 1 : 2, (unsigned long)(generation + 1U));
        break;
    case UPDATE_RESET:
        break;
    default:
        status = UPDATE_BAD_FRAME;
        break;
    }
respond:
    if (status != UPDATE_OK) {
        if (boot_diagnostics.error_sequence == previous_error_sequence) {
            const uint8_t codes[] = {ERROR_NONE, ERROR_FRAME, ERROR_STATE, ERROR_SIZE,
                                     ERROR_OFFSET, ERROR_FLASH_OPERATION, ERROR_HASH};
            uint32_t detail = received;
            if (status == UPDATE_BAD_FRAME) detail = len;
            else if (status == UPDATE_BAD_SIZE) detail = request.image_length;
            else if (status == UPDATE_BAD_OFFSET)
                detail = request.image_offset;
            boot_diag_error(codes[status], boot_diagnostics.upload_bank, SOURCE_UPLOAD, detail);
        }
        upload_status(status == UPDATE_BAD_IMAGE || status == UPDATE_FLASH_ERROR || (!active && !complete)
                      ? UPLOAD_FAILED : complete ? UPLOAD_COMMITTED : UPLOAD_RECEIVING);
    }
    if (status != UPDATE_OK)
        BOOT_LOG_WRN("CAN command 0x%02x rejected: status %u, received %lu/%lu, DLC %u",
                     cmd, status, (unsigned long)received, (unsigned long)length, len);
    if (cmd != UPDATE_INFO) value = received;
    const struct bootloader_bootloader_response_t response = {
        cmd | 0x80U, status, value, BOOT_PROTOCOL_VERSION, complete ? 2 : active
    };
    bootloader_bootloader_response_pack(reply, &response, 8);
}
