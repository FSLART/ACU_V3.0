#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "boot_update.h"
#include "mcuboot_config/region_defs.h"
#include "flash_map/flash_map.h"
#include "bootutil/bootutil.h"
#include "bootutil/image.h"
#include "mbedtls/sha256.h"

static uint8_t memory[0x100000], image[569], reply[8];
static int writes, erases, fail_write, fail_erase, fail_read_bank, reset_after;
static jmp_buf reset;
static ARM_FLASH_INFO info = {NULL, 512, 2048, 8, 8, 0xff};
void Error_Handler(void) { assert(0); }
static int32_t read_data(uint32_t off, void *p, uint32_t len)
{
    assert(off <= sizeof memory && len <= sizeof memory - off);
    if (fail_read_bank && (off < 0x80000U ? 1 : 2) == fail_read_bank) return -1;
    memcpy(p, memory + off, len);
    return 0;
}
static int32_t program(uint32_t off, const void *p, uint32_t len)
{
    assert((off >= FLASH_AREA_0_OFFSET && off + len <= FLASH_AREA_0_OFFSET + FLASH_AREA_0_SIZE) ||
           (off >= FLASH_AREA_2_OFFSET && off + len <= FLASH_AREA_2_OFFSET + FLASH_AREA_2_SIZE));
    assert(off % 8 == 0 && len % 8 == 0);
    if (fail_write) return -1;
    if (reset_after && ++writes == reset_after) {
        memory[off] = ((const uint8_t *)p)[0]; /* Partially programmed commit. */
        longjmp(reset, 1);
    }
    for (uint32_t i = 0; i < len; ++i) assert(memory[off + i] == 0xff);
    memcpy(memory + off, p, len);
    return 0;
}
static int32_t erase(uint32_t off)
{
    assert(((off >= FLASH_AREA_0_OFFSET && off + 2048 <= FLASH_AREA_0_OFFSET + FLASH_AREA_0_SIZE) ||
            (off >= FLASH_AREA_2_OFFSET && off + 2048 <= FLASH_AREA_2_OFFSET + FLASH_AREA_2_SIZE)) && off % 2048 == 0);
    if (fail_erase) return -1;
    ++erases;
    memset(memory + off, 0xff, 2048);
    return 0;
}
static ARM_FLASH_INFO *get_info(void) { return &info; }
ARM_DRIVER_FLASH Driver_FLASH0 = {
    .ReadData = read_data, .ProgramData = program,
    .EraseSector = erase, .GetInfo = get_info
};
static void command(uint8_t cmd, uint32_t value, uint8_t expected)
{
    uint8_t frame[8] = {cmd};
    memcpy(frame + 1, &value, 4);
    boot_update_frame(frame, 8, reply);
    if (reply[1] != expected)
        printf("command %u value %lu: status %u expected %u\n", cmd, (unsigned long)value, reply[1], expected);
    assert(reply[0] == (cmd | 0x80) && reply[1] == expected && reply[6] == 1);
}
static uint32_t crc32(void)
{
    uint32_t c = 0xffffffffU;
    for (unsigned i = 0; i < sizeof image; ++i) {
        c ^= image[i];
        for (unsigned j = 0; j < 8; ++j) c = (c >> 1) ^ ((0U - (c & 1U)) & 0xedb88320U);
    }
    return ~c;
}
static void make_image(void)
{
    struct image_header h = {.ih_magic = IMAGE_MAGIC, .ih_hdr_size = 512, .ih_img_size = 17, .ih_ver = {1, 2, 3, 4}};
    uint32_t v[] = {0x20018000, 0x08010209};
    memset(image, 0, sizeof image);
    memcpy(image, &h, sizeof h);
    memcpy(image + 512, v, sizeof v);
    image[528] = 0x42;
    struct image_tlv_info t = {IMAGE_TLV_INFO_MAGIC, 40};
    memcpy(image + 529, &t, sizeof t);
    image[533] = IMAGE_TLV_SHA256;
    image[535] = 32;
    assert(mbedtls_sha256_ret(image, 529, image + 537, 0) == 0);
}
static void upload(void)
{
    command(UPDATE_BEGIN, sizeof image, UPDATE_OK);
    for (uint32_t off = 0; off < sizeof image; off += 4) {
        uint8_t f[8] = {UPDATE_DATA, off, off >> 8, off >> 16};
        uint32_t n = sizeof image - off;
        if (n > 4) n = 4;
        memcpy(f + 4, image + off, n);
        boot_update_frame(f, 8, reply);
        assert(reply[1] == UPDATE_OK);
        boot_update_frame(f, 8, reply); /* Lost ACK: identical retry must not write again. */
        assert(reply[1] == UPDATE_OK);
    }
}
int main(void)
{
    struct boot_rsp rsp;
    memset(memory, 0xff, sizeof memory);
    make_image();
    assert(boot_select(&rsp) != 0);
    command(UPDATE_INFO, 0, UPDATE_OK);
    assert(reply[7] == 0);
    command(UPDATE_DATA, 0, UPDATE_BAD_STATE);
    command(UPDATE_BEGIN, 0xffffffffU, UPDATE_BAD_SIZE);
    uint8_t short_frame[8] = {UPDATE_BEGIN};
    boot_update_frame(short_frame, 5, reply);
    assert(reply[1] == UPDATE_BAD_FRAME && erases == 0);
    command(UPDATE_BEGIN, sizeof image, UPDATE_OK);
    assert(boot_diagnostics.upload_stage == UPLOAD_RECEIVING && boot_diagnostics.total == sizeof image);
    assert(boot_diagnostics.upload_bank == 2 && boot_diagnostics.received == 0);
    int old_erases = erases;
    command(UPDATE_BEGIN, sizeof image, UPDATE_OK);
    assert(erases == old_erases);
    command(UPDATE_DATA, 4, UPDATE_BAD_OFFSET);
    command(UPDATE_END, crc32(), UPDATE_BAD_STATE);
    upload();
    assert(memcmp(memory + FLASH_AREA_2_OFFSET, image, sizeof image) == 0);
    command(UPDATE_END, crc32() ^ 1, UPDATE_BAD_IMAGE);
    assert(boot_diagnostics.error_code == ERROR_CRC && boot_diagnostics.error_source == SOURCE_UPLOAD);
    assert(boot_diagnostics.error_command == UPDATE_END && boot_diagnostics.error_detail == crc32());
    command(UPDATE_INFO, 0, UPDATE_OK);
    assert(boot_diagnostics.error_code == ERROR_CRC && (boot_diagnostics.pending & (1U << DIAG_ERROR)));
    memory[FLASH_AREA_2_OFFSET + 528] ^= 1;
    command(UPDATE_END, crc32(), UPDATE_BAD_IMAGE);
    memory[FLASH_AREA_2_OFFSET + 528] ^= 1;
    command(UPDATE_END, crc32(), UPDATE_OK);
    command(UPDATE_END, crc32(), UPDATE_OK);
    assert(reply[7] == 2);
    assert(boot_diagnostics.upload_stage == UPLOAD_COMMITTED && boot_diagnostics.received == sizeof image);
    command(UPDATE_BEGIN, sizeof image, UPDATE_BAD_STATE);
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET);
    assert(boot_diagnostics.selected_version.iv_major == 1 && boot_diagnostics.selected_version.iv_minor == 2);
    assert(boot_diagnostics.selected_version.iv_revision == 3 && boot_diagnostics.selected_version.iv_build_num == 4);
    assert(memory[FLASH_AREA_0_OFFSET] == 0xff); /* No copying between banks. */

    upload(); /* Next upload must target physical bank 1. */
    assert(memcmp(memory + FLASH_AREA_0_OFFSET, image, sizeof image) == 0);
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET); /* No END. */
    upload();
    command(UPDATE_END, crc32(), UPDATE_OK);
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_0_OFFSET);
    fail_read_bank = 1;
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET);
    fail_read_bank = 0;
    memory[FLASH_AREA_0_OFFSET + 528] ^= 1;
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET); /* SHA fallback. */
    assert(boot_diagnostics.selected_bank == 2 && boot_diagnostics.valid_banks == 2);
    assert(boot_diagnostics.rejected_bank == 1 && boot_diagnostics.reject_reason == ERROR_HASH);
    assert(boot_diagnostics.rejected_generation == 2 && boot_diagnostics.rejected_version_known);
    assert(boot_diagnostics.pending & (1U << DIAG_ROLLBACK));
    memory[FLASH_AREA_0_OFFSET + 528] ^= 1;
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_0_OFFSET);

    upload();
    writes = 0; reset_after = 1;
    if (setjmp(reset) == 0) { command(UPDATE_END, crc32(), UPDATE_OK); assert(0); }
    reset_after = 0;
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_0_OFFSET); /* Torn commit. */
    upload();
    command(UPDATE_END, crc32(), UPDATE_OK);
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET);
    memory[FLASH_AREA_2_OFFSET + BOOT_BANK_RECORD_OFFSET + 8] ^= 1;
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_0_OFFSET); /* Bad record. */
    memory[FLASH_AREA_2_OFFSET + BOOT_BANK_RECORD_OFFSET + 8] ^= 1;
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET);

    fail_erase = 1;
    command(UPDATE_BEGIN, sizeof image, UPDATE_FLASH_ERROR);
    fail_erase = 0;
    command(UPDATE_BEGIN, sizeof image, UPDATE_OK);
    command(UPDATE_DATA, 0, UPDATE_OK);
    fail_write = 1;
    command(UPDATE_DATA, 4, UPDATE_FLASH_ERROR);
    fail_write = 0;
    command(UPDATE_DATA, 4, UPDATE_BAD_STATE);
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET);
    assert(!boot_image_vectors_valid(0x20018001, 0x08010209, 17));
    assert(!boot_image_vectors_valid(0x20018000, 0x08010208, 17));
    assert(!boot_image_vectors_valid(0x20018000, 0x08000001, 17));

    memory[FLASH_AREA_2_OFFSET + 528] ^= 1;
    assert(boot_select(&rsp) != 0); /* Damaged newest + incomplete older => recovery. */
    memset(memory, 0xff, sizeof memory);
    memcpy(memory + FLASH_AREA_0_OFFSET, image, sizeof image); /* Legacy primary provisioning. */
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_0_OFFSET);
    upload();
    command(UPDATE_END, crc32(), UPDATE_OK);
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_2_OFFSET);
    memory[FLASH_AREA_2_OFFSET + 528] ^= 1;
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_0_OFFSET);
    uint32_t exhausted[] = {sizeof image, UINT32_MAX - 1U, ~((uint32_t)sizeof image), ~(UINT32_MAX - 1U)};
    memcpy(memory + FLASH_AREA_0_OFFSET + BOOT_BANK_RECORD_OFFSET, exhausted, sizeof exhausted);
    assert(boot_select(&rsp) == 0 && rsp.br_image_off == FLASH_AREA_0_OFFSET);
    command(UPDATE_BEGIN, sizeof image, UPDATE_BAD_STATE); /* Never wrap generation. */
    puts("PASS: A/B alternation, SHA rollback, incomplete upload/commit, legacy image, CAN retries/errors");
    return 0;
}
