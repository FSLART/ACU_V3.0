#include "boot_port.h"
#include "boot_update.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "mcuboot_config/region_defs.h"
#include "bootutil/bootutil.h"
#include "bootutil/image.h"
#include "bootutil/bootutil_log.h"

#define BOOTLOADER_FLAG_ADDR  0x20017FFCUL
#define BOOTLOADER_MAGIC      0xB00710ADUL

/* USART2 handle owned by CubeMX-generated main.c (PA2/PA3, 115200 8N1). */
extern UART_HandleTypeDef huart2;

/* Retarget printf() (used by MCUboot BOOT_LOG_*) to the ST-LINK VCP.
 * syscalls.c's weak _write() calls this per character. */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &c, 1, 100);
    return ch;
}



static bool BootloaderRequested(void)
{
    volatile uint32_t *flag =
        (volatile uint32_t *)BOOTLOADER_FLAG_ADDR;

    if (*flag == BOOTLOADER_MAGIC)
    {
        *flag = 0;  // Consume request
        __DSB();
        return true;
    }

    return false;
}



/* Runs entirely in SRAM: remapping must not fetch the next instruction from flash. */
__attribute__((section(".RamFunc"), naked, noreturn)) static void enter_app(
    uint32_t sp __attribute__((unused)), uint32_t pc __attribute__((unused)),
    uint32_t remap __attribute__((unused)))
{
    __asm volatile("ldr r3, =0x40010000\n" /* SYSCFG MEMRMP */
                   "str r2, [r3]\n"
                   "dsb\n"
                   "isb\n"
                   "msr msp, r0\n"
                   "movs r2, #0\n"
                   "msr control, r2\n"
                   "isb\n"
                   "cpsie i\n"
                   "bx r1\n");
}

_Static_assert(sizeof(struct image_version) == 8, "CAN firmware version must occupy 8 bytes");
_Static_assert(BOOT_VERSION_MAJOR <= 255 && BOOT_VERSION_MINOR <= 255 && BOOT_VERSION_PATCH <= 255,
               "Heartbeat version components must fit in a byte");
static CAN_HandleTypeDef *diagnostic_can;

static int send_diagnostic(CAN_HandleTypeDef *can)
{
    struct boot_diagnostics *d = &boot_diagnostics;
    if (!d->pending || HAL_CAN_GetTxMailboxesFreeLevel(can) <= 1U) return 0;
    unsigned kind = 0;
    while ((d->pending & (1U << kind)) == 0) ++kind;
    uint8_t data[8] = {0};
    uint32_t frame_id;
    switch (kind) {
    case DIAG_INFO: {
        const struct bootloader_bci_boot_info_t message = {
            BOOT_PROTOCOL_VERSION, BOOT_BOARD_TYPE, d->valid_banks, d->upload_stage, BOOT_UPLOAD_MAX_SIZE
        };
        bootloader_bci_boot_info_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_INFO_FRAME_ID;
        break;
    }
    case DIAG_VERSION: {
        const struct bootloader_bci_boot_firmware_version_t message = {
            d->selected_version.iv_major, d->selected_version.iv_minor, d->selected_version.iv_revision, d->selected_version.iv_build_num
        };
        bootloader_bci_boot_firmware_version_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_FIRMWARE_VERSION_FRAME_ID;
        break;
    }
    case DIAG_DECISION: {
        const struct bootloader_bci_boot_decision_t message = {
            d->selected_bank, d->entry_reason, d->rejected_bank, d->reject_reason, d->reset_flags
        };
        bootloader_bci_boot_decision_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_DECISION_FRAME_ID;
        break;
    }
    case DIAG_UPLOAD: {
        const struct bootloader_bci_boot_upload_status_t message = {
            d->upload_bank, d->upload_stage, d->received, d->total
        };
        bootloader_bci_boot_upload_status_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_UPLOAD_STATUS_FRAME_ID;
        break;
    }
    case DIAG_ERROR: {
        const struct bootloader_bci_boot_error_t message = {
            d->error_code, d->error_bank, d->error_command, d->error_source, d->error_detail
        };
        bootloader_bci_boot_error_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_ERROR_FRAME_ID;
        break;
    }
    case DIAG_ROLLBACK: {
        const struct bootloader_bci_boot_rollback_t message = {
            d->rejected_bank, d->selected_bank, d->reject_reason, d->rejected_version_known, d->rejected_generation
        };
        bootloader_bci_boot_rollback_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_ROLLBACK_FRAME_ID;
        break;
    }
    case DIAG_FLASH: {
        const struct bootloader_bci_boot_flash_error_t message = {
            d->flash_offset, d->flash_error
        };
        bootloader_bci_boot_flash_error_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_FLASH_ERROR_FRAME_ID;
        break;
    }
    case DIAG_REJECTED_VERSION: {
        const struct bootloader_bci_boot_rejected_version_t message = {
            d->rejected_version.iv_major, d->rejected_version.iv_minor, d->rejected_version.iv_revision, d->rejected_version.iv_build_num
        };
        bootloader_bci_boot_rejected_version_pack(data, &message, sizeof data);
        frame_id = BOOTLOADER_BCI_BOOT_REJECTED_VERSION_FRAME_ID;
        break;
    }
    default:
        d->pending = 0;
        return 0;
    }
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;
    tx.StdId = frame_id;
    tx.IDE = CAN_ID_STD; tx.RTR = CAN_RTR_DATA; tx.DLC = sizeof data;
    if (HAL_CAN_AddTxMessage(can, &tx, data, &mailbox) != HAL_OK) return 0;
    d->pending &= ~(1U << kind);
    return 1;
}

/* Bounded best-effort drain before flash phases, reset or application handoff. */
void boot_diag_flush(void)
{
    if (!diagnostic_can) return;
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 20U &&
           (boot_diagnostics.pending || HAL_CAN_GetTxMailboxesFreeLevel(diagnostic_can) < 3U))
        send_diagnostic(diagnostic_can);
}

static void jump(CAN_HandleTypeDef *can, const struct boot_rsp *rsp)
{
    if (rsp->br_hdr == NULL) {
        boot_diag_error(ERROR_HANDOFF, 0, SOURCE_BOOT, 0);
        boot_diag_flush();
        BOOT_LOG_ERR("Handoff rejected: missing image header");
        return;
    }

    if ((rsp->br_image_off != FLASH_AREA_0_OFFSET && rsp->br_image_off != FLASH_AREA_2_OFFSET) ||
        rsp->br_hdr->ih_hdr_size != BOOT_IMAGE_HEADER_SIZE) {
        boot_diag_error(ERROR_HANDOFF, 0, SOURCE_BOOT, rsp->br_image_off);
        boot_diag_flush();
        BOOT_LOG_ERR("Handoff rejected: offset/header mismatch");
        return;
    }
    uint32_t physical_vector = FLASH_BASE_ADDRESS + rsp->br_image_off + BOOT_IMAGE_HEADER_SIZE;
    const uint32_t *v = (const uint32_t *)physical_vector;
    uint32_t sp = v[0], pc = v[1];
    if (!boot_image_vectors_valid(sp, pc, rsp->br_hdr->ih_img_size)) {
        boot_diag_error(ERROR_HANDOFF, boot_diagnostics.selected_bank, SOURCE_BOOT, pc);
        boot_diag_flush();
        BOOT_LOG_ERR("Handoff rejected: SP=0x%08lx PC=0x%08lx", (unsigned long)sp, (unsigned long)pc);
        return;
    }
    uint32_t remap = SYSCFG->MEMRMP & ~SYSCFG_MEMRMP_FB_MODE;
    if (rsp->br_image_off == FLASH_AREA_2_OFFSET) remap |= SYSCFG_MEMRMP_FB_MODE;
    BOOT_LOG_INF("Boot bank %u: VTOR=0x%08lx SP=0x%08lx PC=0x%08lx",
                 rsp->br_image_off == FLASH_AREA_0_OFFSET ? 1 : 2,
                 (unsigned long)APP_SLOT0_VECTOR_ADDR, (unsigned long)sp, (unsigned long)pc);
    boot_diag_flush();
    fflush(stdout);
    HAL_UART_DeInit(&huart2);
    HAL_CAN_Stop(can);
    HAL_CAN_DeInit(can);
    HAL_RCC_DeInit();
    HAL_DeInit();
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (unsigned i = 0; i < 8; ++i) {
        NVIC->ICER[i] = 0xffffffffU;
        NVIC->ICPR[i] = 0xffffffffU;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    /* HAL_RCC_DeInit/HAL_DeInit may disable the SYSCFG clock. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    /* Invalidate both caches before changing which bank backs application addresses. */
    FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR |= FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH->ACR &= ~(FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    SCB->VTOR = APP_SLOT0_VECTOR_ADDR;
    __DSB();
    __ISB();
    enter_app(sp, pc, remap);
}

static int send_reply(CAN_HandleTypeDef *can, uint8_t reply[8])
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox, start = HAL_GetTick();
    tx.StdId = BOOT_CAN_RESPONSE_ID;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    if (HAL_CAN_AddTxMessage(can, &tx, reply, &mailbox) != HAL_OK) {
        boot_diag_error(ERROR_CAN_QUEUE, 0, SOURCE_CAN, HAL_CAN_GetError(can));
        BOOT_LOG_ERR("CAN reply enqueue failed: HAL error 0x%08lx", (unsigned long)HAL_CAN_GetError(can));
        return -1;
    }
    while (HAL_CAN_IsTxMessagePending(can, mailbox)) {
        if (HAL_GetTick() - start > 100) {
            boot_diag_error(ERROR_CAN_TIMEOUT, 0, SOURCE_CAN, HAL_CAN_GetError(can));
            BOOT_LOG_ERR("CAN reply timeout: command 0x%02x, HAL error 0x%08lx",
                         reply[0], (unsigned long)HAL_CAN_GetError(can));
            HAL_CAN_AbortTxRequest(can, mailbox);
            return -1;
        }
    }
    return 0;
}

/* Best effort: reserve a mailbox for replies and never wait for the heartbeat. */
/* shortcut: blocking flash operations delay heartbeats; use a RAM timer path if strict timing is needed. */
static void send_heartbeat(CAN_HandleTypeDef *can, uint32_t *last)
{
    uint32_t now = HAL_GetTick();
    if (now - *last < 500U) return;
    *last = now;
    if (HAL_CAN_GetTxMailboxesFreeLevel(can) <= 1U) return;
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;
    uint8_t data[8];
    const struct bootloader_bci_boot_heartbeat_t message = {
        0x55U, now, BOOT_VERSION_MAJOR, BOOT_VERSION_MINOR, BOOT_VERSION_PATCH
    };
    bootloader_bci_boot_heartbeat_pack(data, &message, sizeof data);
    tx.StdId = BOOT_CAN_HEARTBEAT_ID;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = sizeof data;
    (void)HAL_CAN_AddTxMessage(can, &tx, data, &mailbox);
}

void boot_run(CAN_HandleTypeDef *can)
{
    uint32_t heartbeat_time;
    struct boot_rsp rsp = {0};
    printf("\r\nCoreWave A/B CAN bootloader\r\n");
    boot_diagnostics.reset_flags = RCC->CSR;
    boot_diagnostics.current_command = 0xff;
    BOOT_LOG_INF("Reset flags RCC_CSR=0x%08lx", (unsigned long)boot_diagnostics.reset_flags);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    int flash_ready = boot_flash_init() == 0;
    int valid = flash_ready && boot_select(&rsp) == 0;

    /* This PCB B1 is active-low: pressed reads GPIO_PIN_RESET.
     * The RAM magic is consumed once; B1 is re-sampled live below so that
     * releasing the button boots a valid application without another reset. */
    int stay_magic = BootloaderRequested();
    boot_diagnostics.entry_reason = (stay_magic ? ENTRY_RAM : 0) |
        (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET ? ENTRY_BUTTON : 0) |
        (!valid ? ENTRY_RECOVERY : 0) |
        (boot_diagnostics.rejected_bank && valid ? ENTRY_ROLLBACK : 0);
    BOOT_LOG_INF("Boot request: RAM magic=%u", stay_magic);
    if (valid && (stay_magic || HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET))
        BOOT_LOG_INF("Application held: %s", stay_magic ? "RAM bootloader request" : "B1 pressed");
    CAN_FilterTypeDef filter = {0};
    filter.FilterIdHigh = BOOT_CAN_REQUEST_ID << 5;
    filter.FilterMaskIdHigh = 0x7ffU << 5;
    filter.FilterMaskIdLow = 6; /* Match standard data frames only. */
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK || HAL_CAN_Start(can) != HAL_OK) {
        BOOT_LOG_ERR("CAN start/filter failed: HAL error 0x%08lx", (unsigned long)HAL_CAN_GetError(can));
        Error_Handler();
    }
    BOOT_LOG_INF("CAN ready: 250 kbit/s, RX=0x%03x TX=0x%03x", BOOT_CAN_REQUEST_ID, BOOT_CAN_RESPONSE_ID);
    diagnostic_can = can;
    boot_diag_snapshot();
    uint32_t activity = HAL_GetTick();
    heartbeat_time = activity - 500U;
    send_heartbeat(can, &heartbeat_time);
    boot_diag_flush();
    if (!flash_ready) Error_Handler();
    for (;;) {
        if (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0)) {
            CAN_RxHeaderTypeDef rx;
            uint8_t data[8], reply[8];
            if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &rx, data) == HAL_OK &&
                rx.IDE == CAN_ID_STD && rx.RTR == CAN_RTR_DATA && rx.StdId == BOOT_CAN_REQUEST_ID) {
                boot_update_frame(data, rx.DLC, reply);
                int sent = send_reply(can, reply);
                if (reply[1] == UPDATE_OK) {
                    activity = HAL_GetTick();
                    if (data[0] == UPDATE_INFO) {
                        heartbeat_time = activity - 500U;
                        send_heartbeat(can, &heartbeat_time);
                    }
                    if (data[0] == UPDATE_RESET && sent == 0) {
                        boot_diag_flush();
                        BOOT_LOG_INF("Reset requested by CAN uploader");
                        fflush(stdout);
                        NVIC_SystemReset();
                    }
                }
            }
        }
        if (boot_update_active()) {
            if (HAL_GetTick() - activity > 30000U) {
                boot_diag_error(ERROR_INACTIVITY, boot_diagnostics.upload_bank, SOURCE_BOOT, boot_diagnostics.received);
                boot_diag_flush();
                BOOT_LOG_WRN("Upload inactivity timeout (30 s); resetting");
                fflush(stdout);
                NVIC_SystemReset();
            }
        } else if (valid && !stay_magic &&
                   HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET) {
            jump(can, &rsp);
            valid = 0;
        }
        /* Slow blink: valid app held by B1/magic. Fast blink: no bootable
         * image (recovery mode) or handoff rejected the vectors. */
        uint32_t period = valid ? 250U : 100U;
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                          (HAL_GetTick() / period) & 1U ? GPIO_PIN_SET : GPIO_PIN_RESET);

        send_heartbeat(can, &heartbeat_time);
        send_diagnostic(can);
    }
}
