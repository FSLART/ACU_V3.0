/*
 * ble_handler.c
 *
 *  Created on: May 25, 2026
 *      Author: Bruno Vicente
 *
 * Plain-text telemetry over the RN4871 (transparent UART on BLE_UART), readable
 * in any BLE serial terminal on a phone:
 *   - one line as soon as a key state changes, e.g.
 *       [12.345 s] Estado IDLE->AS_ON | Arranque WDT_TOGGLE_CHECK->HV_ACTIVATION
 *   - a status block every second.
 * Commands from the phone: "stop", "start", "flush" (each ending in \r\n).
 */

#include "ble_handler.h"
#include "APP.h"
#include "state_machine.h"
#include "hardware_abstraction.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern cant_acu_state_t ACU_STATE;

#define STATUS_PERIOD_MS  1000
#define TX_STUCK_MS       500
#define MAX_EVENTS        20      // must match your EEPROM layout

static uint8_t  dma_rx[BLE_RX_BUF_SIZE];
static uint16_t rx_head = 0;        // our read position in the circular buffer

static char     ble_tx_buf[BLE_TX_BUF_SIZE];
static size_t   tx_len = 0;
volatile uint8_t ble_tx_busy = 0;   // set here, cleared by the TX complete interrupt
static uint32_t ble_tx_tick = 0;

static BLE_STATE_MACHINE_t ble_state = BLE_IDLE;
static uint8_t  logs_paused = 0;
static uint8_t  flushing    = 0;
static uint8_t  flush_index = 0;
static uint32_t status_tick = 0;

/* ── Names shown on the phone ──────────────────────────────────────────── */

#define NAME(x) [x] = #x
#define NAME_OF(table, v) (((unsigned)(v) < sizeof(table) / sizeof((table)[0]) && (table)[v]) ? (table)[v] : "?")

static const char *const vehicle_names[] = { NAME(Start), NAME(IDLE), NAME(AS_ON), NAME(EMERGENCY) };
static const char *const autonomous_names[] = {
    NAME(OFF), NAME(Initial_Sequence), NAME(Monitor_sequence), NAME(AS_Emergency), NAME(Finish) };
static const char *const acu_names[] = {
    NAME(INIT), NAME(MISSION_SELECT), NAME(JETSON_WAITING), NAME(INIT_SEQUENCE), NAME(READY),
    NAME(DRIVING), NAME(EBS_ERROR), NAME(ACU_EMERGENCY), NAME(FINISHED), NAME(ACU_MANUAL) };
static const char *const sequence_names[] = {
    NAME(WDT_TOGGLE_CHECK), NAME(WDT_STP_TOGGLE_CHECK), NAME(PNEUMATIC_CHECK), NAME(PRESSURE_CHECK1),
    NAME(HV_ACTIVATION), NAME(PRESSURE_CHECK_FRONT), NAME(PRESSURE_CHECK_REAR), NAME(PRESSURE_CHECK2),
    NAME(SEQUENCE_ERROR) };
static const char *const as_names[] = {
    [AS_STATE_OFF] = "OFF", [AS_STATE_READY] = "READY", [AS_STATE_DRIVING] = "DRIVING",
    [AS_STATE_EMERGENCY] = "EMERGENCY", [AS_STATE_FINISHED] = "FINISHED" };
static const char *const mission_names[] = {
    NAME(MANUAL), NAME(ACCELERATION), NAME(SKIDPAD), NAME(TRACKDRIVE), NAME(EBS_TEST),
    NAME(INSPECTION), NAME(AUTOCROSS) };
static const char *const cause_names[] = {
    NAME(NONE), NAME(SDC_OPEN), NAME(RES), NAME(Pressure_checks), NAME(VCU_Timeout),
    NAME(Jetson_timeout), NAME(ACU_WDT_TRIGERED), NAME(dir_actuator_timeout),
    NAME(Dynamics_REAR_Pressure_timeout), NAME(UNKOWN) };

/* ── TX: one DMA transfer at a time, never blocks ─────────────────────── */

static uint8_t tx_begin(void) {
    if (ble_tx_busy) {
        return 0;
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    tx_len = 0;
    return 1;
}

static void tx_printf(const char *fmt, ...) {
    va_list args;
    int n;

    if (tx_len >= sizeof(ble_tx_buf) - 1) {
        return;
    }
    va_start(args, fmt);
    n = vsnprintf(ble_tx_buf + tx_len, sizeof(ble_tx_buf) - tx_len, fmt, args);
    va_end(args);
    if (n > 0) {
        tx_len += (size_t)n;
    }
    if (tx_len > sizeof(ble_tx_buf) - 1) {
        tx_len = sizeof(ble_tx_buf) - 1;
    }
}

static void tx_send(void) {
    if (HAL_UART_Transmit_DMA(BLE_UART, (uint8_t *)ble_tx_buf, tx_len) != HAL_OK) {
        ble_tx_busy = 0;
    }
}

static void ble_send_text(const char *s) {
    if (tx_begin()) {
        tx_printf("%s", s);
        tx_send();
    }
}

/* ── RX: commands from the phone in the circular DMA buffer ───────────── */

/* Returns 1 if needle is in the unread bytes, and consumes them. */
static uint8_t rx_contains(const char *needle) {
    uint16_t write_head = BLE_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(BLE_UART->hdmarx);
    char tmp[BLE_RX_BUF_SIZE + 1];
    uint16_t n = 0, i = rx_head;

    while (i != write_head) {
        tmp[n++] = dma_rx[i];
        i = (i + 1) % BLE_RX_BUF_SIZE;
    }
    tmp[n] = '\0';

    if (strstr(tmp, needle)) {
        rx_head = write_head;
        return 1;
    }
    return 0;
}

/* What the RN4871 answered during configuration: "CMD>" after $$$, "AOK" or "Err"
 * per command, "Rebooting". Empty or garbage: the module is not talking at 115200 baud. */
char ble_module_reply[128];

static void save_module_reply(void) {
    uint16_t write_head = BLE_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(BLE_UART->hdmarx);
    size_t n = 0;

    while (rx_head != write_head && n < sizeof(ble_module_reply) - 1) {
        char c = (char)dma_rx[rx_head];
        ble_module_reply[n++] = (c >= ' ' && c <= '~') ? c : ' ';
        rx_head = (rx_head + 1) % BLE_RX_BUF_SIZE;
    }
    ble_module_reply[n] = '\0';
    rx_head = write_head;
}

/* ── What the phone sees ───────────────────────────────────────────────── */

typedef struct {
    uint8_t vehicle, autonomous, acu, sequence, as, mission, jetson_mission, cause, hv, sdc, asms;
} ble_snapshot_t;

static ble_snapshot_t last_sent;

static ble_snapshot_t take_snapshot(void) {
    ble_snapshot_t s = {
        .vehicle = Vehicle_state_machine,
        .autonomous = Autonomous_state,
        .acu = ACU_STATE,
        .sequence = startup_sequence_state,
        .as = t24.Autonomous_State,
        .mission = t24.Current_Mission,
        .jetson_mission = t24.Jetson_mission,
        .cause = Emergency_cause,
        .hv = t24.Ignition_Status,
        .sdc = t24.SDC_feedback,
        .asms = t24.ASMS,
    };
    return s;
}

static void tx_time(void) {
    uint32_t ms = HAL_GetTick();
    tx_printf("[%lu.%03lu s]", (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
}

/* One line with every key state that changed since the last line. Changes that
 * happen while the previous text is still being sent are merged into the next line. */
static void send_changes(void) {
    ble_snapshot_t now = take_snapshot();
    const char *sep = " ";

    if (memcmp(&now, &last_sent, sizeof(now)) == 0 || !tx_begin()) {
        return;
    }
    tx_time();

#define CHANGED(field, label, table) \
    if (now.field != last_sent.field) { \
        tx_printf("%s%s %s->%s", sep, label, NAME_OF(table, last_sent.field), NAME_OF(table, now.field)); \
        sep = " | "; \
    }
#define CHANGED_ON_OFF(field, label, on, off) \
    if (now.field != last_sent.field) { \
        tx_printf("%s%s %s", sep, label, now.field ? on : off); \
        sep = " | "; \
    }

    CHANGED(vehicle, "Estado", vehicle_names)
    CHANGED(autonomous, "Auto", autonomous_names)
    CHANGED(sequence, "Arranque", sequence_names)
    CHANGED(acu, "ACU", acu_names)
    CHANGED(as, "AS", as_names)
    CHANGED(mission, "Missao", mission_names)
    CHANGED(jetson_mission, "Missao Jetson", mission_names)
    CHANGED(cause, "Emergencia", cause_names)
    CHANGED_ON_OFF(hv, "HV", "ON", "OFF")
    CHANGED_ON_OFF(sdc, "SDC", "fechado", "ABERTO")
    CHANGED_ON_OFF(asms, "ASMS", "ON", "OFF")

    tx_printf("\r\n");
    tx_send();
    last_sent = now;
}

static uint8_t send_status(void) {
    uint32_t ms = HAL_GetTick();

    if (!tx_begin()) {
        return 0;
    }
    tx_printf("\r\n== ACU %lu.%lu s ==\r\n", (unsigned long)(ms / 1000), (unsigned long)((ms % 1000) / 100));
    tx_printf("Estado:   %s / %s\r\n", NAME_OF(vehicle_names, Vehicle_state_machine),
              NAME_OF(autonomous_names, Autonomous_state));
    tx_printf("Arranque: %s\r\n", NAME_OF(sequence_names, startup_sequence_state));
    tx_printf("ACU: %s   AS: %s\r\n", NAME_OF(acu_names, ACU_STATE), NAME_OF(as_names, t24.Autonomous_State));
    tx_printf("Missao:   %s (Jetson %s)\r\n", NAME_OF(mission_names, t24.Current_Mission),
              NAME_OF(mission_names, t24.Jetson_mission));
    tx_printf("Emergencia: %s\r\n", NAME_OF(cause_names, Emergency_cause));
    tx_printf("HV: %s   SDC: %s   ASMS: %s\r\n", t24.Ignition_Status ? "ON" : "OFF",
              t24.SDC_feedback ? "fechado" : "ABERTO", t24.ASMS ? "ON" : "OFF");
    tx_printf("EBS frente/tras: %s / %s\r\n", t24.front_solenoid ? "solto" : "travado",
              t24.rear_solenoid ? "solto" : "travado");
    tx_printf("Hidr frente/tras: %.1f / %.1f bar\r\n", (double)t24.Front_Pressure.Hydraulic,
              (double)t24.Rear_Pressure.Hydraulic);
    tx_printf("Pneu frente/tras: %.2f / %.2f bar\r\n", (double)t24.Front_Pressure.Pneumatic,
              (double)t24.Rear_Pressure.Pneumatic);
    tx_send();
    return 1;
}

/* ── EEPROM log flush ──────────────────────────────────────────────────── */

/* Format one EEPROM event record as text. Called once per main loop tick while flushing. */
static void flush_one_record(uint8_t index) {
    /* TODO: read EventRecord from EEPROM at:
       address = LOG_BASE + index * RECORD_SIZE
       e.g. eeprom_read(LOG_BASE + index * RECORD_SIZE, &rec, sizeof(rec)); */

    /* TODO: validate record:
       if (rec.magic != 0xA5 || !crc_valid(&rec)) → skip, send "EMPTY\r\n" */

    /* TODO: format the record with tx_printf, example layout:
       BOOT:%u UPTIME:%lu [TS:%lu]\r\n
       EVENT:%s SEV:%u\r\n
       PC:0x%08lX LR:0x%08lX\r\n
       CFSR:0x%08lX HFSR:0x%08lX\r\n
       STATE:%u SDC:0x%02X\r\n
       CRC:0x%04X\r\n
       ---\r\n  */

    if (!tx_begin()) {
        return;                     /* drop this record, next tick will retry */
    }
    tx_printf("SLOT:%u (EEPROM not yet implemented)\r\n---\r\n", index);
    tx_send();
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void ble_handler_init(void) {
    /* Circular RX DMA runs forever, never needs restarting. */
    HAL_UART_Receive_DMA(BLE_UART, dma_rx, BLE_RX_BUF_SIZE);
    ble_state = BLE_WAIT_CONFIG;
}

void ble_handler(void) {
    switch (ble_state) {

    case BLE_WAIT_CONFIG:
        if (ble_module_config_is_done() == 5) {
            save_module_reply();
            last_sent = take_snapshot();
            if (tx_begin()) {
                tx_printf("\r\n" BLE_DEVICE_NAME " ligada\r\nRN4871: %s\r\n", ble_module_reply);
                tx_send();
            }
            ble_state = BLE_BRIDGE;
        }
        break;

    case BLE_BRIDGE:
        if (ble_tx_busy && (HAL_GetTick() - ble_tx_tick > TX_STUCK_MS)) {
            HAL_UART_AbortTransmit(BLE_UART);  /* TX only, the circular RX DMA keeps running */
            ble_tx_busy = 0;
        }

        if (rx_contains("stop\r\n")) {
            logs_paused = 1;
        } else if (rx_contains("start\r\n")) {
            logs_paused = 0;
            flushing    = 0;
        } else if (rx_contains("flush\r\n")) {
            logs_paused  = 1;
            flushing     = 1;
            flush_index  = 0;
            ble_send_text("=== EVENT LOG ===\r\n");
        }

        if (flushing) {
            if (flush_index < MAX_EVENTS) {
                flush_one_record(flush_index);
                flush_index++;
            } else {
                ble_send_text("=== END LOG ===\r\n");
                flushing    = 0;
                logs_paused = 0;
            }
            break;
        }

        if (!logs_paused) {
            send_changes();
            if (HAL_GetTick() - status_tick >= STATUS_PERIOD_MS && send_status()) {
                status_tick = HAL_GetTick();
            }
        }
        break;

    default:
        ble_state = BLE_BRIDGE;
        break;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == BLE_UART) {
        ble_tx_busy = 0;
    }
}
