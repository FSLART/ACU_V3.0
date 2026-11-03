/*
 * ble_handler.c
 *
 *  Created on: May 25, 2026
 *      Author: Bruno Vicente
 *
 * What the phone sees:
 *   - one line as soon as a key state changes, e.g.
 *       [12.345 s] Modo IDLE->AUTONOMOUS | Arranque NOT_STARTED->TANK_PRESSURE
 *   - a status block every second,
 *   - the fault log on "flush".
 * Commands from the phone: "stop", "start", "flush" (each ending in \r\n).
 */
#include "ble_handler.h"
#include "acu.h"
#include "faults.h"
#include "logger.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define STATUS_PERIOD_MS  1000
#define TX_STUCK_MS       500

/* ── RN4871 configuration ─────────────────────────────────────────────────── */

/* The RN4871 powers up with the ACU and ignores "$$$" until it has booted. */
#define BLE_BOOT_DELAY_MS 1000
#define BLE_CMD_WAIT_MS   500

static const char *const ble_config_commands[] = {
	"SN," BLE_DEVICE_NAME "\r\n",
	"SS,C0\r\n", /* Device Information + Transparent UART service, which BLE terminal apps look for */
	"SR,01\r\n",
	"R,1\r\n",   /* settings take effect after a reboot */
};
#define BLE_CONFIG_COMMAND_COUNT (sizeof(ble_config_commands) / sizeof(ble_config_commands[0]))

typedef enum {
	MODULE_IDLE,
	MODULE_WAIT_BOOT,       /* send $$$ once the module has booted */
	MODULE_WAIT_CMD_PROMPT, /* CMD> prompt after $$$ */
	MODULE_SEND_COMMAND,
	MODULE_WAIT_COMMAND,    /* AOK after each command */
	MODULE_DONE
} module_config_step_t;

static module_config_step_t module_step = MODULE_IDLE;
static uint8_t module_command = 0;
static uint32_t module_step_since_ms = 0;

static void module_go_to(module_config_step_t step, uint32_t now_ms) {
	module_step = step;
	module_step_since_ms = now_ms;
	acu.ble.module_config_step = (uint8_t)step;
}

static void module_config_tick(uint32_t now_ms) {
	switch (module_step) {
	case MODULE_IDLE:
	case MODULE_DONE:
		break;
	case MODULE_WAIT_BOOT:
		if (time_elapsed(module_step_since_ms, now_ms, BLE_BOOT_DELAY_MS)) {
			HAL_UART_Transmit(BLE_UART, (uint8_t *)"$$$", 3, 10);
			module_go_to(MODULE_WAIT_CMD_PROMPT, now_ms);
		}
		break;
	case MODULE_WAIT_CMD_PROMPT:
		if (time_elapsed(module_step_since_ms, now_ms, BLE_CMD_WAIT_MS)) {
			module_command = 0;
			module_go_to(MODULE_SEND_COMMAND, now_ms);
		}
		break;
	case MODULE_SEND_COMMAND:
		if (module_command < BLE_CONFIG_COMMAND_COUNT) {
			const char *command = ble_config_commands[module_command];
			HAL_UART_Transmit(BLE_UART, (uint8_t *)command, (uint16_t)strlen(command), 10);
			module_go_to(MODULE_WAIT_COMMAND, now_ms);
		} else {
			module_go_to(MODULE_DONE, now_ms);
		}
		break;
	case MODULE_WAIT_COMMAND:
		if (time_elapsed(module_step_since_ms, now_ms, BLE_CMD_WAIT_MS)) {
			module_command++;
			module_go_to(MODULE_SEND_COMMAND, now_ms);
		}
		break;
	default:
		module_go_to(MODULE_DONE, now_ms);
		break;
	}
}

/* ── TX: one DMA transfer at a time, never blocks ─────────────────────────── */

static uint8_t dma_rx[BLE_RX_BUF_SIZE];
static uint16_t rx_head = 0;        /* our read position in the circular buffer */
static char ble_tx_buf[BLE_TX_BUF_SIZE];
static size_t tx_len = 0;
volatile uint8_t ble_tx_busy = 0;   /* set here, cleared by the TX complete interrupt */
static uint32_t ble_tx_since_ms = 0;

static bool tx_begin(uint32_t now_ms) {
	if (ble_tx_busy) {
		return false;
	}
	ble_tx_busy = 1;
	ble_tx_since_ms = now_ms;
	tx_len = 0;
	return true;
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
	acu.ble.lines_sent++;
	if (HAL_UART_Transmit_DMA(BLE_UART, (uint8_t *)ble_tx_buf, (uint16_t)tx_len) != HAL_OK) {
		ble_tx_busy = 0;
	}
}

static void tx_time(uint32_t ms) {
	tx_printf("[%lu.%03lu s]", (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
}

/* ── RX: commands from the phone in the circular DMA buffer ───────────────── */

static uint16_t rx_write_position(void) {
	return (uint16_t)(BLE_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(BLE_UART->hdmarx));
}

/* Returns true if needle is in the unread bytes, and consumes them. */
static bool rx_contains(const char *needle) {
	uint16_t write_head = rx_write_position();
	char unread[BLE_RX_BUF_SIZE + 1];
	uint16_t n = 0;

	for (uint16_t i = rx_head; i != write_head; i = (uint16_t)((i + 1) % BLE_RX_BUF_SIZE)) {
		unread[n++] = (char)dma_rx[i];
	}
	unread[n] = '\0';
	if (strstr(unread, needle)) {
		rx_head = write_head;
		return true;
	}
	return false;
}

/* What the RN4871 answered during configuration: "CMD>" after $$$, "AOK" or "Err"
 * per command, "Rebooting". Empty or garbage: the module is not talking at 115200 baud. */
char ble_module_reply[128];

static void save_module_reply(void) {
	uint16_t write_head = rx_write_position();
	size_t n = 0;

	while (rx_head != write_head && n < sizeof(ble_module_reply) - 1) {
		char c = (char)dma_rx[rx_head];
		ble_module_reply[n++] = (c >= ' ' && c <= '~') ? c : ' ';
		rx_head = (uint16_t)((rx_head + 1) % BLE_RX_BUF_SIZE);
	}
	ble_module_reply[n] = '\0';
	rx_head = write_head;
}

/* ── Event lines ──────────────────────────────────────────────────────────── */

typedef struct {
	acu_mode_t mode;
	AS_STATE_t as_state;
	checkup_step_t checkup;
	finish_step_t finish;
	mission_t mission;
	uint8_t jetson_mission;
	fault_reason_t fault;
	bool ts_active;
	bool sdc_closed;
	bool asms_on;
} ble_snapshot_t;

static ble_snapshot_t last_sent;

static ble_snapshot_t take_snapshot(void) {
	ble_snapshot_t s = {
		.mode = acu.mode,
		.as_state = acu.as.state,
		.checkup = acu.checkup.step,
		.finish = acu.as.finish,
		.mission = acu.mission.selected,
		.jetson_mission = acu.jetson.mission,
		.fault = acu.fault.reason,
		.ts_active = acu.vcu.ts_active,
		.sdc_closed = acu.inputs.sdc_closed,
		.asms_on = acu.inputs.asms_on,
	};
	return s;
}

/* One line with every key state that changed since the last line. Changes that happen
 * while the previous text is still being sent are merged into the next line. */
static void send_changes(uint32_t now_ms) {
	ble_snapshot_t now = take_snapshot();
	const char *sep = " ";

	if (memcmp(&now, &last_sent, sizeof(now)) == 0 || !tx_begin(now_ms)) {
		return;
	}
	tx_time(now_ms);

#define CHANGED(field, label, name_of) \
	if (now.field != last_sent.field) { \
		tx_printf("%s%s %s->%s", sep, label, name_of(last_sent.field), name_of(now.field)); \
		sep = " | "; \
	}
#define CHANGED_ON_OFF(field, label, on, off) \
	if (now.field != last_sent.field) { \
		tx_printf("%s%s %s", sep, label, now.field ? on : off); \
		sep = " | "; \
	}

	CHANGED(mode, "Modo", acu_mode_name)
	CHANGED(as_state, "AS", as_state_name)
	CHANGED(checkup, "Arranque", checkup_step_name)
	CHANGED(finish, "Fim", finish_step_name)
	CHANGED(mission, "Missao", mission_name)
	CHANGED(jetson_mission, "Missao Jetson", mission_name)
	CHANGED(fault, "Falha", fault_name)
	CHANGED_ON_OFF(ts_active, "HV", "ON", "OFF")
	CHANGED_ON_OFF(sdc_closed, "SDC", "fechado", "ABERTO")
	CHANGED_ON_OFF(asms_on, "ASMS", "ON", "OFF")

#undef CHANGED
#undef CHANGED_ON_OFF

	if (now.fault != FAULT_NONE && now.fault != last_sent.fault) {
		tx_printf("\r\n  -> %s", fault_description(now.fault));
	}
	tx_printf("\r\n");
	tx_send();
	last_sent = now;
}

/* ── Status block ─────────────────────────────────────────────────────────── */

static const char *on_off(bool on) {
	return on ? "ON" : "OFF";
}

static void print_test_flags(void) {
#define WARN_IF_SET(flag) \
	if (flag) { \
		tx_printf("!! TESTE: " #flag " = 1 !!\r\n"); \
	}
	WARN_IF_SET(SKIP_WDT_CHECK)
	WARN_IF_SET(SKIP_PNEUMATIC_CHECK)
	WARN_IF_SET(SKIP_PRESSURE_CHECK1)
	WARN_IF_SET(SKIP_IGNITION_CHECK)
	WARN_IF_SET(SKIP_PRESSURE_FRONT_CHECK)
	WARN_IF_SET(SKIP_PRESSURE_REAR_CHECK)
	WARN_IF_SET(SKIP_PRESSURE_CHECK2)
	WARN_IF_SET(BYPASS_FRONT_HYD_PRESSURE)
	WARN_IF_SET(BYPASS_REAR_HYD_PRESSURE)
	WARN_IF_SET(BYPASS_JETSON_TIMEOUT)
#undef WARN_IF_SET
}

static bool send_status(uint32_t now_ms) {
	const CAN_BusStatus *can = acu.can_bus;

	if (!tx_begin(now_ms)) {
		return false;
	}
	tx_printf("\r\n== ACU %lu.%lu s ==\r\n", (unsigned long)(now_ms / 1000), (unsigned long)((now_ms % 1000) / 100));
	tx_printf("Modo: %s   AS: %s\r\n", acu_mode_name(acu.mode), as_state_name(acu.as.state));
	tx_printf("Arranque: %s   Fim: %s\r\n", checkup_step_name(acu.checkup.step), finish_step_name(acu.as.finish));
	tx_printf("Missao: %s (Jetson %s, pede %s)\r\n", mission_name(acu.mission.selected),
			mission_name((mission_t)acu.jetson.mission), as_state_name((AS_STATE_t)acu.jetson.as_state));
	tx_printf("Falha: %s\r\n", fault_name(acu.fault.reason));
	tx_printf("HV: %s   SDC: %s   ASMS: %s   WDT: %s\r\n", on_off(acu.vcu.ts_active),
			acu.inputs.sdc_closed ? "fechado" : "ABERTO", on_off(acu.inputs.asms_on),
			on_off(acu.outputs.watchdog_pulses_on));
	tx_printf("EBS frente/tras: %s / %s\r\n", acu.outputs.front_ebs_released ? "solto" : "travado",
			acu.outputs.rear_ebs_released ? "solto" : "travado");
	tx_printf("Hidr frente/tras: %.1f / %.1f bar\r\n", (double)acu.brakes.front_bar, (double)acu.brakes.rear_bar);
	tx_printf("Tanque frente/tras: %.2f / %.2f bar\r\n", (double)acu.inputs.front_tank_bar,
			(double)acu.inputs.rear_tank_bar);
	tx_printf("RPM: %u / %u (%s)\r\n", acu.vcu.motor_rpm_left, acu.vcu.motor_rpm_right,
			!acu.as.speed_known ? "sem dados" : (acu.as.moving ? "a mexer" : "parado"));
	if (can) {
		tx_printf("CAN: %s  TEC %u REC %u  bus-off %lu  erro %s\r\n", can->state_text ? can->state_text : "?",
				can->tec, can->rec, (unsigned long)can->bus_off, can->last_error_text ? can->last_error_text : "?");
	}
	tx_printf("Idade ms: VCU %lu Jetson %lu RES %lu RPM %lu AQT1 %lu AQT7 %lu Dir %lu\r\n",
			(unsigned long)acu.nodes.vcu.age_ms, (unsigned long)acu.nodes.jetson.age_ms,
			(unsigned long)acu.nodes.res.age_ms, (unsigned long)acu.nodes.vcu_rpm.age_ms,
			(unsigned long)acu.nodes.brake_front.age_ms, (unsigned long)acu.nodes.brake_rear.age_ms,
			(unsigned long)acu.nodes.steering.age_ms);
	tx_printf("Ign ACU->VCU: pedido %u enable %u   VCU estado %u R2D-ok %s\r\n", acu.outputs.ignition_request,
			acu.outputs.ignition_enable, acu.vcu.vcu_state, acu.as.flowchart.r2d ? "sim" : "nao");
	tx_printf("RES: %s%s (signal %u)\r\n", acu.res.stop_pressed ? "STOP" : "a andar", acu.res.go_pressed ? " + GO" : "",
			acu.res.signal);
	if (acu.fault.count) {
		tx_printf("Ultima falha (%lu no total): %s\r\n", (unsigned long)acu.fault.count,
				fault_description(acu.fault.last_reason));
	}
	tx_printf("Log: %u registos, EEPROM %s   Reset: %s\r\n", logger_record_count(), acu.log.eeprom_ok ? "ok" : "FALHOU",
			acu.system.reset_cause ? acu.system.reset_cause : "?");
	tx_printf("Stack livre: ctl %lu tel %lu log %lu\r\n", (unsigned long)acu.system.stack_free_control,
			(unsigned long)acu.system.stack_free_telemetry, (unsigned long)acu.system.stack_free_logger);
	print_test_flags();
	tx_send();
	return true;
}

/* ── Fault log ─────────────────────────────────────────────────────────────── */

static bool send_record(uint8_t index, uint32_t now_ms) {
	const log_record_t *r = logger_record(index);

	if (r == NULL || !tx_begin(now_ms)) {
		return r == NULL;
	}
	tx_printf("#%u boot %u ", index, r->boot_number);
	tx_time(r->uptime_ms);
	if (r->event == LOG_EVENT_BOOT) {
		tx_printf(" ARRANQUE (RCC_CSR 0x%08lX)\r\n", (unsigned long)r->detail);
	} else {
		tx_printf(" %s: %s\r\n", fault_name((fault_reason_t)r->reason), fault_description((fault_reason_t)r->reason));
		if (r->detected_as != r->reason) {
			tx_printf("   (visto primeiro como %s)\r\n", fault_name((fault_reason_t)r->detected_as));
		}
		tx_printf("   modo %s AS %s arranque %s missao %s\r\n", acu_mode_name((acu_mode_t)r->mode),
				as_state_name((AS_STATE_t)r->as_state), checkup_step_name((checkup_step_t)r->checkup_step),
				mission_name((mission_t)r->mission));
		tx_printf("   tanques %.2f/%.2f hidr %.1f/%.1f SDC %u ASMS %u HV %u detalhe 0x%08lX\r\n",
				(double)r->front_tank_bar, (double)r->rear_tank_bar, (double)r->front_brake_bar,
				(double)r->rear_brake_bar, r->sdc_closed, r->asms_on, r->ts_active, (unsigned long)r->detail);
	}
	tx_send();
	return true;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

typedef enum {
	BLE_WAIT_MODULE_CONFIG,
	BLE_RUNNING,
	BLE_FLUSHING_LOG
} ble_state_t;

static ble_state_t ble_state = BLE_WAIT_MODULE_CONFIG;
static uint8_t flush_index = 0;
static uint32_t status_since_ms = 0;

void ble_handler_init(uint32_t now_ms) {
	/* Circular RX DMA runs forever, never needs restarting. */
	HAL_UART_Receive_DMA(BLE_UART, dma_rx, BLE_RX_BUF_SIZE);
	module_go_to(MODULE_WAIT_BOOT, now_ms);
	ble_state = BLE_WAIT_MODULE_CONFIG;
}

static void handle_commands(uint32_t now_ms) {
	if (rx_contains("stop\r\n")) {
		acu.ble.paused = true;
	} else if (rx_contains("start\r\n")) {
		acu.ble.paused = false;
	} else if (rx_contains("flush\r\n") && tx_begin(now_ms)) {
		tx_printf("=== LOG DE FALHAS (%u) ===\r\n", logger_record_count());
		tx_send();
		flush_index = 0;
		ble_state = BLE_FLUSHING_LOG;
	}
}

void ble_handler(uint32_t now_ms) {
	module_config_tick(now_ms);

	if (ble_tx_busy && time_elapsed(ble_tx_since_ms, now_ms, TX_STUCK_MS)) {
		HAL_UART_AbortTransmit(BLE_UART); /* TX only, the circular RX DMA keeps running */
		ble_tx_busy = 0;
	}

	switch (ble_state) {
	case BLE_WAIT_MODULE_CONFIG:
		if (module_step == MODULE_DONE && tx_begin(now_ms)) {
			save_module_reply();
			last_sent = take_snapshot();
			tx_printf("\r\n" BLE_DEVICE_NAME " ligada\r\nRN4871: %s\r\n", ble_module_reply);
			tx_send();
			ble_state = BLE_RUNNING;
		}
		break;

	case BLE_RUNNING:
		handle_commands(now_ms);
		if (ble_state == BLE_RUNNING && !acu.ble.paused) {
			send_changes(now_ms);
			if (time_elapsed(status_since_ms, now_ms, STATUS_PERIOD_MS) && send_status(now_ms)) {
				status_since_ms = now_ms;
			}
		}
		break;

	case BLE_FLUSHING_LOG:
		if (flush_index >= logger_record_count()) {
			if (tx_begin(now_ms)) {
				tx_printf("=== FIM DO LOG ===\r\n");
				tx_send();
				ble_state = BLE_RUNNING;
			}
		} else if (send_record(flush_index, now_ms)) {
			flush_index++;
		}
		break;

	default:
		ble_state = BLE_RUNNING;
		break;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == BLE_UART) {
		ble_tx_busy = 0;
	}
}
