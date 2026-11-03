/*
 * PC test harness for the ACU logic.
 *
 * build.py compiles the real, RTOS-free firmware modules (APP.c, acu_mode.c, checkup.c,
 * as_state.c, monitoring.c, outputs.c, can_rx.c, can_tx.c, logger.c, ble_handler.c, ...)
 * against stub/stm32f4xx_hal.h. This file stands in for everything else:
 *   - the HAL (GPIO pins, tick, USART1 to the RN4871, CAN TX, EEPROM),
 *   - the FreeRTOS tasks (control every 1 ms, telemetry every 10 ms, logger every 20 ms),
 *   - a model of the car around the ACU (sim_plant): external watchdog + SDC, VCU and TS,
 *     EBS brakes, and the CAN nodes (VCU, Jetson, RES, brake sensors, steering).
 * acu_sim.py drives it through the sim_* functions.
 */
#include "APP.h"
#include "acu.h"
#include "acu_system.h"
#include "as_state.h"
#include "autonomous_t26.h"
#include "ble_handler.h"
#include "can_rx.h"
#include "ebs.h"
#include "ee24.h"
#include "inputs.h"
#include "logger.h"

#define SIM_API __declspec(dllexport)

GPIO_TypeDef sim_gpio_ports[8] = { {0}, {1}, {2}, {3}, {4}, {5}, {6}, {7} };
IWDG_TypeDef sim_iwdg;

/* Defined by CubeMX code on the target. */
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart1, huart2;
I2C_HandleTypeDef hi2c1;
CAN_BusStatus can1_status;

static uint32_t now_ms;

/* ── GPIO ──────────────────────────────────────────────────────────────── */

typedef struct {
	const char *name;
	GPIO_TypeDef *port;
	uint16_t pin;
	int level;
	uint32_t changes;
} sim_pin_t;

#define PIN(n) { #n, n##_GPIO_Port, n##_Pin, 0, 0 }
static sim_pin_t pins[] = {
	PIN(ASMS), PIN(SDC_FEEDBACK), PIN(IGN_BTN), PIN(MS_BTN), PIN(WDT_PULSE),
	PIN(Front_Solenoid), PIN(Rear_Solenoid), PIN(ASSI_YELLOW), PIN(ASSI_BLUE),
	PIN(LED1), PIN(LED2), PIN(HB), PIN(CAN_INDICATO),
};
#define PIN_COUNT (sizeof(pins) / sizeof(pins[0]))

static sim_pin_t *pin_at(GPIO_TypeDef *port, uint16_t pin) {
	for (size_t i = 0; i < PIN_COUNT; i++) {
		if (pins[i].port == port && pins[i].pin == pin) {
			return &pins[i];
		}
	}
	return NULL;
}

static sim_pin_t *pin_named(const char *name) {
	for (size_t i = 0; i < PIN_COUNT; i++) {
		if (strcmp(pins[i].name, name) == 0) {
			return &pins[i];
		}
	}
	return NULL;
}

uint32_t HAL_GetTick(void) {
	return now_ms;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
	sim_pin_t *p = pin_at(port, pin);
	return (p && p->level) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
	sim_pin_t *p = pin_at(port, pin);
	int level = state != GPIO_PIN_RESET;
	if (!p) {
		return;
	}
	if (level != p->level) {
		p->changes++;
	}
	p->level = level;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin) {
	sim_pin_t *p = pin_at(port, pin);
	if (p) {
		p->level = !p->level;
		p->changes++;
	}
}

static int pin_level(const char *name) {
	sim_pin_t *p = pin_named(name);
	return p ? p->level : -1;
}

static void set_pin_level(const char *name, int level) {
	sim_pin_t *p = pin_named(name);
	if (p) {
		p->level = level;
	}
}

/* ── USART1 = RN4871 ───────────────────────────────────────────────────────
 * ble_out is what the phone receives; DMA transfers take the real time at
 * 115200 baud before the TX complete interrupt fires. */

static char ble_out[262144];
static size_t ble_out_len;
static int ble_tx_pending;
static uint32_t ble_tx_done_ms;
static DMA_HandleTypeDef ble_rx_dma;
static uint8_t *ble_rx_buf;
static uint16_t ble_rx_size;
static char ble_commands[1024];
static size_t ble_commands_len;

static void ble_rx_write(const char *text) {
	for (; *text && ble_rx_buf; text++) {
		ble_rx_buf[ble_rx_size - ble_rx_dma.remaining] = (uint8_t)*text;
		ble_rx_dma.remaining = ble_rx_dma.remaining > 1 ? ble_rx_dma.remaining - 1 : ble_rx_size;
	}
}

/* Blocking sends on USART1 are the RN4871 configuration commands. The module
 * answers like the real one: CMD> after $$$, AOK per command. */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n, uint32_t t) {
	(void)t;
	if (h != &huart1) {
		return HAL_OK;
	}
	if (ble_commands_len + n < sizeof(ble_commands)) {
		memcpy(ble_commands + ble_commands_len, d, n);
		ble_commands_len += n;
	}
	if (n == 3 && memcmp(d, "$$$", 3) == 0) {
		ble_rx_write("CMD> ");
	} else if (n >= 3 && memcmp(d, "R,1", 3) == 0) {
		ble_rx_write("Rebooting\r\n");
	} else {
		ble_rx_write("AOK\r\nCMD> ");
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n) {
	size_t room = sizeof(ble_out) - ble_out_len;
	if (h != &huart1) {
		return HAL_OK;
	}
	if (ble_tx_pending) {
		return HAL_BUSY;
	}
	memcpy(ble_out + ble_out_len, d, n < room ? n : room);
	ble_out_len += n < room ? n : room;
	ble_tx_pending = 1;
	ble_tx_done_ms = now_ms + (n * 10u * 1000u + 115199u) / 115200u;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) {
	if (h == &huart1) {
		ble_rx_buf = d;
		ble_rx_size = n;
		ble_rx_dma.remaining = n;
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h) {
	if (h == &huart1) {
		ble_tx_pending = 0;
	}
	return HAL_OK;
}

/* ── CAN TX: last frame and count per ID ────────────────────────────────── */

#define TX_SLOTS 16
typedef struct {
	uint32_t id;
	uint32_t count;
	uint8_t length;
	uint8_t data[8];
} tx_slot_t;
static tx_slot_t tx_slots[TX_SLOTS];

static tx_slot_t *tx_slot(uint32_t id, int create) {
	for (int i = 0; i < TX_SLOTS; i++) {
		if (tx_slots[i].count && tx_slots[i].id == id) {
			return &tx_slots[i];
		}
	}
	if (!create) {
		return NULL;
	}
	for (int i = 0; i < TX_SLOTS; i++) {
		if (tx_slots[i].count == 0) {
			tx_slots[i].id = id;
			return &tx_slots[i];
		}
	}
	return NULL;
}

void CAN_Service(CAN_HandleTypeDef *hcan) {
	(void)hcan;
}

HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *header, const uint8_t data[8]) {
	(void)hcan;
	tx_slot_t *slot = tx_slot(header->StdId, 1);
	if (slot) {
		slot->count++;
		slot->length = (uint8_t)header->DLC;
		memcpy(slot->data, data, 8);
	}
	return HAL_OK;
}

/* ── EEPROM (24FC08, 1 KB) ─────────────────────────────────────────────── */

static uint8_t eeprom[1024];
static int eeprom_present = 1;
static uint32_t eeprom_writes;

bool EE24_Init(EE24_HandleTypeDef *h, I2C_HandleTypeDef *i2c, uint8_t address) {
	h->HI2c = i2c;
	h->Address = address;
	return eeprom_present;
}

bool EE24_Read(EE24_HandleTypeDef *h, uint32_t address, uint8_t *data, size_t len, uint32_t timeout) {
	(void)h;
	(void)timeout;
	if (!eeprom_present || address + len > sizeof(eeprom)) {
		return false;
	}
	memcpy(data, eeprom + address, len);
	return true;
}

bool EE24_Write(EE24_HandleTypeDef *h, uint32_t address, uint8_t *data, size_t len, uint32_t timeout) {
	(void)h;
	(void)timeout;
	if (!eeprom_present || address + len > sizeof(eeprom)) {
		return false;
	}
	memcpy(eeprom + address, data, len);
	eeprom_writes++;
	return true;
}

/* ── Board services (acu_system.c on the target) ────────────────────────── */

static uint32_t bootloader_resets;

void system_reset_to_bootloader(void) {
	bootloader_resets++;
}

/* ── The car around the ACU ────────────────────────────────────────────── */

typedef struct {
	/* parameters, set from Python */
	float front_tank_bar;
	float rear_tank_bar;
	float front_ratio;            /* hydraulic / tank pressure with the EBS applied */
	float rear_ratio;
	float brake_tau_ms;           /* hydraulic pressure response */
	int32_t sdc_chain_closed;     /* rest of the shutdown circuit (RES, buttons, BSPD, ...) */
	int32_t watchdog_timeout_ms;  /* external watchdog opens the SDC without pulses */
	int32_t watchdog_stuck_closed;/* fault: the watchdog never opens the SDC */
	int32_t vcu_precharge_ms;
	int32_t ts_stuck_on;          /* fault: TS stays active whatever happens (welded AIR) */
	int32_t brakes_released_by_hand; /* ASR operated the EBS deactivation points */
	int32_t ts_forced_off;        /* VCU reports the TS off whatever happens */
	float motor_rpm;              /* both motors */
	int32_t vcu_alive;
	int32_t vcu_rpm_alive;
	int32_t jetson_alive;
	int32_t res_alive;
	int32_t steering_alive;
	int32_t front_brake_alive;
	int32_t rear_brake_alive;
	int32_t jetson_as_state;
	int32_t jetson_mission;
	/* plant state, read from Python */
	float front_brake_bar;
	float rear_brake_bar;
	int32_t ts_active;
	int32_t sdc_closed;
	int32_t precharge_ms;
	uint32_t last_pulse_ms;
	uint32_t pulse_changes;
} sim_plant_t;

SIM_API sim_plant_t sim_plant;

static void plant_defaults(void) {
	memset(&sim_plant, 0, sizeof(sim_plant));
	sim_plant.front_tank_bar = 7.66f;
	sim_plant.rear_tank_bar = 3.36f;
	sim_plant.front_ratio = 9.4f;
	sim_plant.rear_ratio = 7.2f;
	sim_plant.brake_tau_ms = 150.0f;
	sim_plant.sdc_chain_closed = 1;
	sim_plant.watchdog_timeout_ms = 100;
	sim_plant.vcu_precharge_ms = 300;
	sim_plant.vcu_alive = 1;
	sim_plant.vcu_rpm_alive = 1;
	sim_plant.jetson_alive = 1;
	sim_plant.res_alive = 1;
	sim_plant.steering_alive = 1;
	sim_plant.front_brake_alive = 1;
	sim_plant.rear_brake_alive = 1;
	sim_plant.jetson_as_state = AS_STATE_OFF;
	sim_plant.jetson_mission = MISSION_MANUAL;
}

static void rx(uint32_t id, int extended, const uint8_t *data, uint8_t length) {
	can_frame_t frame = { .id = id, .extended = extended != 0, .length = length };
	memcpy(frame.data, data, length);
	can_rx_handle_frame(&frame, now_ms);
}

static void rx_jetson(void) {
	struct autonomous_t26_jetson_t msg = {
		.as_state = (uint8_t)sim_plant.jetson_as_state,
		.as_mission = (uint8_t)sim_plant.jetson_mission,
	};
	uint8_t data[8] = { 0 };
	autonomous_t26_jetson_pack(data, &msg, sizeof(data));
	rx(AUTONOMOUS_T26_JETSON_FRAME_ID, 0, data, AUTONOMOUS_T26_JETSON_LENGTH);
}

static void rx_vcu(void) {
	struct autonomous_t26_vcu_ign_r2_d_t msg = {
		.ignition_auto = (uint8_t)sim_plant.ts_active,
		.shutdown_signal = (uint8_t)sim_plant.sdc_closed,
	};
	uint8_t data[8] = { 0 };
	autonomous_t26_vcu_ign_r2_d_pack(data, &msg, sizeof(data));
	rx(AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID, 0, data, AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH);
}

static void rx_rpm(void) {
	struct autonomous_t26_vcu_rpm_t msg = {
		.motor_rpm_left = (uint16_t)sim_plant.motor_rpm,
		.motor_rpm_right = (uint16_t)sim_plant.motor_rpm,
	};
	uint8_t data[8] = { 0 };
	autonomous_t26_vcu_rpm_pack(data, &msg, sizeof(data));
	rx(AUTONOMOUS_T26_VCU_RPM_FRAME_ID, 0, data, AUTONOMOUS_T26_VCU_RPM_LENGTH);
}

static void rx_brakes(void) {
	uint8_t data[8] = { 0 };
	if (sim_plant.front_brake_alive) {
		struct autonomous_t26_aqt1_t front = {
			.frt_brk_press = autonomous_t26_aqt1_frt_brk_press_encode(sim_plant.front_brake_bar),
		};
		autonomous_t26_aqt1_pack(data, &front, sizeof(data));
		rx(AUTONOMOUS_T26_AQT1_FRAME_ID, 0, data, AUTONOMOUS_T26_AQT1_LENGTH);
	}
	if (sim_plant.rear_brake_alive) {
		struct autonomous_t26_aqt7_t rear = {
			.rear_brk_press = autonomous_t26_aqt7_rear_brk_press_encode(sim_plant.rear_brake_bar),
		};
		autonomous_t26_aqt7_pack(data, &rear, sizeof(data));
		rx(AUTONOMOUS_T26_AQT7_FRAME_ID, 0, data, AUTONOMOUS_T26_AQT7_LENGTH);
	}
}

static uint16_t adc_raw_from_bar(float bar) {
	float sensor_volts = bar * 0.4f + 0.5f;
	float adc = sensor_volts * 0.66f * 4096.0f / 3.3f;
	return adc < 0 ? 0 : (adc > 4095 ? 4095 : (uint16_t)(adc + 0.5f));
}

static void plant_step(void) {
	sim_plant_t *p = &sim_plant;
	sim_pin_t *pulse = pin_named("WDT_PULSE");

	/* external watchdog: SDC stays closed while it got a pulse in the last timeout */
	if (pulse->changes != p->pulse_changes) {
		p->pulse_changes = pulse->changes;
		p->last_pulse_ms = now_ms;
	}
	int pulsing = now_ms - p->last_pulse_ms <= (uint32_t)p->watchdog_timeout_ms && p->pulse_changes > 0;
	p->sdc_closed = p->sdc_chain_closed && (pulsing || p->watchdog_stuck_closed);
	set_pin_level("SDC_FEEDBACK", p->sdc_closed ? 0 : 1); /* active low */

	/* VCU: TS on after precharge while the ACU asks for it and the SDC is closed */
	int requested = acu.outputs.ignition_request;
	if (p->ts_stuck_on) {
		p->ts_active = 1;
	} else if (p->ts_forced_off) {
		p->ts_active = 0;
	} else if (!p->sdc_closed || !requested) {
		p->ts_active = 0;
		p->precharge_ms = 0;
	} else if (!p->ts_active && ++p->precharge_ms >= p->vcu_precharge_ms) {
		p->ts_active = 1;
	}

	/* EBS: a de-energised solenoid (or no actuator power with the ASMS off) applies the brake */
	int asms_on = pin_level("ASMS");
	int front_engaged = !pin_level("Front_Solenoid") || !asms_on;
	int rear_engaged = !pin_level("Rear_Solenoid") || !asms_on;
	float front_target = (front_engaged && !p->brakes_released_by_hand) ? p->front_ratio * p->front_tank_bar : 0.0f;
	float rear_target = (rear_engaged && !p->brakes_released_by_hand) ? p->rear_ratio * p->rear_tank_bar : 0.0f;
	float k = 1.0f / p->brake_tau_ms;
	p->front_brake_bar += (front_target - p->front_brake_bar) * k;
	p->rear_brake_bar += (rear_target - p->rear_brake_bar) * k;

	/* sensors and CAN nodes */
	inputs_on_adc_conversion(adc_raw_from_bar(p->front_tank_bar), adc_raw_from_bar(p->rear_tank_bar), 35.0f);
	if (now_ms % 10 == 0) {
		rx_brakes();
	}
	if (p->steering_alive && now_ms % 10 == 5) {
		uint8_t data[8] = { 0 };
		rx(AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID, 1, data, 8);
	}
	if (p->jetson_alive && now_ms % 20 == 2) {
		rx_jetson();
	}
	if (p->vcu_alive && now_ms % 20 == 7) {
		rx_vcu();
	}
	if (p->vcu_rpm_alive && now_ms % 20 == 9) {
		rx_rpm();
	}
	if (p->res_alive && now_ms % 50 == 13) {
		uint8_t data[8] = { 0 };
		rx(RES_PDO_FRAME_ID, 0, data, 8);
	}
}

/* ── Simulation API ─────────────────────────────────────────────────────── */

/* Power-on. The EEPROM keeps its content, like the real one. */
SIM_API void sim_init(void) {
	now_ms = 0;
	huart1.hdmarx = &ble_rx_dma;
	plant_defaults();
	app_init(now_ms);
	acu.system.reset_cause = "power-on (sim)";
	logger_task_init(0x0C000000u);
	ble_handler_init(now_ms);
}

/* One millisecond: plant, control task, then the telemetry and logger tasks when due. */
SIM_API void sim_step_ms(uint32_t ms) {
	while (ms--) {
		now_ms++;
		if (ble_tx_pending && now_ms >= ble_tx_done_ms) {
			ble_tx_pending = 0;
			HAL_UART_TxCpltCallback(&huart1);
		}
		plant_step();
		app_control_step(now_ms);
		if (now_ms % 10 == 0) {
			ble_handler(now_ms);
		}
		if (now_ms % 20 == 0) {
			logger_task_step();
		}
	}
}

SIM_API void sim_set_pin(const char *name, int level) {
	set_pin_level(name, level);
}

SIM_API int sim_get_pin(const char *name) {
	return pin_level(name);
}

SIM_API uint32_t sim_pin_changes(const char *name) {
	sim_pin_t *p = pin_named(name);
	return p ? p->changes : 0;
}

SIM_API void sim_press_mission_button(void) {
	inputs_on_mission_button(now_ms);
}

SIM_API void sim_set_mission(int mission) {
	acu.mission.selected = (mission_t)mission;
}

SIM_API void sim_can_rx_raw(uint32_t id, int extended, const uint8_t *data, int length) {
	rx(id, extended, data, (uint8_t)length);
}

SIM_API void sim_can_rx_jetson_now(void) {
	rx_jetson();
}

SIM_API uint32_t sim_can_tx_count(uint32_t id) {
	tx_slot_t *slot = tx_slot(id, 0);
	return slot ? slot->count : 0;
}

SIM_API int sim_can_tx_last(uint32_t id, uint8_t out[8]) {
	tx_slot_t *slot = tx_slot(id, 0);
	if (!slot) {
		return -1;
	}
	memcpy(out, slot->data, 8);
	return slot->length;
}

/* Decoded last ACU frame (0x51), as the Jetson sees it. */
typedef struct {
	int32_t as_state, asms, assi_state, emergency, ign, mission_select, acu_state, emergency_cause, cpu_temp;
} sim_acu_frame_t;

SIM_API int sim_last_acu_frame(sim_acu_frame_t *out) {
	uint8_t data[8];
	struct autonomous_t26_acu_t msg;
	if (sim_can_tx_last(AUTONOMOUS_T26_ACU_FRAME_ID, data) < 0
			|| autonomous_t26_acu_unpack(&msg, data, AUTONOMOUS_T26_ACU_LENGTH) != 0) {
		return -1;
	}
	out->as_state = msg.as_state;
	out->asms = msg.asms;
	out->assi_state = msg.assi_state;
	out->emergency = msg.emergency;
	out->ign = msg.ign;
	out->mission_select = msg.mission_select;
	out->acu_state = msg.acu_state;
	out->emergency_cause = msg.emergency_cause;
	out->cpu_temp = msg.acu_cpu_temp;
	return 0;
}

/* Decoded last DV_status (0x502), as the data logger sees it. */
typedef struct {
	int32_t as_status, asb_ebs_state, ami_state, steering_state, asb_redundancy_state, lap_counter;
} sim_dv_status_t;

SIM_API int sim_last_dv_status(sim_dv_status_t *out) {
	uint8_t data[8];
	struct autonomous_t26_dv_status_t msg;
	if (sim_can_tx_last(AUTONOMOUS_T26_DV_STATUS_FRAME_ID, data) < 0
			|| autonomous_t26_dv_status_unpack(&msg, data, AUTONOMOUS_T26_DV_STATUS_LENGTH) != 0) {
		return -1;
	}
	out->as_status = msg.as_status;
	out->asb_ebs_state = msg.asb_ebs_state;
	out->ami_state = msg.ami_state;
	out->steering_state = msg.steering_state;
	out->asb_redundancy_state = msg.asb_redundancy_state;
	out->lap_counter = msg.lap_counter;
	return 0;
}

/* Figure 16 with the inputs as bits, in as_flowchart_inputs_t order. */
SIM_API int sim_flowchart(uint32_t bits) {
	as_flowchart_inputs_t in = {
		.ebs_activated = (bits >> 0) & 1,
		.mission_finished = (bits >> 1) & 1,
		.vehicle_at_standstill = (bits >> 2) & 1,
		.sdc_opened_by_res = (bits >> 3) & 1,
		.mission_selected = (bits >> 4) & 1,
		.asms_on = (bits >> 5) & 1,
		.asb_checks_ok = (bits >> 6) & 1,
		.ts_active = (bits >> 7) & 1,
		.r2d = (bits >> 8) & 1,
		.brakes_engaged = (bits >> 9) & 1,
	};
	return as_flowchart(&in);
}

/* Fault injection: write garbage into a state variable. */
SIM_API void sim_corrupt(const char *what, int value) {
	if (strcmp(what, "mode") == 0) {
		acu.mode = (acu_mode_t)value;
	} else if (strcmp(what, "checkup") == 0) {
		acu.checkup.step = (checkup_step_t)value;
	} else if (strcmp(what, "finish") == 0) {
		acu.as.finish = (finish_step_t)value;
	}
}

/* Flat copy of the parts of `acu` the tests look at. Mirrored in acu_sim.py. */
typedef struct {
	uint32_t time_ms;
	int32_t mode, as_state, checkup_step, checkup_failed_step, finish_step;
	int32_t fault_active, fault_reason, fault_last_reason;
	uint32_t fault_count;
	int32_t mission, jetson_mission, jetson_as_state;
	int32_t asms_on, sdc_closed, ignition_button, ts_active;
	int32_t front_ebs_released, rear_ebs_released, watchdog_pulses_on;
	int32_t ignition_enable, ignition_request, assi_yellow, assi_blue, res_start_pending;
	int32_t ebs_activated, mission_finished, standstill, r2d, asb_checks_ok, brakes_engaged;
	int32_t speed_known, moving, monitor_active;
	float front_tank_bar, rear_tank_bar, front_brake_bar, rear_brake_bar;
	uint32_t age_vcu, age_jetson, age_res;
	uint32_t checkup_passed_count, checkup_failed_count;
	uint32_t log_stored, log_written, log_dropped, bootloader_resets, eeprom_writes;
	int32_t plant_ts_active, plant_sdc_closed;
	int32_t ebs_state;
} sim_state_t;

SIM_API uint32_t sim_state_size(void) {
	return sizeof(sim_state_t);
}

SIM_API void sim_get_state(sim_state_t *s) {
	s->time_ms = now_ms;
	s->mode = acu.mode;
	s->as_state = acu.as.state;
	s->checkup_step = acu.checkup.step;
	s->checkup_failed_step = acu.checkup.failed_step;
	s->finish_step = acu.as.finish;
	s->fault_active = acu.fault.active;
	s->fault_reason = acu.fault.reason;
	s->fault_last_reason = acu.fault.last_reason;
	s->fault_count = acu.fault.count;
	s->mission = acu.mission.selected;
	s->jetson_mission = acu.jetson.mission;
	s->jetson_as_state = acu.jetson.as_state;
	s->asms_on = acu.inputs.asms_on;
	s->sdc_closed = acu.inputs.sdc_closed;
	s->ignition_button = acu.inputs.ignition_button;
	s->ts_active = acu.vcu.ts_active;
	s->front_ebs_released = acu.outputs.front_ebs_released;
	s->rear_ebs_released = acu.outputs.rear_ebs_released;
	s->watchdog_pulses_on = acu.outputs.watchdog_pulses_on;
	s->ignition_enable = acu.outputs.ignition_enable;
	s->ignition_request = acu.outputs.ignition_request;
	s->assi_yellow = acu.outputs.assi_yellow;
	s->assi_blue = acu.outputs.assi_blue;
	s->res_start_pending = acu.outputs.res_start_pending;
	s->ebs_activated = acu.as.flowchart.ebs_activated;
	s->mission_finished = acu.as.flowchart.mission_finished;
	s->standstill = acu.as.flowchart.vehicle_at_standstill;
	s->r2d = acu.as.flowchart.r2d;
	s->asb_checks_ok = acu.as.flowchart.asb_checks_ok;
	s->brakes_engaged = acu.as.flowchart.brakes_engaged;
	s->speed_known = acu.as.speed_known;
	s->moving = acu.as.moving;
	s->monitor_active = acu.monitor.active;
	s->front_tank_bar = acu.inputs.front_tank_bar;
	s->rear_tank_bar = acu.inputs.rear_tank_bar;
	s->front_brake_bar = acu.brakes.front_bar;
	s->rear_brake_bar = acu.brakes.rear_bar;
	s->age_vcu = acu.nodes.vcu.age_ms;
	s->age_jetson = acu.nodes.jetson.age_ms;
	s->age_res = acu.nodes.res.age_ms;
	s->checkup_passed_count = acu.checkup.passed_count;
	s->checkup_failed_count = acu.checkup.failed_count;
	s->log_stored = acu.log.stored;
	s->log_written = acu.log.written;
	s->log_dropped = acu.log.dropped;
	s->bootloader_resets = bootloader_resets;
	s->eeprom_writes = eeprom_writes;
	s->plant_ts_active = sim_plant.ts_active;
	s->plant_sdc_closed = sim_plant.sdc_closed;
	s->ebs_state = (int32_t)ebs_state();
}

/* Text the ACU sent to the phone since the last call. */
SIM_API uint32_t sim_ble_read(char *dst, uint32_t max) {
	uint32_t n = ble_out_len < max ? (uint32_t)ble_out_len : max;
	memcpy(dst, ble_out, n);
	memmove(ble_out, ble_out + n, ble_out_len - n);
	ble_out_len -= n;
	return n;
}

/* Every configuration command sent to the RN4871 since power-on. */
SIM_API uint32_t sim_ble_commands(char *dst, uint32_t max) {
	uint32_t n = ble_commands_len < max ? (uint32_t)ble_commands_len : max;
	memcpy(dst, ble_commands, n);
	return n;
}

/* Text typed on the phone, written into the circular RX DMA buffer. */
SIM_API void sim_ble_receive(const char *text) {
	ble_rx_write(text);
}

/* EEPROM content, to carry it into the next power-on (a new AcuSim). */
SIM_API void sim_eeprom_dump(uint8_t *out) {
	memcpy(out, eeprom, sizeof(eeprom));
}

SIM_API void sim_eeprom_load(const uint8_t *in) {
	memcpy(eeprom, in, sizeof(eeprom));
}

SIM_API void sim_eeprom_present(int present) {
	eeprom_present = present;
}

/* Log record `index` (0 = oldest) as the logger keeps it in RAM. */
SIM_API int sim_log_record(uint32_t index, log_record_t *out) {
	const log_record_t *r = logger_record((uint8_t)index);
	if (!r) {
		return -1;
	}
	*out = *r;
	return 0;
}

SIM_API uint32_t sim_log_record_size(void) {
	return sizeof(log_record_t);
}
