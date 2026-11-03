/*
 * faults.c
 */
#include "faults.h"
#include "logger.h"

typedef struct {
	const char *name;
	const char *description;      /* shown on Bluetooth */
	can_emergency_cause_t can_cause;
} fault_info_t;

static const fault_info_t fault_table[FAULT_COUNT] = {
	[FAULT_NONE] = { "NONE", "sem falha", CAN_CAUSE_NONE },

	[FAULT_CHECKUP_TANK_PRESSURE] = { "CHECKUP_TANK_PRESSURE",
		"arranque: pressao dos tanques EBS fora dos limites", CAN_CAUSE_PRESSURE_CHECKS },
	[FAULT_CHECKUP_BRAKES_NOT_ENGAGED] = { "CHECKUP_BRAKES_NOT_ENGAGED",
		"arranque: travoes nao ganharam pressao antes de fechar o SDC", CAN_CAUSE_PRESSURE_CHECKS },
	[FAULT_CHECKUP_SDC_DID_NOT_CLOSE] = { "CHECKUP_SDC_DID_NOT_CLOSE",
		"arranque: SDC nao fechou com o watchdog a pulsar", CAN_CAUSE_SDC_OPEN },
	[FAULT_CHECKUP_WATCHDOG_DID_NOT_OPEN_SDC] = { "CHECKUP_WATCHDOG_DID_NOT_OPEN_SDC",
		"arranque: watchdog parado mas o SDC nao abriu", CAN_CAUSE_SDC_OPEN },
	[FAULT_CHECKUP_SDC_DID_NOT_RECLOSE] = { "CHECKUP_SDC_DID_NOT_RECLOSE",
		"arranque: SDC nao voltou a fechar depois do teste do watchdog", CAN_CAUSE_SDC_OPEN },
	[FAULT_CHECKUP_SDC_OPENED] = { "CHECKUP_SDC_OPENED",
		"arranque: SDC abriu a meio da sequencia", CAN_CAUSE_SDC_OPEN },
	[FAULT_CHECKUP_FRONT_BRAKE] = { "CHECKUP_FRONT_BRAKE",
		"arranque: circuito EBS da frente sozinho falhou", CAN_CAUSE_PRESSURE_CHECKS },
	[FAULT_CHECKUP_REAR_BRAKE] = { "CHECKUP_REAR_BRAKE",
		"arranque: circuito EBS de tras sozinho falhou", CAN_CAUSE_PRESSURE_CHECKS },
	[FAULT_CHECKUP_BOTH_BRAKES] = { "CHECKUP_BOTH_BRAKES",
		"arranque: os dois circuitos EBS juntos falharam", CAN_CAUSE_PRESSURE_CHECKS },

	[FAULT_RES_STOP] = { "RES_STOP", "RES: botao de emergencia premido", CAN_CAUSE_RES },
	[FAULT_SDC_OPENED] = { "SDC_OPENED", "SDC abriu em AS Ready/Driving", CAN_CAUSE_SDC_OPEN },
	[FAULT_ASMS_SWITCHED_OFF] = { "ASMS_SWITCHED_OFF", "ASMS desligado em AS Ready/Driving", CAN_CAUSE_SDC_OPEN },
	[FAULT_VCU_TIMEOUT] = { "VCU_TIMEOUT", "VCU calado (VCU_IGN_R2D)", CAN_CAUSE_VCU_TIMEOUT },
	[FAULT_JETSON_TIMEOUT] = { "JETSON_TIMEOUT", "Jetson calado", CAN_CAUSE_JETSON_TIMEOUT },
	[FAULT_RES_TIMEOUT] = { "RES_TIMEOUT", "RES calado", CAN_CAUSE_RES },
	[FAULT_TANK_PRESSURE] = { "TANK_PRESSURE", "pressao dos tanques EBS fora dos limites", CAN_CAUSE_PRESSURE_CHECKS },
	[FAULT_BRAKE_PRESSURE_LOST] = { "BRAKE_PRESSURE_LOST",
		"AS Ready: travoes perderam pressao", CAN_CAUSE_PRESSURE_CHECKS },
	[FAULT_TS_DEACTIVATED] = { "TS_DEACTIVATED", "TS desligou em AS Ready/Driving", CAN_CAUSE_UNKNOWN },
	[FAULT_MISSION_MISMATCH] = { "MISSION_MISMATCH", "missao do Jetson diferente da selecionada", CAN_CAUSE_UNKNOWN },
	[FAULT_JETSON_EMERGENCY] = { "JETSON_EMERGENCY", "Jetson pediu EMERGENCY", CAN_CAUSE_UNKNOWN },

	[FAULT_R2D_BEFORE_5S] = { "R2D_BEFORE_5S", "T 14.8.4: DRIVING pedido antes de 5 s em AS Ready", CAN_CAUSE_UNKNOWN },
	[FAULT_MOVED_BEFORE_3S] = { "MOVED_BEFORE_3S", "T 14.8.5: carro mexeu antes de 3 s em AS Driving", CAN_CAUSE_UNKNOWN },
	[FAULT_FINISH_NOT_AT_STANDSTILL] = { "FINISH_NOT_AT_STANDSTILL",
		"missao acabou mas o carro nao parou (ou sem RPM do VCU)", CAN_CAUSE_UNKNOWN },
	[FAULT_TS_STILL_ON_AFTER_FINISH] = { "TS_STILL_ON_AFTER_FINISH",
		"AS Finished mas o VCU continua com o TS ligado", CAN_CAUSE_UNKNOWN },

	[FAULT_INVALID_STATE] = { "INVALID_STATE", "maquina de estados num estado invalido", CAN_CAUSE_ACU_WATCHDOG },
	[FAULT_STACK_OVERFLOW] = { "STACK_OVERFLOW", "FreeRTOS: stack overflow", CAN_CAUSE_ACU_WATCHDOG },
	[FAULT_OUT_OF_MEMORY] = { "OUT_OF_MEMORY", "FreeRTOS: sem heap", CAN_CAUSE_ACU_WATCHDOG },
	[FAULT_HARD_FAULT] = { "HARD_FAULT", "HardFault (reset pelo watchdog)", CAN_CAUSE_ACU_WATCHDOG },
};

static const fault_info_t *info(fault_reason_t reason) {
	if ((unsigned)reason >= FAULT_COUNT || fault_table[reason].name == NULL) {
		return &fault_table[FAULT_INVALID_STATE];
	}
	return &fault_table[reason];
}

void fault_raise(fault_reason_t reason, uint32_t now_ms) {
	if (acu.fault.active) {
		return; /* keep the first reason: it is the one that explains the EMERGENCY */
	}
	acu.fault.count++;
	acu.fault.last_reason = reason;
	acu.fault.active = true;
	acu.fault.reason = reason;
	acu.fault.detected_as = reason;
	acu.fault.logged = false;
	acu.fault.since_ms = now_ms;
	acu.fault.mode = acu.mode;
	acu.fault.as_state = acu.as.state;
	acu.fault.checkup_step = acu.checkup.step;
	acu.fault.front_tank_bar = acu.inputs.front_tank_bar;
	acu.fault.rear_tank_bar = acu.inputs.rear_tank_bar;
	acu.fault.front_brake_bar = acu.brakes.front_bar;
	acu.fault.rear_brake_bar = acu.brakes.rear_bar;
	acu.fault.sdc_closed = acu.inputs.sdc_closed;
	acu.fault.asms_on = acu.inputs.asms_on;
	acu.fault.ts_active = acu.vcu.ts_active;
}

/* What a RES stop also causes: it opens the SDC, drains the EBS tanks and makes the Jetson report
 * EMERGENCY, and the RES frame can arrive after any of these is seen. */
static bool can_be_caused_by_res_stop(fault_reason_t reason) {
	switch (reason) {
	case FAULT_SDC_OPENED:
	case FAULT_CHECKUP_SDC_OPENED:
	case FAULT_TANK_PRESSURE:
	case FAULT_BRAKE_PRESSURE_LOST:
	case FAULT_TS_DEACTIVATED:
	case FAULT_JETSON_EMERGENCY:
		return true;
	default:
		return false;
	}
}

/* The reaction to a fault is immediate; only its reason waits FAULT_SETTLE_MS before it is logged,
 * so a RES stop is logged as the RES stop and not as what it caused. */
void fault_update(uint32_t now_ms) {
	if (!acu.fault.active || acu.fault.logged) {
		return;
	}
	if (acu.res.stop_pressed && can_be_caused_by_res_stop(acu.fault.reason)) {
		acu.fault.reason = FAULT_RES_STOP;
		acu.fault.last_reason = FAULT_RES_STOP;
	}
	if (time_elapsed(acu.fault.since_ms, now_ms, FAULT_SETTLE_MS)) {
		acu.fault.logged = true;
		logger_log_fault();
	}
}

void fault_clear(void) {
	if (acu.fault.active && !acu.fault.logged) {
		acu.fault.logged = true;
		logger_log_fault(); /* left EMERGENCY before the reason settled: log what we have */
	}
	acu.fault.active = false;
	acu.fault.reason = FAULT_NONE;
}

const char *fault_name(fault_reason_t reason) {
	return info(reason)->name;
}

const char *fault_description(fault_reason_t reason) {
	return info(reason)->description;
}

can_emergency_cause_t fault_can_cause(fault_reason_t reason) {
	return info(reason)->can_cause;
}
