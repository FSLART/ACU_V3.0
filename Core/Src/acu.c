/*
 * acu.c
 *
 * The global `acu` struct and the names shown on Bluetooth.
 */
#include "acu.h"
#include <string.h>

acu_t acu;

void acu_init(uint32_t now_ms) {
	memset(&acu, 0, sizeof(acu));
	acu.mode = ACU_MODE_STARTUP;
	acu.mode_since_ms = now_ms;
	acu.mission.selected = MISSION_MANUAL;
	acu.checkup.step = CHECKUP_NOT_STARTED;
	acu.checkup.failed_step = CHECKUP_NOT_STARTED;
	acu.as.state = AS_STATE_OFF;
	acu.as.state_since_ms = now_ms;
	acu.as.finish = FINISH_NOT_FINISHED;
	acu.can_bus = &can1_status;
	acu.system.reset_cause = "?";

	/* Every node counts as just heard at boot: timeouts start counting from power-up. */
	acu.nodes.vcu.last_rx_ms = now_ms;
	acu.nodes.vcu_rpm.last_rx_ms = now_ms;
	acu.nodes.jetson.last_rx_ms = now_ms;
	acu.nodes.res.last_rx_ms = now_ms;
	acu.nodes.steering.last_rx_ms = now_ms;
	acu.nodes.brake_front.last_rx_ms = now_ms;
	acu.nodes.brake_rear.last_rx_ms = now_ms;
}

#define NAME_OF(table, value) \
	(((unsigned)(value) < sizeof(table) / sizeof((table)[0]) && (table)[value]) ? (table)[value] : "?")

const char *acu_mode_name(acu_mode_t mode) {
	static const char *const names[] = {
		[ACU_MODE_STARTUP] = "STARTUP",
		[ACU_MODE_IDLE] = "IDLE",
		[ACU_MODE_AUTONOMOUS] = "AUTONOMOUS",
		[ACU_MODE_EMERGENCY] = "EMERGENCY",
	};
	return NAME_OF(names, mode);
}

const char *as_state_name(AS_STATE_t state) {
	static const char *const names[] = {
		[AS_STATE_OFF] = "OFF",
		[AS_STATE_READY] = "READY",
		[AS_STATE_DRIVING] = "DRIVING",
		[AS_STATE_EMERGENCY] = "EMERGENCY",
		[AS_STATE_FINISHED] = "FINISHED",
	};
	return NAME_OF(names, state);
}

const char *checkup_step_name(checkup_step_t step) {
	static const char *const names[] = {
		[CHECKUP_NOT_STARTED] = "NOT_STARTED",
		[CHECKUP_TANK_PRESSURE] = "TANK_PRESSURE",
		[CHECKUP_BRAKES_ENGAGED] = "BRAKES_ENGAGED",
		[CHECKUP_SDC_CLOSE] = "SDC_CLOSE",
		[CHECKUP_WATCHDOG_OPENS_SDC] = "WATCHDOG_OPENS_SDC",
		[CHECKUP_SDC_RECLOSE] = "SDC_RECLOSE",
		[CHECKUP_TS_ACTIVATION] = "TS_ACTIVATION",
		[CHECKUP_FRONT_BRAKE] = "FRONT_BRAKE",
		[CHECKUP_REAR_BRAKE] = "REAR_BRAKE",
		[CHECKUP_BOTH_BRAKES] = "BOTH_BRAKES",
		[CHECKUP_PASSED] = "PASSED",
		[CHECKUP_FAILED] = "FAILED",
	};
	return NAME_OF(names, step);
}

const char *finish_step_name(finish_step_t step) {
	static const char *const names[] = {
		[FINISH_NOT_FINISHED] = "-",
		[FINISH_WAITING_FOR_STANDSTILL] = "WAITING_FOR_STANDSTILL",
		[FINISH_EBS_ACTIVATED] = "EBS_ACTIVATED",
	};
	return NAME_OF(names, step);
}

const char *mission_name(mission_t mission) {
	static const char *const names[] = {
		[MISSION_MANUAL] = "MANUAL",
		[MISSION_ACCELERATION] = "ACCELERATION",
		[MISSION_SKIDPAD] = "SKIDPAD",
		[MISSION_TRACKDRIVE] = "TRACKDRIVE",
		[MISSION_EBS_TEST] = "EBS_TEST",
		[MISSION_INSPECTION] = "INSPECTION",
		[MISSION_AUTOCROSS] = "AUTOCROSS",
	};
	return NAME_OF(names, mission);
}
