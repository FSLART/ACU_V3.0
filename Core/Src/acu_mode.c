/*
 * acu_mode.c
 */
#include "acu_mode.h"
#include "acu_system.h"
#include "as_state.h"
#include "checkup.h"
#include "faults.h"
#include "monitoring.h"

static bool asms_was_on = false;

static void enter(acu_mode_t mode, uint32_t now_ms) {
	acu.mode = mode;
	acu.mode_since_ms = now_ms;
}

static void enter_idle(uint32_t now_ms) {
	fault_clear();
	checkup_reset();
	as_reset(now_ms);
	enter(ACU_MODE_IDLE, now_ms);
}

/* CAN programming resets the board: only with the car parked and the TS off. */
static void handle_bootloader_request(void) {
	if (!acu.system.bootloader_requested) {
		return;
	}
	acu.system.bootloader_requested = false;
	if (!acu.inputs.asms_on && !acu.vcu.ts_active) {
		system_reset_to_bootloader();
	}
}

void acu_mode_update(uint32_t now_ms) {
	bool asms_switched_on = acu.inputs.asms_on && !asms_was_on;
	asms_was_on = acu.inputs.asms_on;

	switch (acu.mode) {
	case ACU_MODE_STARTUP:
		enter_idle(now_ms);
		break;

	case ACU_MODE_IDLE:
		handle_bootloader_request();
		if (asms_switched_on && !acu.inputs.ignition_button && acu.mission.selected != MISSION_MANUAL) {
			checkup_start(now_ms);
			acu.outputs.res_start_pending = true;
			enter(ACU_MODE_AUTONOMOUS, now_ms);
		}
		break;

	case ACU_MODE_AUTONOMOUS:
		if (!checkup_passed()) {
			if (!acu.inputs.asms_on) {
				enter_idle(now_ms); /* checkup aborted: the EBS was never armed */
				break;
			}
			checkup_update(now_ms);
		}
		break;

	case ACU_MODE_EMERGENCY:
		if (!acu.inputs.asms_on && !acu.as.flowchart.ebs_activated) {
			enter_idle(now_ms);
		}
		break;

	default:
		fault_raise(FAULT_INVALID_STATE, now_ms);
		enter(ACU_MODE_EMERGENCY, now_ms);
		break;
	}

	monitoring_update(now_ms);
	as_update(now_ms);

	if (acu.fault.active && acu.mode != ACU_MODE_EMERGENCY) {
		enter(ACU_MODE_EMERGENCY, now_ms);
	}
	if (acu.mode == ACU_MODE_AUTONOMOUS && acu.as.finish == FINISH_EBS_ACTIVATED
			&& !acu.as.flowchart.ebs_activated) {
		enter_idle(now_ms); /* AS Finished is over: ASMS off and brakes released */
	}
}
