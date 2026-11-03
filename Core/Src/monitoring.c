/*
 * monitoring.c
 */
#include "monitoring.h"
#include "checkup.h"
#include "ebs.h"
#include "faults.h"

static bool ts_lost(uint32_t now_ms) {
	if (acu.vcu.ts_active || SKIP_IGNITION_CHECK) {
		acu.monitor.ts_off_since_ms = now_ms;
		return false;
	}
	return time_elapsed(acu.monitor.ts_off_since_ms, now_ms, TS_LOST_DEBOUNCE_MS);
}

static bool mission_mismatch(uint32_t now_ms) {
	acu.mission.mismatch = acu.jetson.mission != acu.mission.selected;
	if (!acu.mission.mismatch) {
		acu.mission.mismatch_since_ms = now_ms;
		return false;
	}
	return time_elapsed(acu.mission.mismatch_since_ms, now_ms, MISSION_MISMATCH_TIMEOUT_MS);
}

/* T 14.8.5: the vehicle must not start moving during the first 3 s of AS Driving. */
static bool moved_too_early(void) {
	if (acu.as.state != AS_STATE_DRIVING || !acu.as.speed_known || !acu.as.moving) {
		return false;
	}
	uint32_t driving_ms_when_it_moved = acu.as.moving_since_ms - acu.as.state_since_ms;
	return driving_ms_when_it_moved < DRIVING_NO_MOVE_TIME_MS;
}

static fault_reason_t find_fault(uint32_t now_ms) {
	if (!acu.inputs.asms_on) {
		return FAULT_ASMS_SWITCHED_OFF;
	}
	if (!acu.inputs.sdc_closed) {
		return FAULT_SDC_OPENED;
	}
	if (acu.nodes.vcu.age_ms > VCU_TIMEOUT_MS) {
		return FAULT_VCU_TIMEOUT;
	}
#if !BYPASS_JETSON_TIMEOUT
	if (acu.nodes.jetson.age_ms > JETSON_TIMEOUT_MS) {
		return FAULT_JETSON_TIMEOUT;
	}
#endif
	if (acu.nodes.res.age_ms > RES_TIMEOUT_MS) {
		return FAULT_RES_TIMEOUT;
	}
	if (!ebs_tank_pressures_ok()) {
		return FAULT_TANK_PRESSURE;
	}
	/* Until R2D the EBS holds the car (AS Ready): the brakes must stay engaged. */
	if (!acu.as.flowchart.r2d && !ebs_both_brakes_engaged()) {
		return FAULT_BRAKE_PRESSURE_LOST;
	}
	if (ts_lost(now_ms)) {
		return FAULT_TS_DEACTIVATED;
	}
	if (mission_mismatch(now_ms)) {
		return FAULT_MISSION_MISMATCH;
	}
	if (acu.jetson.as_state == AS_STATE_EMERGENCY) {
		return FAULT_JETSON_EMERGENCY;
	}
	if (moved_too_early()) {
		return FAULT_MOVED_BEFORE_3S;
	}
	return FAULT_NONE;
}

void monitoring_update(uint32_t now_ms) {
	acu.monitor.active = acu.mode == ACU_MODE_AUTONOMOUS && checkup_passed()
			&& acu.as.finish != FINISH_EBS_ACTIVATED && !acu.fault.active;

	if (!acu.monitor.active) {
		/* timers start counting only while monitoring */
		acu.monitor.ts_off_since_ms = now_ms;
		acu.mission.mismatch_since_ms = now_ms;
		return;
	}
	fault_reason_t fault = find_fault(now_ms);
	if (fault != FAULT_NONE) {
		acu.monitor.last_detected = fault;
		fault_raise(fault, now_ms);
	}
}
