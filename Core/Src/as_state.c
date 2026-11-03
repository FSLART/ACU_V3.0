/*
 * as_state.c
 */
#include "as_state.h"
#include "checkup.h"
#include "ebs.h"
#include "faults.h"
#include <string.h>

/* Figure 16 of the rules, read top to bottom. */
AS_STATE_t as_flowchart(const as_flowchart_inputs_t *in) {
	if (in->ebs_activated) {
		if (!(in->mission_finished && in->vehicle_at_standstill)) {
			return AS_STATE_EMERGENCY;
		}
		if (in->sdc_opened_by_res) {
			return AS_STATE_EMERGENCY;
		}
		return AS_STATE_FINISHED;
	}
	if (in->mission_selected && in->asms_on && in->asb_checks_ok && in->ts_active) {
		if (in->r2d) {
			return AS_STATE_DRIVING;
		}
		if (in->brakes_engaged) {
			return AS_STATE_READY;
		}
	}
	return AS_STATE_OFF;
}

static void set_state(AS_STATE_t state, uint32_t now_ms) {
	if (state != acu.as.state) {
		acu.as.state = state;
		acu.as.state_since_ms = now_ms;
	}
}

void as_reset(uint32_t now_ms) {
	memset(&acu.as.flowchart, 0, sizeof(acu.as.flowchart));
	acu.as.finish = FINISH_NOT_FINISHED;
	set_state(AS_STATE_OFF, now_ms);
}

bool as_vehicle_at_standstill(void) {
	return acu.as.speed_known && !acu.as.moving;
}

/* Vehicle speed from VCU_RPM, debounced so a single noisy frame changes nothing. */
static void update_speed(uint32_t now_ms) {
	bool above_standstill = acu.vcu.motor_rpm_left > STANDSTILL_MAX_RPM
			|| acu.vcu.motor_rpm_right > STANDSTILL_MAX_RPM;

	acu.as.speed_known = acu.nodes.vcu_rpm.rx_count > 0 && acu.nodes.vcu_rpm.age_ms <= VCU_RPM_VALID_MS;

	if (above_standstill == acu.as.moving) {
		acu.as.speed_change_since_ms = now_ms; /* nothing to confirm */
	} else if (time_elapsed(acu.as.speed_change_since_ms, now_ms, STANDSTILL_CONFIRM_MS)) {
		if (above_standstill) {
			acu.as.moving_since_ms = acu.as.speed_change_since_ms;
		}
		acu.as.moving = above_standstill;
	}
}

/* The Jetson asks for AS Driving (after the RES "Go") or reports the mission finished.
 * Its as_state is only a request: the ACU state still comes from the flowchart. */
static void handle_jetson_request(uint32_t now_ms) {
	bool ready = acu.as.state == AS_STATE_READY;
	bool driving = acu.as.state == AS_STATE_DRIVING;

	/* A request from a Jetson that went silent is stale, also with BYPASS_JETSON_TIMEOUT. */
	bool jetson_alive = acu.nodes.jetson.rx_count > 0 && acu.nodes.jetson.age_ms <= JETSON_TIMEOUT_MS;

	if (acu.fault.active || !jetson_alive || !(ready || driving)) {
		return;
	}
	switch (acu.jetson.as_state) {
	case AS_STATE_DRIVING:
		if (ready) {
			if (time_elapsed(acu.as.state_since_ms, now_ms, READY_MIN_TIME_BEFORE_R2D_MS)) {
				acu.as.flowchart.r2d = true;
			} else {
				fault_raise(FAULT_R2D_BEFORE_5S, now_ms);
			}
		}
		break;
	case AS_STATE_FINISHED:
		if (acu.as.finish == FINISH_NOT_FINISHED) {
			acu.as.finish = FINISH_WAITING_FOR_STANDSTILL;
			acu.as.finish_since_ms = now_ms;
		}
		break;
	default:
		break; /* OFF / READY: the Jetson echoing us. EMERGENCY: see monitoring.c */
	}
}

/* AS Finished = mission finished + standstill + EBS activated. At standstill the ACU
 * activates the EBS itself: outputs.c then opens the SDC, engages the EBS and drops the TS request. */
static void update_finish(uint32_t now_ms) {
	if (acu.fault.active) {
		return;
	}
	switch (acu.as.finish) {
	case FINISH_NOT_FINISHED:
		break;
	case FINISH_WAITING_FOR_STANDSTILL:
		if (as_vehicle_at_standstill()) {
			acu.as.finish = FINISH_EBS_ACTIVATED;
			acu.as.finish_since_ms = now_ms;
		} else if (time_elapsed(acu.as.finish_since_ms, now_ms, FINISH_STANDSTILL_TIMEOUT_MS)) {
			fault_raise(FAULT_FINISH_NOT_AT_STANDSTILL, now_ms);
		}
		break;
	case FINISH_EBS_ACTIVATED:
		if (acu.vcu.ts_active && time_elapsed(acu.as.finish_since_ms, now_ms, FINISH_TS_OFF_TIMEOUT_MS)) {
			fault_raise(FAULT_TS_STILL_ON_AFTER_FINISH, now_ms);
		}
		break;
	default:
		fault_raise(FAULT_INVALID_STATE, now_ms);
		break;
	}
}

/* T 14.8.1: activated when the EBS supply is cut after the initial checkup passed.
 * T 14.8.2: no longer activated once all brakes were released at the deactivation points
 * (checked with the ASMS off, so the ACU cannot be holding them). */
static void update_ebs_activated(uint32_t now_ms) {
	as_flowchart_inputs_t *f = &acu.as.flowchart;
	bool supply_cut = !acu.inputs.sdc_closed || !acu.inputs.asms_on || acu.fault.active
			|| acu.as.finish == FINISH_EBS_ACTIVATED;

	if (!f->ebs_activated) {
		f->ebs_activated = checkup_passed() && supply_cut;
		acu.as.brakes_released_since_ms = now_ms;
		return;
	}
	if (acu.inputs.asms_on || !ebs_both_brakes_released()) {
		acu.as.brakes_released_since_ms = now_ms;
	} else if (time_elapsed(acu.as.brakes_released_since_ms, now_ms, BRAKE_RELEASE_CONFIRM_MS)) {
		f->ebs_activated = false;
	}
}

void as_update(uint32_t now_ms) {
	as_flowchart_inputs_t *f = &acu.as.flowchart;

	update_speed(now_ms);
	if (acu.mode == ACU_MODE_AUTONOMOUS) {
		handle_jetson_request(now_ms);
		update_finish(now_ms);
	}
	update_ebs_activated(now_ms);

	f->mission_finished = acu.as.finish != FINISH_NOT_FINISHED;
	/* In AS Finished the EBS holds the car: still at standstill if VCU_RPM stops with the TS off. */
	f->vehicle_at_standstill = as_vehicle_at_standstill() || (acu.as.state == AS_STATE_FINISHED && !acu.as.moving);
	f->sdc_opened_by_res = false; /* RES stop bit not decoded yet: watch acu.res.payload to find it */
	f->mission_selected = acu.mission.selected != MISSION_MANUAL;
	f->asms_on = acu.inputs.asms_on;
	f->asb_checks_ok = checkup_passed();
	f->ts_active = acu.vcu.ts_active || SKIP_IGNITION_CHECK;
	f->brakes_engaged = ebs_both_brakes_engaged();

	AS_STATE_t next = as_flowchart(f);

	/* AS Emergency lasts until the EBS is no longer activated. A fault after the checkup is
	 * always an emergency, also at standstill after the mission (e.g. the TS did not switch off). */
	if (f->ebs_activated && (acu.fault.active || acu.as.state == AS_STATE_EMERGENCY)) {
		next = AS_STATE_EMERGENCY;
	}
	set_state(next, now_ms);
}
