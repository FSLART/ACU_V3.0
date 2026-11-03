/*
 * checkup.c
 */
#include "checkup.h"
#include "ebs.h"
#include "faults.h"

static void go_to(checkup_step_t step, uint32_t now_ms) {
	acu.checkup.step = step;
	acu.checkup.step_since_ms = now_ms;
	if (step == CHECKUP_PASSED) {
		acu.checkup.passed_count++;
	}
}

static void fail(fault_reason_t reason, uint32_t now_ms) {
	acu.checkup.failed_step = acu.checkup.step;
	acu.checkup.failed_count++;
	fault_raise(reason, now_ms);
	go_to(CHECKUP_FAILED, now_ms);
}

void checkup_start(uint32_t now_ms) {
	go_to(CHECKUP_TANK_PRESSURE, now_ms);
}

void checkup_reset(void) {
	acu.checkup.step = CHECKUP_NOT_STARTED;
}

bool checkup_passed(void) {
	return acu.checkup.step == CHECKUP_PASSED;
}

/* Once the watchdog test is done the SDC must stay closed until the end of the checkup. */
static bool sdc_must_be_closed(void) {
	switch (acu.checkup.step) {
	case CHECKUP_TS_ACTIVATION:
	case CHECKUP_FRONT_BRAKE:
	case CHECKUP_REAR_BRAKE:
	case CHECKUP_BOTH_BRAKES:
		return !SKIP_WDT_CHECK;
	default:
		return false;
	}
}

/* One EBS circuit alone: judged after it had time to settle, failed if it never got there. */
static void check_brake_circuit(bool skip, bool pressures_as_expected, checkup_step_t next,
		fault_reason_t reason, uint32_t now_ms) {
	uint32_t since = acu.checkup.step_since_ms;
	if (skip) {
		go_to(next, now_ms);
	} else if (pressures_as_expected && time_elapsed(since, now_ms, CHECKUP_BRAKE_SETTLE_MS)) {
		go_to(next, now_ms);
	} else if (time_elapsed(since, now_ms, CHECKUP_BRAKE_TIMEOUT_MS + 1)) {
		fail(reason, now_ms);
	}
}

void checkup_update(uint32_t now_ms) {
	uint32_t since = acu.checkup.step_since_ms;
	bool running = acu.checkup.step != CHECKUP_NOT_STARTED && acu.checkup.step != CHECKUP_PASSED
			&& acu.checkup.step != CHECKUP_FAILED;

	if (running && acu.res.stop_pressed) {
		fail(FAULT_RES_STOP, now_ms);
		return;
	}
	if (sdc_must_be_closed() && !acu.inputs.sdc_closed) {
		fail(FAULT_CHECKUP_SDC_OPENED, now_ms);
		return;
	}

	switch (acu.checkup.step) {
	case CHECKUP_NOT_STARTED:
	case CHECKUP_PASSED:
	case CHECKUP_FAILED:
		break;

	case CHECKUP_TANK_PRESSURE:
		if (SKIP_PNEUMATIC_CHECK || ebs_tank_pressures_ok()) {
			go_to(CHECKUP_BRAKES_ENGAGED, now_ms);
		} else {
			fail(FAULT_CHECKUP_TANK_PRESSURE, now_ms);
		}
		break;

	case CHECKUP_BRAKES_ENGAGED:
		if (SKIP_PRESSURE_CHECK1 || ebs_both_brakes_engaged()) {
			go_to(CHECKUP_SDC_CLOSE, now_ms);
		} else if (time_elapsed(since, now_ms, CHECKUP_BRAKES_ENGAGED_TIMEOUT_MS + 1)) {
			fail(FAULT_CHECKUP_BRAKES_NOT_ENGAGED, now_ms);
		}
		break;

	case CHECKUP_SDC_CLOSE:
		if (SKIP_WDT_CHECK) {
			go_to(CHECKUP_TS_ACTIVATION, now_ms);
		} else if (acu.inputs.sdc_closed) {
			go_to(CHECKUP_WATCHDOG_OPENS_SDC, now_ms);
		} else if (time_elapsed(since, now_ms, CHECKUP_SDC_CLOSE_TIMEOUT_MS + 1)) {
			fail(FAULT_CHECKUP_SDC_DID_NOT_CLOSE, now_ms);
		}
		break;

	case CHECKUP_WATCHDOG_OPENS_SDC:
		if (!acu.inputs.sdc_closed) {
			go_to(CHECKUP_SDC_RECLOSE, now_ms);
		} else if (time_elapsed(since, now_ms, CHECKUP_SDC_OPEN_TIMEOUT_MS + 1)) {
			fail(FAULT_CHECKUP_WATCHDOG_DID_NOT_OPEN_SDC, now_ms);
		}
		break;

	case CHECKUP_SDC_RECLOSE:
		if (acu.inputs.sdc_closed) {
			go_to(CHECKUP_TS_ACTIVATION, now_ms);
		} else if (time_elapsed(since, now_ms, CHECKUP_SDC_CLOSE_TIMEOUT_MS + 1)) {
			fail(FAULT_CHECKUP_SDC_DID_NOT_RECLOSE, now_ms);
		}
		break;

	case CHECKUP_TS_ACTIVATION:
		/* No timeout: the ASR switches the TS on with the IGN button when ready. */
		if (SKIP_IGNITION_CHECK || acu.vcu.ts_active) {
			go_to(CHECKUP_FRONT_BRAKE, now_ms);
		}
		break;

	case CHECKUP_FRONT_BRAKE:
		check_brake_circuit(SKIP_PRESSURE_FRONT_CHECK, ebs_front_brake_engaged() && ebs_rear_brake_released(),
				CHECKUP_REAR_BRAKE, FAULT_CHECKUP_FRONT_BRAKE, now_ms);
		break;

	case CHECKUP_REAR_BRAKE:
		check_brake_circuit(SKIP_PRESSURE_REAR_CHECK, ebs_rear_brake_engaged() && ebs_front_brake_released(),
				CHECKUP_BOTH_BRAKES, FAULT_CHECKUP_REAR_BRAKE, now_ms);
		break;

	case CHECKUP_BOTH_BRAKES:
		if (SKIP_PRESSURE_CHECK2 || ebs_both_brakes_engaged()) {
			go_to(CHECKUP_PASSED, now_ms);
		} else if (time_elapsed(since, now_ms, CHECKUP_BRAKE_TIMEOUT_MS + 1)) {
			fail(FAULT_CHECKUP_BOTH_BRAKES, now_ms);
		}
		break;

	default:
		fail(FAULT_INVALID_STATE, now_ms);
		break;
	}
}

bool checkup_wants_watchdog_pulses(void) {
	switch (acu.checkup.step) {
	case CHECKUP_SDC_CLOSE:
	case CHECKUP_SDC_RECLOSE:
	case CHECKUP_TS_ACTIVATION:
	case CHECKUP_FRONT_BRAKE:
	case CHECKUP_REAR_BRAKE:
	case CHECKUP_BOTH_BRAKES:
	case CHECKUP_PASSED:
		return true;
	default:
		return false; /* brakes not proven engaged yet (T 14.4.1), watchdog test, or failed */
	}
}

bool checkup_allows_ts_request(void) {
	switch (acu.checkup.step) {
	case CHECKUP_TS_ACTIVATION:
	case CHECKUP_FRONT_BRAKE:
	case CHECKUP_REAR_BRAKE:
	case CHECKUP_BOTH_BRAKES:
	case CHECKUP_PASSED:
		return true;
	default:
		return false;
	}
}

bool checkup_releases_front_brake(void) {
	return acu.checkup.step == CHECKUP_REAR_BRAKE;
}

bool checkup_releases_rear_brake(void) {
	return acu.checkup.step == CHECKUP_FRONT_BRAKE;
}
