/*
 * ebs.c
 */
#include "ebs.h"
#include "checkup.h"

static bool in_range(float value, float min, float max) {
	return value > min && value < max;
}

bool ebs_tank_pressures_ok(void) {
	return in_range(acu.inputs.front_tank_bar, EBS_FRONT_TANK_MIN_BAR, EBS_TANK_MAX_BAR)
			&& in_range(acu.inputs.rear_tank_bar, EBS_REAR_TANK_MIN_BAR, EBS_TANK_MAX_BAR);
}

bool ebs_front_brake_engaged(void) {
	return acu.brakes.front_bar >= EBS_FRONT_HYD_GAIN * acu.inputs.front_tank_bar;
}

bool ebs_rear_brake_engaged(void) {
	return acu.brakes.rear_bar >= EBS_REAR_HYD_GAIN * acu.inputs.rear_tank_bar;
}

bool ebs_both_brakes_engaged(void) {
	return ebs_front_brake_engaged() && ebs_rear_brake_engaged();
}

bool ebs_front_brake_released(void) {
	return acu.brakes.front_bar <= BRAKE_RELEASED_MAX_BAR;
}

bool ebs_rear_brake_released(void) {
	return acu.brakes.rear_bar <= BRAKE_RELEASED_MAX_BAR;
}

/* A bypassed sensor follows the solenoid, so it can never show the brakes released by hand. */
bool ebs_both_brakes_released(void) {
	bool front = BYPASS_FRONT_HYD_PRESSURE || ebs_front_brake_released();
	bool rear = BYPASS_REAR_HYD_PRESSURE || ebs_rear_brake_released();
	return front && rear;
}

ebs_state_t ebs_state(void) {
	if (acu.as.flowchart.ebs_activated) {
		return EBS_STATE_ACTIVATED;
	}
	if (checkup_passed()) {
		return EBS_STATE_CHECKUP_PASSED;
	}
	return EBS_STATE_DEACTIVATED;
}
