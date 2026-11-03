/*
 * outputs.c
 */
#include "outputs.h"
#include "checkup.h"

#define IGNITION_BUTTON_SAMPLE_MS 100  /* 10 Hz sampling also debounces the button */

static bool autonomous_without_fault(void) {
	return acu.mode == ACU_MODE_AUTONOMOUS && !acu.fault.active;
}

/* EBS solenoid energised = that circuit released. */
static void decide_ebs(void) {
	bool driving = autonomous_without_fault() && acu.as.state == AS_STATE_DRIVING;

	acu.outputs.front_ebs_released = driving || (autonomous_without_fault() && checkup_releases_front_brake());
	acu.outputs.rear_ebs_released = driving || (autonomous_without_fault() && checkup_releases_rear_brake());
}

/* T 14.4.1: the AS may only close the SDC
 *   - for manual driving: manual mission and ASMS off (no autonomous brake actuation possible),
 *   - for autonomous driving: autonomous mission, ASMS on and brakes engaged (checkup order).
 * The external watchdog keeps the SDC closed only while it gets pulses. */
static void decide_sdc(void) {
	bool closed = false;

	switch (acu.mode) {
	case ACU_MODE_IDLE:
		closed = acu.mission.selected == MISSION_MANUAL && !acu.inputs.asms_on;
		break;
	case ACU_MODE_AUTONOMOUS:
		closed = !acu.fault.active && acu.inputs.asms_on && checkup_wants_watchdog_pulses()
				&& acu.as.finish != FINISH_EBS_ACTIVATED;
		break;
	default:
		closed = false; /* STARTUP and EMERGENCY: SDC open */
		break;
	}
	acu.outputs.watchdog_pulses_on = closed;
}

/* The IGN button toggles the TS request sent to the VCU (ACU.ign), only while the ACU allows it. */
static void decide_ts_request(uint32_t now_ms) {
	static uint32_t last_sample_ms = 0;
	static bool button_was_pressed = false;

	acu.outputs.ignition_enable = autonomous_without_fault() && checkup_allows_ts_request()
			&& acu.as.finish != FINISH_EBS_ACTIVATED;

	if (time_elapsed(last_sample_ms, now_ms, IGNITION_BUTTON_SAMPLE_MS)) {
		last_sample_ms = now_ms;
		bool pressed = acu.inputs.ignition_button;
		if (pressed && !button_was_pressed) {
			acu.outputs.ignition_request = !acu.outputs.ignition_request;
		}
		button_was_pressed = pressed;
	}
	if (!acu.outputs.ignition_enable) {
		acu.outputs.ignition_request = false; /* losing the enable drops a request already sent */
	}
}

/* T 14.9.1 */
static void decide_assi(uint32_t now_ms) {
	bool flash_on = (now_ms / ASSI_HALF_PERIOD_MS) % 2 == 0;
	bool yellow = false;
	bool blue = false;

	switch (acu.as.state) {
	case AS_STATE_READY:
		yellow = true;
		break;
	case AS_STATE_DRIVING:
		yellow = flash_on;
		break;
	case AS_STATE_EMERGENCY:
		blue = flash_on;
		break;
	case AS_STATE_FINISHED:
		blue = true;
		break;
	case AS_STATE_OFF:
	default:
		break;
	}
	acu.outputs.assi_yellow = yellow;
	acu.outputs.assi_blue = blue;
}

void outputs_update(uint32_t now_ms) {
	decide_ebs();
	decide_sdc();
	decide_ts_request(now_ms);
	decide_assi(now_ms);
}

static GPIO_PinState level(bool on) {
	return on ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void outputs_write(uint32_t now_ms) {
	static uint32_t last_pulse_ms = 0;
	static uint32_t last_heartbeat_ms = 0;

	HAL_GPIO_WritePin(Front_Solenoid_GPIO_Port, Front_Solenoid_Pin, level(acu.outputs.front_ebs_released));
	HAL_GPIO_WritePin(Rear_Solenoid_GPIO_Port, Rear_Solenoid_Pin, level(acu.outputs.rear_ebs_released));
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, level(!acu.outputs.front_ebs_released));
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, level(!acu.outputs.rear_ebs_released));
	HAL_GPIO_WritePin(ASSI_YELLOW_GPIO_Port, ASSI_YELLOW_Pin, level(acu.outputs.assi_yellow));
	HAL_GPIO_WritePin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin, level(acu.outputs.assi_blue));

	if (acu.outputs.watchdog_pulses_on && time_elapsed(last_pulse_ms, now_ms, WATCHDOG_PULSE_PERIOD_MS)) {
		HAL_GPIO_TogglePin(WDT_PULSE_GPIO_Port, WDT_PULSE_Pin);
		last_pulse_ms = now_ms;
	}
	if (time_elapsed(last_heartbeat_ms, now_ms, HEARTBEAT_LED_PERIOD_MS)) {
		HAL_GPIO_TogglePin(HB_GPIO_Port, HB_Pin);
		last_heartbeat_ms = now_ms;
	}
}
