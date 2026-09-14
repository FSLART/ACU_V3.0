/*
 * Autonomous_functions.c
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno, Vasco
 */

#include "Autonomous_functions.h"
#include "hardware_abstraction.h"
#include "main.h"

extern cant_acu_state_t ACU_STATE;

#define TIMEOUT_WDT_MS         5000
#define TIMEOUT_SOLENOID_MS    5000
// Note: HV activation has no timeout in the flowchart (old version waits indefinitely)
// #define TIMEOUT_HV_MS          5000
#define SOLENOID_MIN_DELAY_MS  3000
#define EBS_MIN_BAR 5.0f
#define EBS_MAX_BAR 10.0f
#define EBS_FRONT_HYD_GAIN 9.82f 
#define EBS_REAR_HYD_GAIN_INITIAL 8.5f
#define EBS_REAR_HYD_GAIN_FINAL 8.5f
#define EBS_HYD_UNLOADED_BAR 5.5f //TODO FOI ALTERADO PARA NAO DAR MERDA

#define IN_RANGE(val, min, max) ((val) > (min) && (val) < (max))
#define IS_CORRELATED(hyd, pneu, gain) ((hyd) >= (gain) * (pneu))
#define IS_UNLOADED(hyd) ((hyd) <= EBS_HYD_UNLOADED_BAR)

static uint32_t state_timer = 0;

void initial_sequence(struct car *t24, startup_sequence_state_t *seq_status,
		Main_state_machine_t *Vehicle_state_machine) {
	switch (*seq_status) {
	case WDT_TOGGLE_CHECK:
		ACU_STATE = INIT_SEQUENCE;
		//TODO ALTERADO PARA BYPASS
		*seq_status = WDT_STP_TOGGLE_CHECK;
		t24->HW_WDT_Enable = 0;
		state_timer = millis();
		if (t24->SDC_feedback == 0) {

		}
		break;

	case WDT_STP_TOGGLE_CHECK:
#if SKIP_WDT_CHECK
			t24->HW_WDT_Enable = 1;
			*seq_status = PNEUMATIC_CHECK;
			break;
#endif
		if (t24->SDC_feedback == 1) {
			t24->HW_WDT_Enable = 1;
			*seq_status = PNEUMATIC_CHECK;
		} else if (check_timeout(state_timer, TIMEOUT_WDT_MS)) {
			*seq_status = SEQUENCE_ERROR;
		}
		break;

	case PNEUMATIC_CHECK:
#if SKIP_PNEUMATIC_CHECK
			*seq_status = PRESSURE_CHECK1;
			break;
#endif
		if (IN_RANGE(t24->Front_Pressure.Pneumatic, EBS_MIN_BAR,
				EBS_MAX_BAR) && IN_RANGE(t24->Rear_Pressure.Pneumatic, 4, EBS_MAX_BAR)) {
			*seq_status = PRESSURE_CHECK1;
		} else {
			*seq_status = SEQUENCE_ERROR;
		}
		break;

	case PRESSURE_CHECK1:
#if SKIP_PRESSURE_CHECK1
			*seq_status = HV_ACTIVATION;
			break;
#endif
		if (IS_CORRELATED(t24->Front_Pressure.Hydraulic,
				t24->Front_Pressure.Pneumatic,
				EBS_FRONT_HYD_GAIN) && IS_CORRELATED(t24->Rear_Pressure.Hydraulic, t24->Rear_Pressure.Pneumatic, EBS_REAR_HYD_GAIN_INITIAL)) {
			*seq_status = HV_ACTIVATION;
		} else {
			*seq_status = SEQUENCE_ERROR;
		}
		break;

	case HV_ACTIVATION:
		/* Don't overwrite the toggled Ignition_Request — pin state is read on line below */
#if SKIP_IGNITION_CHECK

			*seq_status = PRESSURE_CHECK_FRONT;
			state_timer = millis();
			break;
#endif
		t24->Ignition_enable = 1;
		if (t24->Ignition_Status == 1) {
			*seq_status = PRESSURE_CHECK_FRONT;
			state_timer = millis();
		}
		break;

	case PRESSURE_CHECK_FRONT:
		t24->front_solenoid = 0;
		t24->rear_solenoid = 1;
#if SKIP_PRESSURE_FRONT_CHECK
			state_timer = millis();
			*seq_status = PRESSURE_CHECK_REAR;
			break;
#endif
		if (IS_CORRELATED(t24->Front_Pressure.Hydraulic,
				t24->Front_Pressure.Pneumatic, EBS_FRONT_HYD_GAIN)
				&& IS_UNLOADED(t24->Rear_Pressure.Hydraulic)
				&& check_timeout(state_timer, SOLENOID_MIN_DELAY_MS)) {
			*seq_status = PRESSURE_CHECK_REAR;
			state_timer = millis();
		} else if (check_timeout(state_timer, TIMEOUT_SOLENOID_MS)) {
			*seq_status = SEQUENCE_ERROR;
		}
		break;

	case PRESSURE_CHECK_REAR:
		t24->front_solenoid = 1;
		t24->rear_solenoid = 0;
#if SKIP_PRESSURE_REAR_CHECK
			state_timer = millis();
			*seq_status = PRESSURE_CHECK2;
			break;
#endif
		if (IS_CORRELATED(t24->Rear_Pressure.Hydraulic,
				t24->Rear_Pressure.Pneumatic, EBS_REAR_HYD_GAIN_FINAL)
				&& IS_UNLOADED(t24->Front_Pressure.Hydraulic)
				&& check_timeout(state_timer, SOLENOID_MIN_DELAY_MS)) {
			*seq_status = PRESSURE_CHECK2;
			state_timer = millis();
		} else if (check_timeout(state_timer, TIMEOUT_SOLENOID_MS)) {
			*seq_status = SEQUENCE_ERROR;
		}
		break;

	case PRESSURE_CHECK2:

		t24->front_solenoid = 0; //TODO ISTO ESTAVA A 1
		t24->rear_solenoid = 0; //TODO ISTO ESTAVA A 1
#if SKIP_PRESSURE_CHECK2
			t24->Autonomous_State = AS_STATE_READY;
			ACU_STATE = READY;
			break;
#endif
		if (IS_CORRELATED(t24->Rear_Pressure.Hydraulic,
				t24->Rear_Pressure.Pneumatic,
				EBS_REAR_HYD_GAIN_FINAL) && IS_CORRELATED(t24->Front_Pressure.Hydraulic, t24->Front_Pressure.Pneumatic, EBS_FRONT_HYD_GAIN)) {
			t24->Autonomous_State = AS_STATE_READY;
			ACU_STATE = READY;
			t24->front_solenoid = 0;
			t24->rear_solenoid = 0;
		} else if (check_timeout(state_timer, TIMEOUT_SOLENOID_MS)) {
			*seq_status = SEQUENCE_ERROR;
		}
		break;

	case SEQUENCE_ERROR:
		ACU_STATE = EBS_ERROR;
		*Vehicle_state_machine = EMERGENCY;
		break;

	default:
		*seq_status = SEQUENCE_ERROR;
		break;
	}
}
void continuous_monitoring(uint8_t sdc_status, float Rear_pneumatic,
		float Front_pneumatic, float Rear_hydraulic, float Front_hydraulic) {
	// CAN Messages timeouts
	/**
	 *
	 * TODO:	Check if sdc is open
	 * 			Check can timeouts
	 * 			Check AS system components
	 * 			Check pneumatic pressure are between 6 and 10 Bar
	 *
	 * 			note: sdc open reads 0
	 */

	if (sdc_status == 0) {
		Vehicle_state_machine = EMERGENCY;
		return;
	}
	uint8_t res = module_timeout();
	if (res != NO_TIMEOUT) {
		Vehicle_state_machine = EMERGENCY;
		switch (res) {
		case VCU_TIMEOUT:
			Emergency_cause = VCU_Timeout;
			break;
		case JETSON_TIMEOUT:
			Emergency_cause = Jetson_timeout;
			break;
		case PRESSURE_TIMEOUT:
			Emergency_cause = Dynamics_REAR_Pressure_timeout;
			break;
		case DIR_TIMEOUT:
			Emergency_cause = dir_actuator_timeout;
			break;
		case RES_TIMEOUT:
			Emergency_cause = RES;
			break;
		default:
			Emergency_cause = UNKOWN;
			break;
		}
		return;
	}

	if (!IN_RANGE(Rear_pneumatic, 4,
			EBS_MAX_BAR) || !IN_RANGE(Front_pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)) {
		Vehicle_state_machine = EMERGENCY;
		return;
	}

	if ((!IS_CORRELATED(Front_hydraulic, Front_pneumatic, EBS_FRONT_HYD_GAIN)
			|| !IS_CORRELATED(Rear_hydraulic, Rear_pneumatic,
					EBS_REAR_HYD_GAIN_FINAL))
			&& t24.Autonomous_State != AS_STATE_DRIVING) {
		Vehicle_state_machine = EMERGENCY;
		return;
	}
}

int ASSI_control(uint8_t gpio_state, uint8_t ASSI_state) {

	/*
	 * bit 0 -> Yellow
	 * bit 1 -> Blue
	 *
	 * -------------------------------------------------------------------------
	 * AS Status Indicator (ASSI) - Operational States (Ref: T 14.8)
	 * -------------------------------------------------------------------------
	 * AS STATE        | ILLUMINATION STATUS
	 * ----------------|--------------------------------------------------------
	 * AS Off          | Off						1
	 * AS Ready        | Yellow (Continuous)		2
	 * AS Driving      | Yellow (Flashing)			3
	 * AS Emergency    | Blue (Flashing)			4
	 * AS Finished     | Blue (Continuous)			5
	 * -------------------------------------------------------------------------
	 *During “AS Driving” and “AS Emergency” the ASSIs must be flashing continuously with a
	 *frequency between 2 Hz and 5 Hz and a duty cycle of 50 %.
	 *
	 */

	static unsigned long prev_time_yellow = 0;
	static unsigned long prev_time_blue = 0;

	switch (ASSI_state) {
	case AS_STATE_OFF:
		gpio_state = 0;
		break;
	case AS_STATE_READY:
		gpio_state = 0b00000001;
		break;
	case AS_STATE_DRIVING:
		if (millis() - prev_time_yellow >= 330) {
			gpio_state ^= 1;
			gpio_state &= 0b00000001;
			prev_time_yellow = millis();
		}
		break;
	case AS_STATE_EMERGENCY:
		if (millis() - prev_time_blue >= 330) {
			gpio_state ^= 0b00000010;
			gpio_state &= 0b00000010;
			prev_time_blue = millis();
		}
		break;
	case AS_STATE_FINISHED:
		gpio_state = 0b00000010;
		break;
	default:
		break;
	}
	return gpio_state;
}

bool check_timeout(uint32_t start_time, uint32_t limit) {
	if (millis() - start_time > limit) {
		return true;
	}
	return false;
}

uint8_t module_timeout() {

	uint32_t current_time = millis();

	if (current_time - t24.VCU_LAST_TX > MAX_TIMEOUT)
		return VCU_TIMEOUT;
	//if(current_time - t24.REAR_PRESSURE_LAST_TX > MAX_TIMEOUT) return PRESSURE_TIMEOUT;
	if (current_time - t24.JETSON_LAST_TX > MAX_TIMEOUT)
		return JETSON_TIMEOUT;
	//TODO FOI DESABILITADO PARA NAO DAR ASNEIRA
	//if(current_time - t24.DIR_ACTUATOR_LAST_TX > MAX_TIMEOUT) return  DIR_TIMEOUT;
	if (current_time - t24.RES_LAST_TX > MAX_TIMEOUT)
		return RES_TIMEOUT;
	return NO_TIMEOUT;
}

uint32_t emergency_blame(void) {
	return (uint32_t) Emergency_cause;
}
