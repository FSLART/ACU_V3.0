#include "state_machine.h"
#include "main.h"

/* ── Mission-mismatch debounce state (file-scope so it can be reset on re-entry) ── */
static uint32_t mismatch_tick = 0;
static uint8_t mismatch_active = 0;
static uint32_t ready_tick = 0;
static uint8_t ready_timer_active = 0;
uint8_t activate_res = 0;
extern cant_acu_state_t ACU_STATE;
uint32_t pressure_recovery_time = 0;

void Handle_autonomous_state() {
	switch (Autonomous_state) {
	case Initial_Sequence:
		mismatch_active = 0; /* reset mismatch debounce on fresh startup cycle */
		initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
		if (t24.Autonomous_State == AS_STATE_READY) {
			if (!ready_timer_active) {
				ready_timer_active = 1;
				ready_tick = millis();
			} else if (millis() - ready_tick >= 500) {
				Autonomous_state = Monitor_sequence;
				pressure_recovery_time = millis();
				ready_timer_active = 0;
			}
		} else {
			ready_timer_active = 0;
		}
		break;
	case Monitor_sequence:
		if (t24.Autonomous_State == AS_STATE_DRIVING) {
			ACU_STATE = DRIVING;
			t24.front_solenoid = 1;
			t24.rear_solenoid = 1;
			//TODO METER DELAY DE 500MS PARA DAR TEMPO DE CORRELACIONAR
		}

		if (millis() - pressure_recovery_time >= 600) {
			continuous_monitoring(t24.SDC_feedback, t24.Rear_Pressure.Pneumatic, t24.Front_Pressure.Pneumatic,
					t24.Rear_Pressure.Hydraulic, t24.Front_Pressure.Hydraulic);
		}

		if (t24.Current_Mission != t24.Jetson_mission) {
			if (!mismatch_active) {
				mismatch_active = 1;
				mismatch_tick = millis();
			} else if (millis() - mismatch_tick >= 1000) {
				Vehicle_state_machine = EMERGENCY;
			}
		} else {
			mismatch_active = 0; /* mission match restored -> reset debounce */
		}
		if (t24.Autonomous_State == AS_STATE_FINISHED) {
			Autonomous_state = Finish;
			ACU_STATE = FINISHED;
		}
		break;
	case Finish:
		/***
		 * Ensure car stoped safely
		 */
		if (t24.rpm <= 10) {
			if (t24.ASMS == 0) {
				t24.ASSI_state = AS_STATE_FINISHED;
				t24.front_solenoid = 0;
				t24.rear_solenoid = 0;
				if (!t24.ASMS) {
					Vehicle_state_machine = IDLE;
				}
			}
		}
		break;
	case AS_Emergency:
		Vehicle_state_machine = EMERGENCY;
		ACU_STATE = ACU_EMERGENCY;
		break;
	default:
		Vehicle_state_machine = EMERGENCY;
		break;
	}
}

void Handle_Emergency() {
	t24.Ignition_enable = 0;
	t24.HW_WDT_Enable = 0;
	t24.Ignition_Request = 0;
	t24.Emergency = 1;
	HAL_GPIO_WritePin(Front_Solenoid_GPIO_Port, Front_Solenoid_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rear_Solenoid_GPIO_Port, Rear_Solenoid_Pin, GPIO_PIN_RESET);
	t24.front_solenoid = 0;
	t24.rear_solenoid = 0;
}

void Handle_state(uint8_t prev_asms_state) {
	static uint8_t as_on_first_time = 0;
	switch (Vehicle_state_machine) {
	case Start:
		t24.HW_WDT_Enable = 1;
		t24.Ignition_enable = 0;
		Vehicle_state_machine = IDLE;
		break;
	case IDLE:
		t24.Ignition_enable = 0;
		as_on_first_time = 0;
		ACU_STATE = MISSION_SELECT;
		if (t24.ASMS == 1 && prev_asms_state == 0 && t24.ignition_pin_state == 0) {
			activate_res = 1;
			Vehicle_state_machine = AS_ON;
			as_on_first_time = 0;
		}
		break;
	case AS_ON:
		if (!as_on_first_time) {
			startup_sequence_state = WDT_TOGGLE_CHECK;
			Autonomous_state = Initial_Sequence;
			as_on_first_time = 1;
		}
		if (!t24.ASMS) {
			Vehicle_state_machine = IDLE;
		}
		Handle_autonomous_state();
		break;
	case EMERGENCY:
		Handle_Emergency();
		if (t24.ASMS == 0 && t24.rpm < 10) {
			Vehicle_state_machine = IDLE;
		}
		break;
	default:
		Vehicle_state_machine = EMERGENCY;
		break;
	}
}

void toggle_wdt() {
	static unsigned long wdt_time = 0;

	if (millis() - wdt_time >= 10) {
		if (t24.HW_WDT_Enable == 1) {
			HAL_GPIO_TogglePin(WDT_PULSE_GPIO_Port, WDT_PULSE_Pin);
			wdt_time = millis();
		}
	}

}

