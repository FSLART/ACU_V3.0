/*
 * can_tx.c
 */
#include "can_tx.h"
#include "autonomous_t26.h"
#include "ebs.h"
#include "faults.h"

extern CAN_HandleTypeDef hcan1;

/* DV_status.ASB_redundancy_state */
#define REDUNDANCY_DEACTIVATED   1
#define REDUNDANCY_ENGAGED       2
#define REDUNDANCY_CHECKUP_PASSED 3

/* ASF_SIGNALS has 8-bit signals with 0.1 bar resolution: anything above 25.5 bar is sent as 25.5. */
#define ASF_SIGNAL_MAX_BAR 25.5f

static HAL_StatusTypeDef send(uint32_t id, const uint8_t data[8], uint8_t length) {
	CAN_TxHeaderTypeDef header = {
		.StdId = id,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = length,
		.TransmitGlobalTime = DISABLE,
	};
	return CAN_Send(&hcan1, &header, data);
}

static float clamp(float value, float min, float max) {
	if (value < min) {
		return min;
	}
	if (value > max) {
		return max;
	}
	return value;
}

static void send_res_start(void) {
	uint8_t data[8] = { 0x01, 0x00 }; /* NMT "start remote node", node 0 = all nodes */
	if (send(CANOPEN_NMT_FRAME_ID, data, 2) == HAL_OK) {
		acu.outputs.res_start_pending = false;
	}
}

static void send_acu_status(void) {
	struct autonomous_t26_acu_t msg = {
		.acu_cpu_temp = (uint8_t)clamp(acu.inputs.chip_temp_c, 0.0f, 255.0f),
		.as_state = (uint8_t)acu.as.state,
		.asms = acu.inputs.asms_on,
		.assi_state = (uint8_t)acu.as.state,
		.emergency = acu.mode == ACU_MODE_EMERGENCY,
		.ign = acu.outputs.ignition_request,
		.mission_select = (uint8_t)acu.mission.selected,
		.acu_state = (uint8_t)acu.mode,
		.emergency_cause = (uint8_t)(acu.fault.active ? fault_can_cause(acu.fault.reason) : CAN_CAUSE_NONE),
	};
	uint8_t data[8] = { 0 };
	autonomous_t26_acu_pack(data, &msg, sizeof(data));
	send(AUTONOMOUS_T26_ACU_FRAME_ID, data, AUTONOMOUS_T26_ACU_LENGTH);
}

static uint8_t redundancy_state(void) {
	switch (ebs_state()) {
	case EBS_STATE_ACTIVATED:
		return REDUNDANCY_ENGAGED;
	case EBS_STATE_CHECKUP_PASSED:
		return REDUNDANCY_CHECKUP_PASSED;
	default:
		return REDUNDANCY_DEACTIVATED;
	}
}

static void send_dv_status(void) {
	struct autonomous_t26_dv_status_t msg = {
		.as_status = (uint8_t)acu.as.state,
		.asb_ebs_state = (uint8_t)ebs_state(),
		.ami_state = (uint8_t)acu.mission.selected,
		.steering_state = acu.as.state == AS_STATE_DRIVING,
		.asb_redundancy_state = redundancy_state(),
		.lap_counter = acu.jetson.lap_counter,
		.cones_count_actual = acu.jetson.cones_count_actual,
		.cones_count_all = acu.jetson.cones_count_all,
	};
	uint8_t data[8] = { 0 };
	autonomous_t26_dv_status_pack(data, &msg, sizeof(data));
	send(AUTONOMOUS_T26_DV_STATUS_FRAME_ID, data, AUTONOMOUS_T26_DV_STATUS_LENGTH);
}

static void send_asf_signals(void) {
	struct autonomous_t26_asf_signals_t msg = {
		.ebs_pressure_tank_front = autonomous_t26_asf_signals_ebs_pressure_tank_front_encode(
				clamp(acu.inputs.front_tank_bar, 0.0f, ASF_SIGNAL_MAX_BAR)),
		.ebs_pressure_tank_rear = autonomous_t26_asf_signals_ebs_pressure_tank_rear_encode(
				clamp(acu.inputs.rear_tank_bar, 0.0f, ASF_SIGNAL_MAX_BAR)),
		.brake_pressure_front = autonomous_t26_asf_signals_brake_pressure_front_encode(
				clamp(acu.brakes.front_bar, 0.0f, ASF_SIGNAL_MAX_BAR)),
		.brake_pressure_rear = autonomous_t26_asf_signals_brake_pressure_rear_encode(
				clamp(acu.brakes.rear_bar, 0.0f, ASF_SIGNAL_MAX_BAR)),
	};
	uint8_t data[8] = { 0 };
	autonomous_t26_asf_signals_pack(data, &msg, sizeof(data));
	send(AUTONOMOUS_T26_ASF_SIGNALS_FRAME_ID, data, AUTONOMOUS_T26_ASF_SIGNALS_LENGTH);
}

void can_tx_update(uint32_t now_ms) {
	static uint32_t last_tx_ms = 0;

	if (!time_elapsed(last_tx_ms, now_ms, CAN_TX_PERIOD_MS)) {
		return;
	}
	last_tx_ms = now_ms;

	/* One-shot command first, so the periodic frames can't take all three mailboxes. */
	if (acu.outputs.res_start_pending) {
		send_res_start();
	}
	send_acu_status();
	send_dv_status();
	send_asf_signals();
}
