/*
 * can_rx.c
 */
#include "can_rx.h"
#include "autonomous_t26.h"
#include <string.h>

static void node_heard(can_node_t *node, uint32_t now_ms) {
	node->last_rx_ms = now_ms;
	node->age_ms = 0;
	node->rx_count++;
}

static void handle_front_brake(const can_frame_t *frame, uint32_t now_ms) {
	struct autonomous_t26_aqt1_t msg;
	if (autonomous_t26_aqt1_unpack(&msg, frame->data, frame->length) != 0) {
		return;
	}
#if BYPASS_FRONT_HYD_PRESSURE
	acu.brakes.front_bar = acu.outputs.front_ebs_released ? BYPASS_HYD_UNLOADED_BAR : BYPASS_HYD_LOADED_BAR;
#else
	acu.brakes.front_bar = (float)autonomous_t26_aqt1_frt_brk_press_decode(msg.frt_brk_press);
#endif
	acu.brakes.aqt1_res_bit = msg.res;
	acu.brakes.aqt1_bots_bit = msg.bots;
	node_heard(&acu.nodes.brake_front, now_ms);
}

static void handle_rear_brake(const can_frame_t *frame, uint32_t now_ms) {
	struct autonomous_t26_aqt7_t msg;
	if (autonomous_t26_aqt7_unpack(&msg, frame->data, frame->length) != 0) {
		return;
	}
#if BYPASS_REAR_HYD_PRESSURE
	acu.brakes.rear_bar = acu.outputs.rear_ebs_released ? BYPASS_HYD_UNLOADED_BAR : BYPASS_HYD_LOADED_BAR;
#else
	acu.brakes.rear_bar = (float)autonomous_t26_aqt7_rear_brk_press_decode(msg.rear_brk_press);
#endif
	node_heard(&acu.nodes.brake_rear, now_ms);
}

static void handle_vcu_ignition(const can_frame_t *frame, uint32_t now_ms) {
	struct autonomous_t26_vcu_ign_r2_d_t msg;
	if (autonomous_t26_vcu_ign_r2_d_unpack(&msg, frame->data, frame->length) != 0) {
		return;
	}
	acu.vcu.ts_active = autonomous_t26_vcu_ign_r2_d_ignition_auto_decode(msg.ignition_auto) != 0;
	acu.vcu.shutdown_signal = (uint8_t)autonomous_t26_vcu_ign_r2_d_shutdown_signal_decode(msg.shutdown_signal);
	acu.vcu.vcu_state = (uint8_t)autonomous_t26_vcu_ign_r2_d_vcu_state_decode(msg.vcu_state);
	node_heard(&acu.nodes.vcu, now_ms);
}

static void handle_vcu_rpm(const can_frame_t *frame, uint32_t now_ms) {
	struct autonomous_t26_vcu_rpm_t msg;
	if (autonomous_t26_vcu_rpm_unpack(&msg, frame->data, frame->length) != 0) {
		return;
	}
	acu.vcu.motor_rpm_left = (uint16_t)autonomous_t26_vcu_rpm_motor_rpm_left_decode(msg.motor_rpm_left);
	acu.vcu.motor_rpm_right = (uint16_t)autonomous_t26_vcu_rpm_motor_rpm_right_decode(msg.motor_rpm_right);
	node_heard(&acu.nodes.vcu_rpm, now_ms);
}

static void handle_jetson(const can_frame_t *frame, uint32_t now_ms) {
	struct autonomous_t26_jetson_t msg;
	if (autonomous_t26_jetson_unpack(&msg, frame->data, frame->length) != 0) {
		return;
	}
	acu.jetson.as_state = (uint8_t)autonomous_t26_jetson_as_state_decode(msg.as_state);
	acu.jetson.mission = (uint8_t)autonomous_t26_jetson_as_mission_decode(msg.as_mission);
	acu.jetson.temperature = (uint8_t)autonomous_t26_jetson_temperature_decode(msg.temperature);
	acu.jetson.cpu = (uint8_t)autonomous_t26_jetson_cpu_decode(msg.cpu);
	acu.jetson.gpu = (uint8_t)autonomous_t26_jetson_gpu_decode(msg.gpu);
	acu.jetson.emergency_cause = (uint8_t)autonomous_t26_jetson_emergency_cause_decode(msg.emergency_cause);
	node_heard(&acu.nodes.jetson, now_ms);
}

static void handle_slam_stats(const can_frame_t *frame) {
	struct autonomous_t26_slam_stats_can_t msg;
	if (autonomous_t26_slam_stats_can_unpack(&msg, frame->data, frame->length) != 0) {
		return;
	}
	acu.jetson.lap_counter = (uint8_t)autonomous_t26_slam_stats_can_lap_counter_decode(msg.lap_counter);
	acu.jetson.cones_count_actual =
			(uint8_t)autonomous_t26_slam_stats_can_cones_count_actual_decode(msg.cones_count_actual);
	acu.jetson.cones_count_all = (uint32_t)autonomous_t26_slam_stats_can_cones_count_all_decode(msg.cones_count_all);
}

static void handle_res(const can_frame_t *frame, uint32_t now_ms) {
	memcpy(acu.res.payload, frame->data, sizeof(acu.res.payload));
	acu.res.length = frame->length;
	node_heard(&acu.nodes.res, now_ms);
}

/* ENTER_BOOT: acu_mode.c resets into the bootloader if the car is safe. */
static void handle_bootloader_entry(const can_frame_t *frame) {
	if (frame->length == 8 && frame->data[0] == 1) {
		acu.system.bootloader_requested = true;
	}
}

void can_rx_handle_frame(const can_frame_t *frame, uint32_t now_ms) {
	if (frame->extended) {
		if (frame->id == AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID) {
			node_heard(&acu.nodes.steering, now_ms);
		}
		return;
	}
	switch (frame->id) {
	case AUTONOMOUS_T26_AQT1_FRAME_ID:
		handle_front_brake(frame, now_ms);
		break;
	case AUTONOMOUS_T26_AQT7_FRAME_ID:
		handle_rear_brake(frame, now_ms);
		break;
	case AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID:
		handle_vcu_ignition(frame, now_ms);
		break;
	case AUTONOMOUS_T26_VCU_RPM_FRAME_ID:
		handle_vcu_rpm(frame, now_ms);
		break;
	case AUTONOMOUS_T26_JETSON_FRAME_ID:
		handle_jetson(frame, now_ms);
		break;
	case AUTONOMOUS_T26_SLAM_STATS_CAN_FRAME_ID:
		handle_slam_stats(frame);
		break;
	case RES_PDO_FRAME_ID:
		handle_res(frame, now_ms);
		break;
	case BOOTLOADER_ENTER_FRAME_ID:
		handle_bootloader_entry(frame);
		break;
	default:
		break;
	}
}

static void update_age(can_node_t *node, uint32_t now_ms) {
	node->age_ms = now_ms - node->last_rx_ms;
}

void can_rx_update_node_ages(uint32_t now_ms) {
	update_age(&acu.nodes.vcu, now_ms);
	update_age(&acu.nodes.vcu_rpm, now_ms);
	update_age(&acu.nodes.jetson, now_ms);
	update_age(&acu.nodes.res, now_ms);
	update_age(&acu.nodes.steering, now_ms);
	update_age(&acu.nodes.brake_front, now_ms);
	update_age(&acu.nodes.brake_rear, now_ms);
}
