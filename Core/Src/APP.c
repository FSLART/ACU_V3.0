/*
 * APP.c
 *
 *  Created on: Apr 25, 2026
 *      Author: bruno
 */

#include "APP.h"
#include "ble_handler.h"
#include <string.h>


struct car t24;
struct can_queue can_rx_data;

extern uint32_t ADC_Samples[4];
extern uint8_t mission_selector_enable;
uint8_t prev_ASMS = 0;
uint8_t ASSI_leds_control_signal = 0;
float temporary_temp = 0;

Main_state_machine_t Vehicle_state_machine;
cant_acu_state_t ACU_STATE;
Autonomous_System_states_t Autonomous_state;
startup_sequence_state_t startup_sequence_state;
Emergency_cause_t Emergency_cause = NONE;

extern struct ring can_tx_ringbuffer;
extern struct ring can_rx_ringbuffer;

uint8_t prev_car_state = -1;
uint8_t prev_as_state = -1;

/* RN4871 configuration commands — transparent serial bridge mode */
#define BLE_CFG_NUM_CMDS  3

static const char* ble_cfg_cmds[BLE_CFG_NUM_CMDS] = {
    "SN,ACU_V3\r\n",
    "SR,01\r\n",
    "R,1\r\n",
};

static uint8_t      ble_cfg_state = 0;
static uint8_t      ble_cfg_index = 0;
static uint32_t     ble_cfg_tick  = 0;


void app_init() {

	/***
	 * Var initialization
	 ***/

	t24.Front_Pressure.Hydraulic = 0;
	t24.Front_Pressure.Pneumatic = 0;
	t24.Rear_Pressure.Hydraulic = 0;
	t24.Rear_Pressure.Pneumatic = 0;
	t24.Ignition_Status = 0;
	t24.Ignition_Request = 0;
	t24.front_solenoid = 0;
	t24.rear_solenoid = 0;
	t24.Speed.Speed = 0;
	t24.Speed.Target_Speed = 0;
	t24.Emergency = 0;
	t24.Res = 0;
	t24.Current_Mission = MANUAL;
	t24.Autonomous_State = AS_STATE_OFF;
	t24.HW_WDT_Enable = 1;
	t24.prev_ign_pin_state = 0;
	t24.Ignition_enable = 0;


	Vehicle_state_machine = Start;
	 Autonomous_state = OFF;
	 ACU_STATE = INIT;
	 startup_sequence_state = WDT_TOGGLE_CHECK;


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Samples, 4);

	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_Base_Start_IT(&htim2);


	CAN_FilterTypeDef can_filter;

	can_filter.FilterBank = 0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterIdHigh = 0x0000;
	can_filter.FilterIdLow = 0x0000;
	can_filter.FilterMaskIdHigh = 0x0000;
	can_filter.FilterMaskIdLow = 0x0000;
	can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan1, &can_filter);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);

	extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

	ble_module_config_start();
	ble_handler_init();


}

void app() {
	t24.HW_WDT_Enable
	ble_module_config_tick();
	ble_handler();
	prev_ASMS = t24.ASMS;
	Peripheral_aquisition(&ASSI_leds_control_signal);
	temporary_temp = t24.chip_temp;
	Handle_state(prev_ASMS);
	toggle_wdt();
	//handle_uart_logs();
	LED_indicator_controller();
	ASSI_leds_control_signal = ASSI_control(ASSI_leds_control_signal, t24.Autonomous_State);
	Peripheral_actuation();
	handle_can_tx();
	can_buffer_pop(&can_rx_ringbuffer, 0,&can_rx_data);
	dbc_decode();
	/*if(t24.Autonomous_State == AS_STATE_EMERGENCY){
		t24.Emergency = 1;
	}*/
}


void ble_module_config_start(void) {
    ble_cfg_state = 1;
    ble_cfg_index = 0;
    ble_cfg_tick  = HAL_GetTick();
}

void ble_module_config_tick(void) {
    uint32_t now = HAL_GetTick();

    switch (ble_cfg_state) {

    case 0: /* idle */
    case 5: /* done */
        break;

    case 1: /* ENTER_CMD: send $$$ */
        HAL_UART_Transmit(&huart1, (uint8_t*)"$$$", 3, 10);
        ble_cfg_tick = now;
        ble_cfg_state = 2;
        break;

    case 2: /* WAIT_ENTER: wait 500ms after $$$ for CMD> prompt */
        if (now - ble_cfg_tick >= 500) {
            ble_cfg_index = 0;
            ble_cfg_tick = now;
            ble_cfg_state = 3;
        }
        break;

    case 3: /* SEND_CMD: send next configuration command */
        if (ble_cfg_index < BLE_CFG_NUM_CMDS) {
            HAL_UART_Transmit(&huart1,
                              (uint8_t*)ble_cfg_cmds[ble_cfg_index],
                              strlen(ble_cfg_cmds[ble_cfg_index]), 10);
            ble_cfg_tick = now;
            ble_cfg_state = 4;
        } else {
            ble_cfg_state = 5;   /* all done */
        }
        break;

    case 4: /* WAIT_CMD: wait 500ms between commands for AOK response */
        if (now - ble_cfg_tick >= 500) {
            ble_cfg_index++;
            ble_cfg_tick = now;
            ble_cfg_state = 3;
        }
        break;
    }
}


uint8_t ble_module_config_is_done(void) {
   // return (ble_cfg_state == 5);
	ble_cfg_state == 5;
	return ble_cfg_state;
}


void dbc_decode(){
	switch(can_rx_data.can_rx_header.StdId){
	case AUTONOMOUS_T26_AQT7_FRAME_ID:
		struct autonomous_t26_aqt7_t rear_dynamics;
		autonomous_t26_aqt7_unpack(&rear_dynamics, can_rx_data.tx_data, AUTONOMOUS_T26_AQT7_LENGTH);
		t24.Rear_Pressure.Hydraulic = autonomous_t26_aqt7_rear_brk_press_decode(rear_dynamics.rear_brk_press);
		t24.REAR_PRESSURE_LAST_TX = can_rx_ringbuffer.queue[can_rx_ringbuffer.tail].arrival_time;
		break;
	case AUTONOMOUS_T26_AQT1_FRAME_ID:
			struct autonomous_t26_aqt1_t front_dynamics;
			autonomous_t26_aqt1_unpack(&front_dynamics, can_rx_data.tx_data, AUTONOMOUS_T26_AQT1_LENGTH);
#if BYPASS_FRONT_HYD_PRESSURE
			// TEST BYPASS: ignore the CAN reading, synthesize a value consistent with
			// rear_solenoid (which physically locks/releases the front line) instead.
			t24.Front_Pressure.Hydraulic = t24.rear_solenoid
					? BYPASS_FRONT_HYD_PRESSURE_UNLOADED
					: BYPASS_FRONT_HYD_PRESSURE_LOADED;
#else
			t24.Front_Pressure.Hydraulic = autonomous_t26_aqt1_frt_brk_press_decode(front_dynamics.frt_brk_press);
#endif
			//t24.REAR_PRESSURE_LAST_TX = can_rx_ringbuffer.queue[can_rx_ringbuffer.tail].arrival_time;
			break;
	case AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID:
		struct autonomous_t26_vcu_ign_r2_d_t vcu_data;
		autonomous_t26_vcu_ign_r2_d_unpack(&vcu_data, can_rx_data.tx_data, AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH);
		t24.Ignition_Status = autonomous_t26_vcu_ign_r2_d_ignition_auto_decode(vcu_data.ignition_auto);
		t24.vcu_sdc = autonomous_t26_vcu_ign_r2_d_shutdown_signal_decode(vcu_data.shutdown_signal);
		t24.VCU_LAST_TX = HAL_GetTick();
		break;
	case AUTONOMOUS_T26_JETSON_FRAME_ID:
		struct autonomous_t26_jetson_t jetson_data;
		autonomous_t26_jetson_unpack(&jetson_data, can_rx_data.tx_data,AUTONOMOUS_T26_JETSON_LENGTH);
		{
			volatile uint8_t decoded_state = autonomous_t26_jetson_as_state_decode(jetson_data.as_state);
			if (decoded_state == AS_STATE_DRIVING || decoded_state == AS_STATE_FINISHED || decoded_state == AS_STATE_EMERGENCY) {
				t24.Autonomous_State = decoded_state;
			}
		}
		t24.Jetson_mission = autonomous_t26_jetson_as_mission_decode(jetson_data.as_mission);
		t24.JETSON_LAST_TX = HAL_GetTick();
		break;

	case AUTONOMOUS_T26_VCU_RPM_FRAME_ID:
		struct autonomous_t26_vcu_rpm_t vcu_rpm;
		autonomous_t26_vcu_rpm_unpack(&vcu_rpm,can_rx_data.tx_data,AUTONOMOUS_T26_VCU_RPM_LENGTH);
		t24.rpm = autonomous_t26_vcu_rpm_rpm_actual_decode(vcu_rpm.rpm_actual);
		break;
		case AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID:
			t24.DIR_ACTUATOR_LAST_TX = HAL_GetTick();
			break;
		case AUTONOMOUS_T26_RES_FRAME_ID:
			t24.RES_LAST_TX = HAL_GetTick();
			break;
	default:
		break;
	}
}
