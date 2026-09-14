/*
 * hardware_abstaction.c
 *
 *  Created on: Mar 23, 2026
 *      Author: bruno
 */


#include "hardware_abstraction.h"

extern int can_queue_index;
extern struct can_queue can_tx_queue[64];
static uint8_t UART_TxBuffer[512];

extern uint32_t TX_MAILBOX;
extern CAN_TxHeaderTypeDef can_tx_header;
extern uint8_t tx_data[8];


uint32_t millis() {

#if defined (__ARM_ARCH_7EM__)

	return HAL_GetTick();

#else
	struct timespec ts;
	    clock_gettime(CLOCK_MONOTONIC, &ts);
	    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif

}



void Peripheral_aquisition(uint8_t *assi_leds) {
	extern uint8_t mission_selector_enable;
	t24.ASMS = HAL_GPIO_ReadPin(ASMS_GPIO_Port, ASMS_Pin);
	t24.SDC_feedback = !HAL_GPIO_ReadPin(SDC_FEEDBACK_GPIO_Port,
	SDC_FEEDBACK_Pin);
	t24.ignition_pin_state = HAL_GPIO_ReadPin(IGN_BTN_GPIO_Port, IGN_BTN_Pin);
	*assi_leds = HAL_GPIO_ReadPin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin) << 1
			| HAL_GPIO_ReadPin(ASSI_YELLOW_GPIO_Port, ASSI_YELLOW_Pin);



	mission_selector_enable = !t24.ASMS;
}

void Peripheral_actuation() {
	HAL_GPIO_WritePin(Front_Solenoid_GPIO_Port, Front_Solenoid_Pin,
			t24.front_solenoid);
	HAL_GPIO_WritePin(Rear_Solenoid_GPIO_Port, Rear_Solenoid_Pin,
			t24.rear_solenoid);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
					!t24.rear_solenoid);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
					!t24.front_solenoid);

	extern uint8_t ASSI_leds_control_signal;
	HAL_GPIO_WritePin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin, ASSI_leds_control_signal & 0b00000010);
	HAL_GPIO_WritePin(ASSI_YELLOW_GPIO_Port, ASSI_YELLOW_Pin, ASSI_leds_control_signal & 0b00000001);
}




void handle_can_tx() {
	static uint8_t tx_index = 0;
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && can_queue_index > -1) {
		HAL_CAN_AddTxMessage(&hcan1, &can_tx_queue[tx_index].can_tx_header,
				can_tx_queue[tx_index].tx_data,
				&can_tx_queue[tx_index].TX_MAILBOX);
		tx_index++;
		if (can_queue_index == (tx_index - 1)) {
			can_queue_index = -1;
			tx_index = 0;
		}
	}

}

void add_can_message(uint32_t mailbox, CAN_TxHeaderTypeDef tx_header,
		uint8_t tx_data[8]) {
	if (can_queue_index >= 63) {
		return;  /* queue full, drop message */
	}
	can_queue_index++;
	can_tx_queue[can_queue_index].TX_MAILBOX = mailbox;
	can_tx_queue[can_queue_index].can_tx_header = tx_header;
	memcpy(can_tx_queue[can_queue_index].tx_data, tx_data, 8);
}


void handle_uart_logs() {
	static unsigned long timestamp = 0;

	if (millis() - timestamp > 1000) {




		int len =
				snprintf(UART_TxBuffer, sizeof(UART_TxBuffer),
						"Chip temperature:%.2f\n\rRear pressure:%.2f\n\rFront Pressure:%.2f\n\r",
						t24.chip_temp, t24.Rear_Pressure.Pneumatic, t24.Front_Pressure.Pneumatic);

		HAL_UART_Transmit_DMA(&huart1, UART_TxBuffer, len);
		//HAL_UART_Transmit(&huart1, UART_TxBuffer, len, 500);
		timestamp = millis();
	}
}

void LED_indicator_controller() {
	static unsigned long timestamp = 0;

	if (millis() - timestamp >= 1000) {
		HAL_GPIO_TogglePin(HB_GPIO_Port, HB_Pin);
		timestamp = millis();
	}

}
