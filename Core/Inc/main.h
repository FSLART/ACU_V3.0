/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define RX_BUFFER_SIZE 32
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef __attribute__((packed)) struct {
    uint8_t  state_machine;       // 0  — Vehicle_state_machine
    uint8_t  assi_status;         // 1  — t24.ASSI_state
    uint8_t  mission;             // 2  — t24.Current_Mission
    uint16_t hydraulic_p1;        // 3-4 LE — t24.Front_Pressure.Hydraulic * 100
    uint16_t hydraulic_p2;        // 5-6 LE — t24.Rear_Pressure.Hydraulic * 100
    uint16_t pneumatic_p1;        // 7-8 LE — t24.Front_Pressure.Pneumatic * 100
    uint16_t pneumatic_p2;        // 9-10 LE — t24.Rear_Pressure.Pneumatic * 100
    int16_t  chip_temp;           // 11-12 LE — t24.chip_temp * 100
    uint8_t  solenoid_front;      // 13 — t24.front_solenoid
    uint8_t  solenoid_rear;       // 14 — t24.rear_solenoid
} ble_telemetry_packet_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WDT_PULSE_Pin GPIO_PIN_0
#define WDT_PULSE_GPIO_Port GPIOC
#define SDC_FEEDBACK_Pin GPIO_PIN_1
#define SDC_FEEDBACK_GPIO_Port GPIOC
#define HB_Pin GPIO_PIN_5
#define HB_GPIO_Port GPIOC
#define CAN_INDICATO_Pin GPIO_PIN_0
#define CAN_INDICATO_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define Rear_Solenoid_Pin GPIO_PIN_12
#define Rear_Solenoid_GPIO_Port GPIOB
#define Front_Solenoid_Pin GPIO_PIN_13
#define Front_Solenoid_GPIO_Port GPIOB
#define ASSI_YELLOW_Pin GPIO_PIN_14
#define ASSI_YELLOW_GPIO_Port GPIOB
#define ASSI_BLUE_Pin GPIO_PIN_15
#define ASSI_BLUE_GPIO_Port GPIOB
#define MS1_Pin GPIO_PIN_6
#define MS1_GPIO_Port GPIOC
#define MS2_Pin GPIO_PIN_7
#define MS2_GPIO_Port GPIOC
#define MS3_Pin GPIO_PIN_8
#define MS3_GPIO_Port GPIOC
#define MS4_Pin GPIO_PIN_9
#define MS4_GPIO_Port GPIOC
#define MS5_Pin GPIO_PIN_8
#define MS5_GPIO_Port GPIOA
#define ASMS_Pin GPIO_PIN_2
#define ASMS_GPIO_Port GPIOD
#define MS_BTN_Pin GPIO_PIN_3
#define MS_BTN_GPIO_Port GPIOB
#define MS_BTN_EXTI_IRQn EXTI3_IRQn
#define IGN_BTN_Pin GPIO_PIN_4
#define IGN_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


#define CAN_MSG_MAX_TIMEOUT 1000

typedef enum {
	AS_STATE_OFF = 1,       // 1
	AS_STATE_READY = 2,     // 2
	AS_STATE_DRIVING = 3,   // 3
	AS_STATE_EMERGENCY = 4, // 4
	AS_STATE_FINISHED = 5   // 5
} AS_STATE_t;

typedef enum {
	MANUAL,       // 0
	ACCELERATION, // 1
	SKIDPAD,      // 2   // 3
	TRACKDRIVE,   // 4
	EBS_TEST,     // 5
	INSPECTION,   // 6
	AUTOCROSS     // 7
} current_mission_t;


typedef enum{
	NONE,
	SDC_OPEN,
	RES,
	Pressure_checks,
	VCU_Timeout,
	Jetson_timeout,
	ACU_WDT_TRIGERED,
	dir_actuator_timeout,
	Dynamics_REAR_Pressure_timeout,
	UNKOWN
}Emergency_cause_t;


typedef enum{
	BLE_IDLE,
	BLE_WAIT_CONFIG,
	BLE_BRIDGE
}BLE_STATE_MACHINE_t;

struct pressure {
	float Pneumatic;
	float Hydraulic;
};

struct speed {
	uint8_t Speed;			// Km/h
	uint8_t Target_Speed;	// Km/h
};

struct car {
	struct pressure Rear_Pressure;
	struct pressure Front_Pressure;
	uint8_t front_solenoid;
	uint8_t rear_solenoid;
	uint8_t Ignition_Status;			// 0 - OFF, 1 - ON     (Real from VCU)
	uint8_t Ignition_Request;		// 0 - OFF, 1 - ON		(Request from ACU)
	uint8_t Ignition_enable;
	uint8_t ASMS;								// 0 - OFF, 1 - ON
	uint8_t Emergency;							// 0 - No , 1 - Emergency
	uint8_t Res;					// 0 - Not active, 1 - ON , 2 - Emergency
	uint8_t HW_WDT_Enable;
	uint8_t ignition_pin_state;
	uint8_t prev_ign_pin_state;
	uint8_t SDC_feedback;
	uint8_t ASSI_state;
	uint8_t vcu_sdc;
	volatile AS_STATE_t Autonomous_State; 		// Autonomous system state
	volatile current_mission_t Current_Mission; // Current mission state
	volatile current_mission_t Jetson_mission;
	struct speed Speed;
	float chip_temp;
	int rpm;
	uint32_t VCU_LAST_TX,REAR_PRESSURE_LAST_TX,JETSON_LAST_TX,DIR_ACTUATOR_LAST_TX,RES_LAST_TX;
};

typedef enum {
	Start, IDLE, AS_ON, EMERGENCY
} Main_state_machine_t;


typedef enum {
	INIT,
	MISSION_SELECT,
	JETSON_WAITING,
	INIT_SEQUENCE,
	READY,
	DRIVING,
	EBS_ERROR,
	ACU_EMERGENCY,
	FINISHED,
	ACU_MANUAL
} cant_acu_state_t;


typedef enum {
	OFF,
	Initial_Sequence,
	Monitor_sequence,
	AS_Emergency,
	Finish

}Autonomous_System_states_t;


typedef enum {
	NO_TIMEOUT,
	VCU_TIMEOUT,
	JETSON_TIMEOUT,
	PRESSURE_TIMEOUT,
	DIR_TIMEOUT,
	RES_TIMEOUT
}can_timeouts_t;


typedef enum {
	WDT_TOGGLE_CHECK,        // 0 — Check SDC open (LOW), disable WDT
	WDT_STP_TOGGLE_CHECK,    // 1 — Wait for SDC closed (HIGH), enable WDT (5000ms timeout)
	PNEUMATIC_CHECK,         // 2 — Check pneumatic 6-10 bar
	PRESSURE_CHECK1,         // 3 — Hyd correlation pre-ignition (Front >= 9xPneu, Rear >= 3.8xPneu)
	HV_ACTIVATION,           // 4 — Enable ignition, wait for confirmation
	PRESSURE_CHECK_FRONT,    // 5 — front_solenoid=1, rear_solenoid=0, check Front correlated & Rear unloaded
	PRESSURE_CHECK_REAR,     // 6 — front_solenoid=0, rear_solenoid=1, check Rear correlated & Front unloaded
	PRESSURE_CHECK2,         // 7 — Both off, final correlation check -> READY
	SEQUENCE_ERROR           // 8 — Error state -> EMERGENCY
} startup_sequence_state_t;


struct can_queue {
	uint32_t TX_MAILBOX;
	CAN_TxHeaderTypeDef can_tx_header;
	CAN_RxHeaderTypeDef can_rx_header;
	uint8_t tx_data[8];
	uint32_t arrival_time;
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
