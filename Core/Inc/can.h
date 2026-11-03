/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* Worst to best. */
typedef enum {
	CAN_BUS_OK,
	CAN_BUS_WARNING,      /* TEC or REC >= 96 */
	CAN_BUS_PASSIVE,      /* TEC or REC >= 128 */
	CAN_BUS_OFF,          /* TEC > 255, hardware recovers by itself (AutoBusOff) */
	CAN_BUS_NOT_STARTED   /* HAL state != LISTENING, CAN_Service restarts it */
} CAN_BusState;

/* Same values as ESR.LEC. */
typedef enum {
	CAN_ERR_NONE,
	CAN_ERR_STUFF,
	CAN_ERR_FORM,
	CAN_ERR_ACK,
	CAN_ERR_BIT_RECESSIVE,
	CAN_ERR_BIT_DOMINANT,
	CAN_ERR_CRC
} CAN_LastError;

/* hcan->ErrorCode, one bit per HAL_CAN_ERROR_*. */
typedef union {
	uint32_t raw;
	struct {
		uint32_t ewg : 1;
		uint32_t epv : 1;
		uint32_t bof : 1;
		uint32_t stuff : 1;
		uint32_t form : 1;
		uint32_t ack : 1;
		uint32_t bit_recessive : 1;
		uint32_t bit_dominant : 1;
		uint32_t crc : 1;
		uint32_t rx_fifo0_overrun : 1;
		uint32_t rx_fifo1_overrun : 1;
		uint32_t tx_arb_lost0 : 1;
		uint32_t tx_error0 : 1;
		uint32_t tx_arb_lost1 : 1;
		uint32_t tx_error1 : 1;
		uint32_t tx_arb_lost2 : 1;
		uint32_t tx_error2 : 1;
		uint32_t timeout : 1;
		uint32_t not_initialized : 1;
		uint32_t not_ready : 1;
		uint32_t not_started : 1;
		uint32_t param : 1;
		uint32_t invalid_callback : 1;
		uint32_t internal : 1;
	} bit;
} CAN_HalError;

/* Diagnostics only: add `can1_status` to Live Expressions. */
typedef struct {
	CAN_BusState state;
	const char *state_text;
	CAN_LastError last_error;
	const char *last_error_text;
	uint32_t last_error_ms;
	uint8_t tec, rec;
	uint8_t rx_pin;               /* CAN_RX (PB8) at the last service: 1 = recessive, 0 = dominant */
	uint32_t rx_dominant_ms;      /* time since CAN_RX was last seen recessive; > 100 ms = bus stuck dominant */
	uint8_t fault;
	uint32_t fault_count;
	CAN_HalError error_last;
	CAN_HalError error_seen;
	/* 10 ms service windows in which ESR.LEC showed each error */
	uint32_t stuff, form, ack, bit_recessive, bit_dominant, crc;
	uint32_t bus_off;
	uint32_t tx_arb_lost, tx_error;
	uint32_t tx_queued, tx_dropped, tx_aborted;
	uint32_t restarts;
} CAN_BusStatus;

extern CAN_BusStatus can1_status;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef CAN_Config(CAN_HandleTypeDef *hcan);
void CAN_Service(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *header, const uint8_t data[8]);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

