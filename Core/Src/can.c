/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* A failed HAL_CAN_Start blocks for up to CAN_TIMEOUT_VALUE (10 ms), so retries are spaced out. */
#define CAN_RESTART_MS 100U

CAN_BusStatus can1_status;

static const char *const can_state_text[] = {
	[CAN_BUS_OK]          = "OK",
	[CAN_BUS_WARNING]     = "WARNING: TEC/REC >= 96. Erros no barramento: ruido, terminacao em falta ou a mais, cabo/ficha",
	[CAN_BUS_PASSIVE]     = "PASSIVE: TEC/REC >= 128. Sozinho no barramento, bitrate/sample point diferente, CANH-CANL trocados",
	[CAN_BUS_OFF]         = "BUS-OFF: TEC > 255. O HW recupera sozinho apos 128x11 bits recessivos. Se nao sai daqui: CAN_RX sempre dominante (ver rx_dominant_ms): transceiver sem alimentacao/standby, CANH a VCC ou CANL a GND, no a forcar o barramento",
	[CAN_BUS_NOT_STARTED] = "NOT STARTED: arranque falhou (transceiver sem alimentacao, barramento preso a dominante). Nova tentativa a cada 100 ms",
};

static const char *const can_error_text[] = {
	[CAN_ERR_NONE]          = "nenhum",
	[CAN_ERR_STUFF]         = "STUFF: 6 bits iguais seguidos. Bitrate diferente, ruido, terminacao",
	[CAN_ERR_FORM]          = "FORM: bit de formato fixo errado. Bitrate/sample point diferente, ruido",
	[CAN_ERR_ACK]           = "ACK: ninguem confirmou a trama. Nenhum no ligado, CANH-CANL trocados, bitrate diferente",
	[CAN_ERR_BIT_RECESSIVE] = "BIT RECESSIVE: mandamos recessivo, lemos dominante. Outro no a forcar o barramento, CANH/CANL em curto",
	[CAN_ERR_BIT_DOMINANT]  = "BIT DOMINANT: mandamos dominante, lemos recessivo. O nosso TX nao chega ao barramento (transceiver em standby/sem alimentacao)",
	[CAN_ERR_CRC]           = "CRC: CRC nao bate certo. Ruido, terminacao, sample point diferente",
};

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* A failed start is retried by CAN_Service, never Error_Handler. */
  CAN_Config(&hcan1);

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* Filters + start + RX FIFO0 interrupt. Used at boot and on every restart. */
HAL_StatusTypeDef CAN_Config(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef filter = {
		.FilterBank = 0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterIdHigh = 0x0000,
		.FilterIdLow = 0x0000,
		.FilterMaskIdHigh = 0x0000,   /* mask 0: accept everything */
		.FilterMaskIdLow = 0x0000,
		.FilterFIFOAssignment = CAN_RX_FIFO0,
		.FilterActivation = ENABLE,
		.SlaveStartFilterBank = 14,   /* written to the CAN1/CAN2 shared FMR: banks 0-13 stay on CAN1 */
	};

	if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
		return HAL_ERROR;
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		return HAL_ERROR;
	return HAL_CAN_Start(hcan);
}

/* No HAL_CAN_DeInit: MspDeInit would gate the CAN1 clock and reset the pins. */
static void CAN_Restart(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING)
		HAL_CAN_Stop(hcan);
	if (HAL_CAN_Init(hcan) == HAL_OK)
		CAN_Config(hcan);
}

/* Every 10 ms. Error interrupts (SCE) stay off: everything is polled so a broken bus cannot cause an interrupt storm. */
void CAN_Service(CAN_HandleTypeDef *hcan)
{
	static uint32_t last_restart_ms;
	static uint32_t last_recessive_ms;
	CAN_BusStatus *s = &can1_status;
	uint32_t now = HAL_GetTick();
	uint32_t esr = hcan->Instance->ESR;
	uint32_t tsr = hcan->Instance->TSR;
	uint32_t lec = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;

	s->tec = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
	s->rec = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;

	/* On a working bus a 10 ms sample lands on a recessive bit almost every time */
	s->rx_pin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	if (s->rx_pin)
		last_recessive_ms = now;
	s->rx_dominant_ms = now - last_recessive_ms;

	/* LEC 0 = frame OK, 7 = nothing new since we last set it to 7 */
	if (lec >= CAN_ERR_STUFF && lec <= CAN_ERR_CRC) {
		uint32_t *const count[] = { 0, &s->stuff, &s->form, &s->ack, &s->bit_recessive, &s->bit_dominant, &s->crc };
		(*count[lec])++;
		s->last_error = (CAN_LastError)lec;
		s->last_error_text = can_error_text[lec];
		s->last_error_ms = now;
	}
	hcan->Instance->ESR = CAN_ESR_LEC;

	/* ALST/TERR are only valid once the mailbox request completed (RQCP) */
	uint32_t rqcp = 0;
	for (uint32_t mb = 0; mb < 3; mb++) {
		uint32_t shift = 8U * mb;
		if (tsr & (CAN_TSR_RQCP0 << shift)) {
			rqcp |= CAN_TSR_RQCP0 << shift;
			if (tsr & (CAN_TSR_ALST0 << shift))
				s->tx_arb_lost++;
			if (tsr & (CAN_TSR_TERR0 << shift))
				s->tx_error++;
		}
	}
	if (rqcp)
		hcan->Instance->TSR = rqcp;

	CAN_BusState state;
	if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING)
		state = CAN_BUS_NOT_STARTED;
	else if (esr & CAN_ESR_BOFF)
		state = CAN_BUS_OFF;
	else if (esr & CAN_ESR_EPVF)
		state = CAN_BUS_PASSIVE;
	else if (esr & CAN_ESR_EWGF)
		state = CAN_BUS_WARNING;
	else
		state = CAN_BUS_OK;

	if (state == CAN_BUS_OFF && s->state != CAN_BUS_OFF)
		s->bus_off++;
	if (state != CAN_BUS_OK && s->state == CAN_BUS_OK)
		s->fault_count++;
	s->state = state;
	s->state_text = can_state_text[state];
	s->fault = state != CAN_BUS_OK;
	if (s->last_error_text == NULL)
		s->last_error_text = can_error_text[CAN_ERR_NONE];

	if (hcan->ErrorCode != HAL_CAN_ERROR_NONE) {
		s->error_last.raw = hcan->ErrorCode;
		s->error_seen.raw |= hcan->ErrorCode;
	}

	if (state == CAN_BUS_NOT_STARTED) {
		if (now - last_restart_ms >= CAN_RESTART_MS) {
			last_restart_ms = now;
			s->restarts++;
			CAN_Restart(hcan);
		}
	} else {
		HAL_CAN_ResetError(hcan);
	}
}

/* Never blocks, never Error_Handler. A stale frame with the same ID is aborted so only the newest value goes out. */
HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *header, const uint8_t data[8])
{
	CAN_BusStatus *s = &can1_status;
	uint32_t mailbox;

	if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING) {
		s->tx_dropped++;
		return HAL_ERROR;
	}

	for (uint32_t mb = 0; mb < 3; mb++) {
		uint32_t tir = hcan->Instance->sTxMailBox[mb].TIR;
		if ((hcan->Instance->TSR & (CAN_TSR_TME0 << mb)) == 0
				&& (tir & CAN_TI0R_IDE) == 0
				&& (tir >> CAN_TI0R_STID_Pos) == header->StdId) {
			HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 << mb);
			s->tx_aborted++;
		}
	}

	if (HAL_CAN_AddTxMessage(hcan, header, data, &mailbox) != HAL_OK) {
		s->tx_dropped++;
		return HAL_ERROR;
	}
	s->tx_queued++;
	return HAL_OK;
}

/* USER CODE END 1 */
