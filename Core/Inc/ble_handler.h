/*
 * ble_handler.h
 *
 *  Created on: May 25, 2026
 *      Author: Bruno Vicente
 */

#ifndef INC_BLE_HANDLER_H_
#define INC_BLE_HANDLER_H_
#include "main.h"
#include "stm32f4xx_hal.h"

/* RN4871 is wired to USART1 (PA9 = BLE_TX, PA10 = BLE_RX). */
extern UART_HandleTypeDef huart1;
#define BLE_UART            (&huart1)

/* Name the phone sees; set with "SN," on every boot (RN4871 limit: 20 characters). */
#define BLE_DEVICE_NAME     "ACU TEK-26e"

#define BLE_RX_BUF_SIZE     256
#define BLE_TX_BUF_SIZE     512

#define TIMEOUT_CMD_MS      1000

extern volatile uint8_t ble_tx_busy;

void ble_handler_init(void);
void ble_handler(void);

#endif /* INC_BLE_HANDLER_H_ */
