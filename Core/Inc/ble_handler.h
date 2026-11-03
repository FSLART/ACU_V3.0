/*
 * ble_handler.h
 *
 * Plain-text telemetry over the RN4871 (transparent UART on USART1), readable in any
 * BLE serial terminal on a phone. Runs in the telemetry task, only reads `acu`.
 */
#ifndef INC_BLE_HANDLER_H_
#define INC_BLE_HANDLER_H_

#include "main.h"

/* RN4871 is wired to USART1 (PA9 = BLE_TX, PA10 = BLE_RX). */
extern UART_HandleTypeDef huart1;
#define BLE_UART            (&huart1)

/* Name the phone sees; set with "SN," on every boot (RN4871 limit: 20 characters). */
#define BLE_DEVICE_NAME     "ACU TEK-26e"

#define BLE_RX_BUF_SIZE     256
#define BLE_TX_BUF_SIZE     1024

extern volatile uint8_t ble_tx_busy;
extern char ble_module_reply[128];

void ble_handler_init(uint32_t now_ms);
void ble_handler(uint32_t now_ms);

#endif /* INC_BLE_HANDLER_H_ */
