/*
 * Host stand-in for Drivers/24FC08/ee24.h: sim.c emulates the 1 KB EEPROM.
 */
#ifndef EE24_HOST_STUB_H
#define EE24_HOST_STUB_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define EE24_ADDRESS_DEFAULT 0xA0

typedef struct {
	I2C_HandleTypeDef *HI2c;
	uint8_t Address;
	uint8_t Lock;
} EE24_HandleTypeDef;

bool EE24_Init(EE24_HandleTypeDef *Handle, I2C_HandleTypeDef *HI2c, uint8_t I2CAddress);
bool EE24_Read(EE24_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, size_t Len, uint32_t Timeout);
bool EE24_Write(EE24_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, size_t Len, uint32_t Timeout);

#endif
