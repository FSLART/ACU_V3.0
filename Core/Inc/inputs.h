/*
 * inputs.h
 *
 * The ACU's own inputs: pins (every control cycle), the EBS tank pressure sensors
 * and chip temperature (ADC interrupt) and the mission button (EXTI interrupt).
 */
#ifndef INPUTS_H
#define INPUTS_H

#include "acu.h"

void inputs_init(void);
void inputs_read_pins(void);

/* Interrupt context */
void inputs_on_adc_conversion(uint32_t front_tank_raw, uint32_t rear_tank_raw, float chip_temp_c);
void inputs_on_mission_button(uint32_t now_ms);

float inputs_tank_bar_from_adc(uint32_t raw);

#endif /* INPUTS_H */
