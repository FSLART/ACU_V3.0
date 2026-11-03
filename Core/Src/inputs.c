/*
 * inputs.c
 */
#include "inputs.h"
#include "EMA_Filter.h"

/* EBS tank pressure sensors: 0.5 V at 0 bar, 0.4 V per bar, divided down to the ADC. */
#define ADC_FULL_SCALE         4096.0f
#define ADC_REFERENCE_V        3.3f
#define TANK_DIVIDER_RATIO     0.66f
#define TANK_SENSOR_ZERO_V     0.5f
#define TANK_SENSOR_V_PER_BAR  0.4f
#define TANK_FILTER_ALPHA      0.5f

static ema_data_structure front_tank_filter;
static ema_data_structure rear_tank_filter;

void inputs_init(void) {
	ema_init(&front_tank_filter, TANK_FILTER_ALPHA);
	ema_init(&rear_tank_filter, TANK_FILTER_ALPHA);
}

void inputs_read_pins(void) {
	acu.inputs.asms_on = HAL_GPIO_ReadPin(ASMS_GPIO_Port, ASMS_Pin) == GPIO_PIN_SET;
	acu.inputs.sdc_closed = HAL_GPIO_ReadPin(SDC_FEEDBACK_GPIO_Port, SDC_FEEDBACK_Pin) == GPIO_PIN_RESET;
	acu.inputs.ignition_button = HAL_GPIO_ReadPin(IGN_BTN_GPIO_Port, IGN_BTN_Pin) == GPIO_PIN_SET;
}

float inputs_tank_bar_from_adc(uint32_t raw) {
	float sensor_volts = ((float)raw * ADC_REFERENCE_V / ADC_FULL_SCALE) / TANK_DIVIDER_RATIO;
	return (sensor_volts - TANK_SENSOR_ZERO_V) / TANK_SENSOR_V_PER_BAR;
}

void inputs_on_adc_conversion(uint32_t front_tank_raw, uint32_t rear_tank_raw, float chip_temp_c) {
	acu.inputs.front_tank_bar = ema_update(&front_tank_filter, inputs_tank_bar_from_adc(front_tank_raw));
	acu.inputs.rear_tank_bar = ema_update(&rear_tank_filter, inputs_tank_bar_from_adc(rear_tank_raw));
	acu.inputs.chip_temp_c = chip_temp_c;
}

void inputs_on_mission_button(uint32_t now_ms) {
	if (time_elapsed(acu.inputs.mission_button_last_ms, now_ms, MISSION_BUTTON_DEBOUNCE_MS)) {
		acu.inputs.mission_button_presses++;
		acu.inputs.mission_button_last_ms = now_ms;
	}
}
