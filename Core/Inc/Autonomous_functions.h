/*
 * Autonomous_functions.h
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#ifndef INC_AUTONOMOUS_FUNCTIONS_H_
#define INC_AUTONOMOUS_FUNCTIONS_H_

#include "main.h"
#include <stdbool.h>
#include "hardware_abstraction.h"

// Per-step skip flags for debugging (set to 1 to skip a step)
#define SKIP_WDT_CHECK        0
#define SKIP_PNEUMATIC_CHECK  0
#define SKIP_PRESSURE_CHECK1  0
#define SKIP_IGNITION_CHECK   0
#define SKIP_PRESSURE_FRONT_CHECK 0
#define SKIP_PRESSURE_REAR_CHECK  0
#define SKIP_PRESSURE_CHECK2  0

// TEST BENCH BYPASS: the front hydraulic circuit bleeds down within ~2s of being
// locked (rear_solenoid==0), so the real CAN reading can't hold long enough to test
// the rest of the pipeline. When enabled, dbc_decode() ignores the CAN value and
// synthesizes a loaded/unloaded reading from rear_solenoid (see APP.c). Set to 0 to
// use the real sensor.
#define BYPASS_FRONT_HYD_PRESSURE         0
#define BYPASS_REAR_HYD_PRESSURE          0

#define BYPASS_FRONT_HYD_PRESSURE_LOADED   150.0f
#define BYPASS_FRONT_HYD_PRESSURE_UNLOADED 0.0f

#define MAX_TIMEOUT 1000

extern Main_state_machine_t Vehicle_state_machine;
extern struct ring can_rx_ringbuffer;
extern Emergency_cause_t Emergency_cause;
extern struct car t24;

void initial_sequence(struct car *v, startup_sequence_state_t *seq_status, Main_state_machine_t *Vehicle_state_machine);
void continuous_monitoring(uint8_t sdc_status, float Rear_pneumatic, float Front_pneumatic, float Rear_hydraulic,
		float Front_hydraulic);
int ASSI_control(uint8_t gpio_state, uint8_t ASSI_state);
bool check_timeout(uint32_t start_time, uint32_t limit);
uint8_t module_timeout();
uint32_t emergency_blame(void);

#endif /* INC_AUTONOMOUS_FUNCTIONS_H_ */
