/*
 * state_machine.h
 *
 *  Created on: Apr 25, 2026
 *      Author: bruno
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "main.h"
#include "APP.h"

extern Main_state_machine_t Vehicle_state_machine;
extern Autonomous_System_states_t Autonomous_state;
extern startup_sequence_state_t startup_sequence_state;
extern Emergency_cause_t Emergency_cause;

void Handle_state(uint8_t prev_asms_state);
void Handle_autonomous_state();
void Handle_Emergency();
void toggle_wdt();
void update_ignition_request(void);



#endif /* INC_STATE_MACHINE_H_ */
