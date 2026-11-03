/*
 * APP.h
 *
 * One control cycle of the ACU, in order. Called every 1 ms by the control task
 * (acu_tasks.c) and by the host tests (tests/host).
 */
#ifndef INC_APP_H_
#define INC_APP_H_

#include "acu.h"

void app_init(uint32_t now_ms);
void app_control_step(uint32_t now_ms);

#endif /* INC_APP_H_ */
