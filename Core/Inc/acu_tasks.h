/*
 * acu_tasks.h
 *
 * FreeRTOS layer of the ACU (target only):
 *   control   1 ms   CAN RX queue -> app_control_step() -> CAN service, IWDG refresh
 *   telemetry 10 ms  Bluetooth (RN4871)
 *   logger    20 ms  fault log to the EEPROM
 * The CAN RX interrupt only copies the frame into a queue for the control task.
 */
#ifndef ACU_TASKS_H
#define ACU_TASKS_H

#include "can_rx.h"

void acu_hardware_start(void);   /* main(), before the scheduler */
void acu_tasks_create(void);     /* MX_FREERTOS_Init() */

void acu_tasks_can_frame_from_isr(const can_frame_t *frame);

#endif /* ACU_TASKS_H */
