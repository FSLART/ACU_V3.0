/*
 * acu_tasks.h
 *
 * FreeRTOS layer of the ACU (target only). The tasks and the queue are defined in the
 * .ioc (FreeRTOS > Tasks and Queues) and created by CubeMX in freertos.c; their bodies,
 * "As external" in the .ioc, are here:
 *   controlTask    ControlTask    osPriorityHigh         512 words  1 ms: CAN RX queue -> app_control_step()
 *                                                                    -> CAN service, IWDG refresh
 *   telemetryTask  TelemetryTask  osPriorityBelowNormal  768 words  10 ms: Bluetooth (RN4871)
 *   loggerTask     LoggerTask     osPriorityLow          512 words  20 ms: fault log to the EEPROM
 *   canRxQueue     64 x can_frame_t: the CAN RX interrupt only copies the frame here
 * No semaphore or mutex: every resource has one owner (CAN TX: control, USART1: telemetry,
 * I2C EEPROM: logger) and the fault log uses its own single-producer ring (logger.c).
 */
#ifndef ACU_TASKS_H
#define ACU_TASKS_H

#include "can_rx.h"

void acu_hardware_start(void);   /* main(), before the scheduler */

void acu_tasks_can_frame_from_isr(const can_frame_t *frame);

#endif /* ACU_TASKS_H */
