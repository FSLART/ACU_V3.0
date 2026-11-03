/*
 * faults.h
 *
 * The single way into EMERGENCY: any module that finds a problem calls fault_raise().
 * The first reason is latched with a snapshot of the car, settled for FAULT_SETTLE_MS (a RES stop
 * can show up first as its consequences) and then sent to the logger.
 */
#ifndef FAULTS_H
#define FAULTS_H

#include "acu.h"

void fault_raise(fault_reason_t reason, uint32_t now_ms);
void fault_update(uint32_t now_ms);
void fault_clear(void);

const char *fault_name(fault_reason_t reason);
const char *fault_description(fault_reason_t reason);
can_emergency_cause_t fault_can_cause(fault_reason_t reason);

#endif /* FAULTS_H */
