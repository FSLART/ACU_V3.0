/*
 * outputs.h
 *
 * Every output is decided again each cycle from the state machines, never left set
 * by an old state:
 *   - EBS solenoids: released only in AS Driving and in the single-circuit checkup steps,
 *   - SDC (external watchdog pulses): always closed, open only on a fault and for the
 *     checkup watchdog test,
 *   - TS request to the VCU: only during the checkup TS step and AS Ready/Driving,
 *   - ASSIs: T 14.9.1 pattern of the AS state.
 */
#ifndef OUTPUTS_H
#define OUTPUTS_H

#include "acu.h"

void outputs_update(uint32_t now_ms);  /* decide (logic only) */
void outputs_write(uint32_t now_ms);   /* drive the pins */

#endif /* OUTPUTS_H */
