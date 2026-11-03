/*
 * acu_mode.h
 *
 * Main state machine of the ACU (acu.mode):
 *
 *   STARTUP -> IDLE                       at boot
 *   IDLE -> AUTONOMOUS                    ASMS switched on, autonomous mission, IGN button released
 *   AUTONOMOUS -> IDLE                    ASMS off during the checkup, or AS Finished ended
 *                                         (ASMS off and brakes released)
 *   any -> EMERGENCY                      a fault was raised (faults.c)
 *   EMERGENCY -> IDLE                     ASMS off and the EBS no longer activated
 *
 * It also runs the checkup, the monitoring and the AS status every cycle.
 */
#ifndef ACU_MODE_H
#define ACU_MODE_H

#include "acu.h"

void acu_mode_update(uint32_t now_ms);

#endif /* ACU_MODE_H */
