/*
 * checkup.h
 *
 * Initial checkup, T 15.3.1: before AS Ready the ACU proves that the EBS can brake
 * and that its watchdog can open the SDC. Order (see checkup_step_t in acu.h):
 *
 *   TANK_PRESSURE -> BRAKES_ENGAGED -> SDC_CLOSE -> WATCHDOG_OPENS_SDC -> SDC_RECLOSE
 *   -> TS_ACTIVATION -> FRONT_BRAKE -> REAR_BRAKE -> BOTH_BRAKES -> PASSED
 *
 * Any failing step goes to FAILED and raises a fault with the reason.
 */
#ifndef CHECKUP_H
#define CHECKUP_H

#include "acu.h"

void checkup_start(uint32_t now_ms);
void checkup_reset(void);
void checkup_update(uint32_t now_ms);

bool checkup_passed(void);
bool checkup_wants_watchdog_pulses(void);  /* T 14.4.1: may the SDC be closed in this step? */
bool checkup_allows_ts_request(void);
bool checkup_releases_front_brake(void);
bool checkup_releases_rear_brake(void);

#endif /* CHECKUP_H */
