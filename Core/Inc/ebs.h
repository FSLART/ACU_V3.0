/*
 * ebs.h
 *
 * What the EBS pressures mean: tanks in range, brakes engaged, brakes released.
 * Used by the initial checkup, the continuous monitoring and the AS status.
 */
#ifndef EBS_H
#define EBS_H

#include "acu.h"

bool ebs_tank_pressures_ok(void);

bool ebs_front_brake_engaged(void);   /* hydraulic >= gain x tank pressure */
bool ebs_rear_brake_engaged(void);
bool ebs_both_brakes_engaged(void);

bool ebs_front_brake_released(void);  /* hydraulic <= BRAKE_RELEASED_MAX_BAR */
bool ebs_rear_brake_released(void);
bool ebs_both_brakes_released(void);

ebs_state_t ebs_state(void);          /* DV_status.ASB_EBS_state */

#endif /* EBS_H */
