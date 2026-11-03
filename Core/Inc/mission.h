/*
 * mission.h
 *
 * Mission selection with the mission button: each press selects the next mission
 * (MANUAL -> ACCELERATION -> ... -> AUTOCROSS -> MANUAL). Only with the ASMS off,
 * so the mission cannot change while the AS is active.
 */
#ifndef MISSION_H
#define MISSION_H

#include "acu.h"

void mission_update(void);

#endif /* MISSION_H */
