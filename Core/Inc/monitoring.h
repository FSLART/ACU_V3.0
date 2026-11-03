/*
 * monitoring.h
 *
 * Continuous monitoring, T 15.3.2: from the end of the initial checkup until the
 * EBS is activated, every cycle. The first problem found raises a fault.
 */
#ifndef MONITORING_H
#define MONITORING_H

#include "acu.h"

void monitoring_update(uint32_t now_ms);

#endif /* MONITORING_H */
