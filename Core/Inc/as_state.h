/*
 * as_state.h
 *
 * AS status, T 14.8: the state shown on the ASSIs and sent on CAN is always the
 * result of the flowchart of Figure 16 (as_flowchart). This module keeps its inputs:
 *   - EBS activated (T 14.8.1 / 14.8.2),
 *   - R2D, only accepted after 5 s in AS Ready (T 14.8.4),
 *   - mission finished + standstill, then the ACU activates the EBS (AS Finished),
 *   - vehicle speed from VCU_RPM.
 */
#ifndef AS_STATE_H
#define AS_STATE_H

#include "acu.h"

void as_reset(uint32_t now_ms);
void as_update(uint32_t now_ms);

/* Figure 16, nothing else. */
AS_STATE_t as_flowchart(const as_flowchart_inputs_t *in);

bool as_vehicle_at_standstill(void);

#endif /* AS_STATE_H */
