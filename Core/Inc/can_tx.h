/*
 * can_tx.h
 *
 * Frames the ACU sends every CAN_TX_PERIOD_MS: ACU (0x51, read by the Jetson and the VCU),
 * DV_status (0x502, data logger) and ASF_SIGNALS (0x511). Also the one-shot
 * CANopen NMT start that wakes the RES up when the AS starts.
 */
#ifndef CAN_TX_H
#define CAN_TX_H

#include "acu.h"

void can_tx_update(uint32_t now_ms);

#endif /* CAN_TX_H */
