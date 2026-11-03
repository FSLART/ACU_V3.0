/*
 * can_rx.h
 *
 * Decodes the frames the ACU listens to (autonomous_t26.dbc) into the `acu` struct
 * and keeps the age of every node for the timeouts.
 * The Jetson frame is decoded exactly as before: as_state and mission are only stored here,
 * as_state.c decides what they mean.
 */
#ifndef CAN_RX_H
#define CAN_RX_H

#include "acu.h"

typedef struct {
	uint32_t id;
	bool extended;
	uint8_t length;
	uint8_t data[8];
} can_frame_t;

void can_rx_handle_frame(const can_frame_t *frame, uint32_t now_ms);
void can_rx_update_node_ages(uint32_t now_ms);

#endif /* CAN_RX_H */
