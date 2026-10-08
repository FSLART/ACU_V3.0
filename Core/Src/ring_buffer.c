/*
 * ring_buffer.c
 *
 *  Created on: Apr 22, 2026
 *      Author: TC-Desenvolvimento
 */

#include "ring_buffer.h"
#include "can.h"


void can_buffer_init(struct ring *ring_buffer) {
	ring_buffer->counter = 0;
	ring_buffer->head = 0;
	ring_buffer->tail = 0;
}

void can_buffer_push(struct ring *ring_buffer, CAN_TxHeaderTypeDef  tx_header,
		uint8_t data[8]) {
	if (ring_buffer->counter >= MAX_SIZE) {
		/* Buffer full — advance tail to discard oldest entry */
		ring_buffer->tail = (ring_buffer->tail + 1) % MAX_SIZE;
	} else {
		ring_buffer->counter++;
	}
	ring_buffer->queue[ring_buffer->head].can_tx_header = tx_header;
	memcpy(ring_buffer->queue[ring_buffer->head].tx_data, data, 8);

	ring_buffer->head = (ring_buffer->head + 1) % MAX_SIZE;
}

void can_rx_buffer_push(struct ring *ring_buffer, CAN_RxHeaderTypeDef  tx_header,
		uint8_t data[8]) {
	if (ring_buffer->counter >= MAX_SIZE) {
		/* Buffer full — advance tail to discard oldest entry */
		ring_buffer->tail = (ring_buffer->tail + 1) % MAX_SIZE;
	} else {
		ring_buffer->counter++;
	}
	ring_buffer->queue[ring_buffer->head].arrival_time = HAL_GetTick();
	ring_buffer->queue[ring_buffer->head].can_rx_header = tx_header;
	memcpy(ring_buffer->queue[ring_buffer->head].tx_data, data, 8);

	ring_buffer->head = (ring_buffer->head + 1) % MAX_SIZE;
}

/* Returns 1 if an entry was consumed. */
uint8_t can_buffer_pop(struct ring *ring_buffer, uint8_t tx_or_rx,struct can_queue *can_rx) {
	if (ring_buffer->counter == 0) {
		return 0;
	}

	if(tx_or_rx){
		if (CAN_Send(&hcan1, &ring_buffer->queue[ring_buffer->tail].can_tx_header,
				ring_buffer->queue[ring_buffer->tail].tx_data) != HAL_OK) {
			return 0;
		}
	}else {
		memcpy(can_rx, &ring_buffer->queue[ring_buffer->tail], sizeof(ring_buffer->queue[ring_buffer->tail]));
        memset(&ring_buffer->queue[ring_buffer->tail], 0, sizeof(ring_buffer->queue[ring_buffer->tail]));
    }

	/* push() runs in interrupt context; its counter++ must not land between this read and write */
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	ring_buffer->tail = (ring_buffer->tail + 1) % MAX_SIZE;
	ring_buffer->counter--;
	__set_PRIMASK(primask);
	return 1;
}
