/*
 * APP.c
 *
 *  Created on: Apr 25, 2026
 *      Author: bruno
 */
#include "APP.h"
#include "acu_mode.h"
#include "can_rx.h"
#include "can_tx.h"
#include "inputs.h"
#include "mission.h"
#include "outputs.h"

void app_init(uint32_t now_ms) {
	acu_init(now_ms);
	inputs_init();
}

void app_control_step(uint32_t now_ms) {
	acu.system.uptime_ms = now_ms;
	acu.system.control_cycles++;

	inputs_read_pins();
	can_rx_update_node_ages(now_ms);
	mission_update();
	acu_mode_update(now_ms);
	outputs_update(now_ms);
	outputs_write(now_ms);
	can_tx_update(now_ms);
}
