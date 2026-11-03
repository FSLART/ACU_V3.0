/*
 * mission.c
 */
#include "mission.h"

static mission_t next_mission(mission_t mission) {
	return mission >= MISSION_LAST ? MISSION_MANUAL : (mission_t)(mission + 1);
}

void mission_update(void) {
	uint32_t presses = acu.inputs.mission_button_presses;

	acu.mission.selection_allowed = !acu.inputs.asms_on && acu.mode != ACU_MODE_AUTONOMOUS;

	while (acu.mission.presses_handled != presses) {
		acu.mission.presses_handled++;
		if (acu.mission.selection_allowed) {
			acu.mission.selected = next_mission(acu.mission.selected);
		}
	}
}
