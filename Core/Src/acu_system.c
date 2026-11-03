/*
 * acu_system.c
 */
#include "acu_system.h"
#include <string.h>

#define CRASH_RECORD_ADDRESS 0x2003FFE0u
#define CRASH_MAGIC          0xC2A5BEEFu

typedef struct {
	uint32_t magic;
	uint32_t reason;     /* fault_reason_t */
	uint32_t detail;     /* HardFault: PC */
	char task[16];
} crash_record_t;

#define CRASH_RECORD ((volatile crash_record_t *)CRASH_RECORD_ADDRESS)

/* Factory calibration of the temperature sensor, measured at VDDA = 3.3 V. */
#define TS_CAL1_ADDR    ((uint16_t *)0x1FFF7A2C)  /* raw at 30 degC */
#define TS_CAL2_ADDR    ((uint16_t *)0x1FFF7A2E)  /* raw at 110 degC */
#define TS_CAL1_TEMP_C  30.0f
#define TS_CAL2_TEMP_C  110.0f

void system_capture_reset_cause(void) {
	acu.system.reset_flags = RCC->CSR;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		acu.system.reset_cause = "watchdog interno (IWDG)";
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
		acu.system.reset_cause = "WWDG";
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
		acu.system.reset_cause = "software";
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
		acu.system.reset_cause = "low-power";
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		acu.system.reset_cause = "power-on";
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
		acu.system.reset_cause = "brown-out";
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		acu.system.reset_cause = "pino NRST";
	} else {
		acu.system.reset_cause = "?";
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

/* ~2 s (LSI 32 kHz / 64 * 1000). Register access: the HAL IWDG module is not enabled. */
void system_watchdog_start(void) {
	__HAL_DBGMCU_FREEZE_IWDG(); /* don't reset the board while halted at a breakpoint */
	IWDG->KR = 0xCCCC;          /* start, forces LSI on */
	IWDG->KR = 0x5555;          /* unlock PR and RLR */
	IWDG->PR = IWDG_PR_PR_2;    /* /64 */
	IWDG->RLR = 1000;
	uint32_t start = HAL_GetTick();
	while (IWDG->SR != 0 && HAL_GetTick() - start < 10) {
	}
	system_watchdog_refresh();
}

void system_watchdog_refresh(void) {
	IWDG->KR = 0xAAAA;
}

void system_save_crash(fault_reason_t reason, uint32_t detail, const char *task_name) {
	CRASH_RECORD->magic = CRASH_MAGIC;
	CRASH_RECORD->reason = (uint32_t)reason;
	CRASH_RECORD->detail = detail;
	for (size_t i = 0; i < sizeof(CRASH_RECORD->task); i++) {
		char c = task_name ? task_name[i] : '\0';
		CRASH_RECORD->task[i] = c;
		if (c == '\0') {
			break;
		}
	}
	CRASH_RECORD->task[sizeof(CRASH_RECORD->task) - 1] = '\0';
}

/* EBS engaged; nothing runs any more, so the watchdog pulses stop and the external
 * watchdog opens the SDC within ~100 ms. The IWDG then resets the board. */
void system_halt_safe(void) {
	__disable_irq();
	HAL_GPIO_WritePin(Front_Solenoid_GPIO_Port, Front_Solenoid_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rear_Solenoid_GPIO_Port, Rear_Solenoid_Pin, GPIO_PIN_RESET);
	if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
		__BKPT(0); /* debugger attached: stop here */
	}
	for (;;) {
	}
}

bool system_take_crash(fault_reason_t *reason, uint32_t *detail) {
	if (CRASH_RECORD->magic != CRASH_MAGIC) {
		return false;
	}
	*reason = (fault_reason_t)CRASH_RECORD->reason;
	*detail = CRASH_RECORD->detail;
	for (size_t i = 0; i < sizeof(acu.system.stack_overflow_task); i++) {
		acu.system.stack_overflow_task[i] = CRASH_RECORD->task[i];
	}
	CRASH_RECORD->magic = 0;
	return true;
}

/* The bootloader reads the flag after the reset and stays in programming mode. */
void system_reset_to_bootloader(void) {
	*(volatile uint32_t *)BOOTLOADER_FLAG_ADDRESS = BOOTLOADER_MAGIC;
	__DSB();
	NVIC_SystemReset();
}

float system_chip_temperature(uint32_t raw_temp, uint32_t raw_vref) {
	if (raw_vref == 0) {
		return 0.0f;
	}
	/* Real VDDA from the factory VREFINT calibration, then rescale the reading to 3.3 V. */
	float vdda = 3.3f * ((float)(*VREFINT_CAL_ADDR) / (float)raw_vref);
	float raw_at_3v3 = (float)raw_temp * (vdda / 3.3f);
	float cal1 = (float)(*TS_CAL1_ADDR);
	float cal2 = (float)(*TS_CAL2_ADDR);
	return (raw_at_3v3 - cal1) * (TS_CAL2_TEMP_C - TS_CAL1_TEMP_C) / (cal2 - cal1) + TS_CAL1_TEMP_C;
}
