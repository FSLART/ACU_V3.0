/*
 * acu_system.h
 *
 * Board-level services (target only, the host tests replace them):
 * independent watchdog, reset cause, crash record kept across a reset,
 * reset into the CAN bootloader and the safe halt used by the fault handlers.
 *
 * The last 32 bytes of RAM are left out of the linker map (STM32F412RETX_FLASH.ld):
 *   0x2003FFE0  crash record (28 bytes)
 *   0x2003FFFC  bootloader request flag (BOOTLOADER_FLAG_ADDRESS)
 */
#ifndef ACU_SYSTEM_H
#define ACU_SYSTEM_H

#include "acu.h"

void system_capture_reset_cause(void);
void system_watchdog_start(void);
void system_watchdog_refresh(void);

/* Fault handlers: remember why, put the outputs in the safe state, wait for the IWDG reset. */
void system_save_crash(fault_reason_t reason, uint32_t detail, const char *task_name);
void system_halt_safe(void);

/* At boot, once: the crash saved by the previous run, if any. */
bool system_take_crash(fault_reason_t *reason, uint32_t *detail);

void system_reset_to_bootloader(void);

float system_chip_temperature(uint32_t raw_temp, uint32_t raw_vref);

#endif /* ACU_SYSTEM_H */
