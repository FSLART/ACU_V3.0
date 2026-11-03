/*
 * logger.h
 *
 * Fault log in the 24FC08 EEPROM (1 KB).
 *   - the control task queues a record in RAM (logger_log_fault, never blocks),
 *   - the logger task writes it to the EEPROM ring (logger_task_*, I2C, slow),
 *   - the last records are mirrored in RAM so Bluetooth prints them without I2C.
 * One BOOT record per power-up stores the reset cause (watchdog, brown-out, ...).
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_MAGIC         0xAC0E1065u
#define LOG_HEADER_ADDR   0
#define LOG_RECORDS_ADDR  16          /* one EEPROM page after the header */
#define LOG_MAX_RECORDS   28          /* (1024 - 16) / sizeof(log_record_t) */
#define LOG_QUEUE_SIZE    8

typedef enum {
	LOG_EVENT_BOOT = 1,
	LOG_EVENT_FAULT = 2
} log_event_t;

typedef struct {
	uint32_t magic;
	uint16_t boot_number;
	uint8_t next;                /* slot of the next record */
	uint8_t count;               /* valid records, up to LOG_MAX_RECORDS */
} log_header_t;

typedef struct {
	uint32_t uptime_ms;
	uint32_t detail;             /* BOOT: RCC->CSR reset flags. Crash: PC of the HardFault */
	float front_tank_bar;
	float rear_tank_bar;
	float front_brake_bar;
	float rear_brake_bar;
	uint16_t boot_number;
	uint8_t event;               /* log_event_t */
	uint8_t reason;              /* fault_reason_t */
	uint8_t mode;                /* acu_mode_t */
	uint8_t as_state;
	uint8_t checkup_step;
	uint8_t mission;
	uint8_t sdc_closed;
	uint8_t asms_on;
	uint8_t ts_active;
	uint8_t detected_as;          /* fault_reason_t first seen (RES stop attribution) */
} log_record_t;                  /* 36 bytes */

/* Control task */
void logger_log_fault(void);

/* Logger task, at boot: a crash saved by the previous run (stack overflow, HardFault, ...) */
void logger_log_crash(uint8_t reason, uint32_t detail);

/* Logger task */
void logger_task_init(uint32_t reset_flags);
void logger_task_step(void);

/* Bluetooth: index 0 = oldest */
uint8_t logger_record_count(void);
const log_record_t *logger_record(uint8_t index);

#endif /* LOGGER_H */
