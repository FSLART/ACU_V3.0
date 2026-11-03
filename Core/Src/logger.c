/*
 * logger.c
 */
#include "logger.h"
#include "acu.h"
#include "ee24.h"
#include <string.h>

#define EEPROM_TIMEOUT_MS 100

extern I2C_HandleTypeDef hi2c1;
static EE24_HandleTypeDef eeprom;

static log_header_t header;
static log_record_t recent[LOG_MAX_RECORDS];  /* RAM copy of the EEPROM ring, oldest first */

/* Single producer (control task) / single consumer (logger task) queue. */
static log_record_t queue[LOG_QUEUE_SIZE];
static volatile uint8_t queue_write = 0;
static volatile uint8_t queue_read = 0;

static uint8_t next_index(uint8_t index) {
	return (uint8_t)((index + 1) % LOG_QUEUE_SIZE);
}

static void queue_push(const log_record_t *record) {
	uint8_t next = next_index(queue_write);
	if (next == queue_read) {
		acu.log.dropped++;
		return;
	}
	queue[queue_write] = *record;
	__sync_synchronize(); /* record complete before the logger task can see it */
	queue_write = next;
}

void logger_log_fault(void) {
	log_record_t record = {
		.uptime_ms = acu.fault.since_ms,
		.front_tank_bar = acu.fault.front_tank_bar,
		.rear_tank_bar = acu.fault.rear_tank_bar,
		.front_brake_bar = acu.fault.front_brake_bar,
		.rear_brake_bar = acu.fault.rear_brake_bar,
		.boot_number = header.boot_number,
		.event = LOG_EVENT_FAULT,
		.reason = (uint8_t)acu.fault.reason,
		.detected_as = (uint8_t)acu.fault.detected_as,
		.mode = (uint8_t)acu.fault.mode,
		.as_state = (uint8_t)acu.fault.as_state,
		.checkup_step = (uint8_t)acu.fault.checkup_step,
		.mission = (uint8_t)acu.mission.selected,
		.sdc_closed = acu.fault.sdc_closed,
		.asms_on = acu.fault.asms_on,
		.ts_active = acu.fault.ts_active,
	};
	queue_push(&record);
}

static void remember(const log_record_t *record) {
	if (acu.log.stored < LOG_MAX_RECORDS) {
		recent[acu.log.stored++] = *record;
	} else {
		memmove(&recent[0], &recent[1], sizeof(recent) - sizeof(recent[0]));
		recent[LOG_MAX_RECORDS - 1] = *record;
	}
}

static uint32_t slot_address(uint8_t slot) {
	return LOG_RECORDS_ADDR + (uint32_t)slot * sizeof(log_record_t);
}

static bool write_header(void) {
	return EE24_Write(&eeprom, LOG_HEADER_ADDR, (uint8_t *)&header, sizeof(header), EEPROM_TIMEOUT_MS);
}

static void load_records(void) {
	uint8_t oldest = (uint8_t)((header.next + LOG_MAX_RECORDS - header.count) % LOG_MAX_RECORDS);
	for (uint8_t i = 0; i < header.count; i++) {
		log_record_t record;
		uint8_t slot = (uint8_t)((oldest + i) % LOG_MAX_RECORDS);
		if (EE24_Read(&eeprom, slot_address(slot), (uint8_t *)&record, sizeof(record), EEPROM_TIMEOUT_MS)) {
			remember(&record);
		}
	}
}

void logger_task_init(uint32_t reset_flags) {
	acu.log.eeprom_ok = EE24_Init(&eeprom, &hi2c1, EE24_ADDRESS_DEFAULT)
			&& EE24_Read(&eeprom, LOG_HEADER_ADDR, (uint8_t *)&header, sizeof(header), EEPROM_TIMEOUT_MS);

	if (acu.log.eeprom_ok) {
		if (header.magic != LOG_MAGIC || header.next >= LOG_MAX_RECORDS || header.count > LOG_MAX_RECORDS) {
			memset(&header, 0, sizeof(header));
			header.magic = LOG_MAGIC;
		}
		header.boot_number++;
		acu.log.eeprom_ok = write_header();
		load_records();
	}

	log_record_t boot = {
		.uptime_ms = acu.system.uptime_ms,
		.detail = reset_flags,
		.boot_number = header.boot_number,
		.event = LOG_EVENT_BOOT,
		.mission = (uint8_t)acu.mission.selected,
	};
	queue_push(&boot);
}

void logger_log_crash(uint8_t reason, uint32_t detail) {
	log_record_t record = {
		.uptime_ms = 0,
		.detail = detail,
		.boot_number = (uint16_t)(header.boot_number - 1), /* it happened in the previous run */
		.event = LOG_EVENT_FAULT,
		.reason = reason,
	};
	queue_push(&record);
}

void logger_task_step(void) {
	while (queue_read != queue_write) {
		log_record_t record = queue[queue_read];
		__sync_synchronize(); /* copy done before the slot is handed back */
		queue_read = next_index(queue_read);

		remember(&record);
		if (!acu.log.eeprom_ok) {
			continue; /* RAM copy only: still visible on Bluetooth until power-off */
		}
		bool ok = EE24_Write(&eeprom, slot_address(header.next), (uint8_t *)&record, sizeof(record),
				EEPROM_TIMEOUT_MS);
		if (ok) {
			header.next = (uint8_t)((header.next + 1) % LOG_MAX_RECORDS);
			if (header.count < LOG_MAX_RECORDS) {
				header.count++;
			}
			ok = write_header();
		}
		if (ok) {
			acu.log.written++;
		} else {
			acu.log.write_errors++;
		}
	}
}

uint8_t logger_record_count(void) {
	return acu.log.stored;
}

const log_record_t *logger_record(uint8_t index) {
	return index < acu.log.stored ? &recent[index] : NULL;
}
