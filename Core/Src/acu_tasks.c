/*
 * acu_tasks.c
 */
#include "acu_tasks.h"
#include "APP.h"
#include "acu_system.h"
#include "adc.h"
#include "ble_handler.h"
#include "logger.h"
#include "tim.h"
#include "cmsis_os.h"

#define CONTROL_PERIOD_MS      1
#define TELEMETRY_PERIOD_MS    10
#define LOGGER_PERIOD_MS       20
#define CONTROL_LATE_MS        2

extern CAN_HandleTypeDef hcan1;
extern uint32_t ADC_Samples[4];

extern osMessageQId canRxQueueHandle; /* freertos.c, from the .ioc */
static volatile uint32_t can_rx_dropped = 0;

void acu_hardware_start(void) {
	app_init(HAL_GetTick());
	system_capture_reset_cause();
	HAL_ADC_Start_DMA(&hadc1, ADC_Samples, 4);
	HAL_TIM_Base_Start(&htim8); /* ADC trigger */
	system_watchdog_start();
}

void acu_tasks_can_frame_from_isr(const can_frame_t *frame) {
	BaseType_t higher_priority_woken = pdFALSE;

	if (canRxQueueHandle == NULL) {
		return; /* MX_FREERTOS_Init() not run yet */
	}
	if (xQueueSendFromISR(canRxQueueHandle, frame, &higher_priority_woken) != pdTRUE) {
		can_rx_dropped++;
	}
	portYIELD_FROM_ISR(higher_priority_woken);
}

void ControlTask(void const *argument) {
	(void)argument;
	TickType_t wake_time = xTaskGetTickCount();
	uint32_t last_cycle_ms = HAL_GetTick();
	uint32_t last_service_ms = last_cycle_ms;

	for (;;) {
		vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
		uint32_t now = HAL_GetTick();

		if (now - last_cycle_ms > CONTROL_LATE_MS) {
			acu.system.control_late++;
		}
		last_cycle_ms = now;

		can_frame_t frame;
		while (xQueueReceive(canRxQueueHandle, &frame, 0) == pdTRUE) {
			can_rx_handle_frame(&frame, now);
		}

		app_control_step(now);

		if (time_elapsed(last_service_ms, now, CAN_SERVICE_PERIOD_MS)) {
			last_service_ms = now;
			CAN_Service(&hcan1);
			system_watchdog_refresh(); /* only here: a stuck control task resets the board */
			acu.system.stack_free_control = uxTaskGetStackHighWaterMark(NULL);
		}
	}
}

void TelemetryTask(void const *argument) {
	(void)argument;
	ble_handler_init(HAL_GetTick());

	for (;;) {
		ble_handler(HAL_GetTick());
		acu.system.stack_free_telemetry = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
	}
}

void LoggerTask(void const *argument) {
	(void)argument;
	fault_reason_t crash_reason;
	uint32_t crash_detail;

	logger_task_init(acu.system.reset_flags);
	if (system_take_crash(&crash_reason, &crash_detail)) {
		logger_log_crash((uint8_t)crash_reason, crash_detail);
	}

	for (;;) {
		logger_task_step();
		acu.system.stack_free_logger = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelay(pdMS_TO_TICKS(LOGGER_PERIOD_MS));
	}
}

/* configCHECK_FOR_STACK_OVERFLOW = 2 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
	(void)task;
	system_save_crash(FAULT_STACK_OVERFLOW, 0, task_name);
	system_halt_safe();
}

/* configUSE_MALLOC_FAILED_HOOK = 1 */
void vApplicationMallocFailedHook(void) {
	system_save_crash(FAULT_OUT_OF_MEMORY, 0, NULL);
	system_halt_safe();
}
