/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "APP.h"
#include <stdio.h>
#include <ctype.h>
#include "Autonomous_functions.h"
#include "hardware_abstraction.h"
#include "autonomous_t26.h"
#include "rn4871.h"
#include "EMA_Filter.h"
#include <stdio.h>
#include "ee24.h"
#include "ble_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint32_t ADC_Samples[4];
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t uart_log_dump = 0;

extern DMA_HandleTypeDef hdma_usart2_rx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TS_CAL1_ADDR        ((uint16_t*)0x1FFF7A2C)   // Raw @ 30°C, Vdda=3.3V
#define TS_CAL2_ADDR        ((uint16_t*)0x1FFF7A2E)   // Raw @ 110°C, Vdda=3.3V
//#define VREFINT_CAL_ADDR    ((uint16_t*)0x1FFF7A2A)   // Raw Vref @ Vdda=3.3V

#define TEMP_CAL1_TEMPC     30.0f
#define TEMP_CAL2_TEMPC     110.0f

uint16_t raw_vref;

struct ring can_tx_ringbuffer;
struct ring can_rx_ringbuffer;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t TX_MAILBOX;
uint8_t tx_data[8];

ema_data_structure ema_rear_pressure;
ema_data_structure ema_front_pressure;

extern uint8_t activate_res;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

struct can_queue can_tx_queue[64];
int can_queue_index = -1;
uint8_t mission_selector_enable = 0;
EE24_HandleTypeDef hee24;

CAN_TxHeaderTypeDef can_tx_header;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

float GetTemperature(uint16_t raw_temp, uint16_t raw_vref);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*) rx_buffer, RX_BUFFER_SIZE);  /* will be overridden by ble_handler_init() */
	uint32_t state = huart2.RxState;  // should be 0x22 (BUSY_RX)
	uint32_t dma_ndtr = hdma_usart2_rx.Instance->NDTR;
	ema_init(&ema_front_pressure, 0.5f);
	ema_init(&ema_rear_pressure, 0.5f);
	app_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		app();
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		float raw_voltage_front = (ADC_Samples[1] * 3.3 / 4096) / 0.66;
		float raw_voltage_rear = (ADC_Samples[0] * 3.3 / 4096) / 0.66;
		t24.chip_temp = GetTemperature(ADC_Samples[2], ADC_Samples[3]);

		float front_pressure = (raw_voltage_front - 0.5) / 0.4;
		float rear_pressure = (raw_voltage_rear - 0.5) / 0.4;

		t24.Front_Pressure.Pneumatic = ema_update(&ema_front_pressure, front_pressure);
		t24.Rear_Pressure.Pneumatic = ema_update(&ema_rear_pressure, rear_pressure);

	}
}

float GetTemperature(uint16_t raw_temp, uint16_t raw_vref) {
	if (raw_vref == 0)
		return 0.0f;

// 1. Calcular VDDA real usando a calibração de fábrica do VREF (3.3V)
// VDDA = 3.3V * (VREFINT_CAL / Raw_VREF)
	float vdda = 3.3f * ((float) (*VREFINT_CAL_ADDR) / (float) raw_vref);

// 2. Ajustar o valor lido da temperatura para a escala de 3.3V
// (A calibração TS_CAL foi feita a 3.3V, se o teu VDDA for diferente, o valor "mexe")
	float raw_equiv_3v3 = (float) raw_temp * (vdda / 3.3f);

// 3. Interpolação Linear
	float ts_cal1 = (float) (*TS_CAL1_ADDR);
	float ts_cal2 = (float) (*TS_CAL2_ADDR);

	float temperature = ((raw_equiv_3v3 - ts_cal1) * (TEMP_CAL2_TEMPC - TEMP_CAL1_TEMPC) / (ts_cal2 - ts_cal1))
			+ TEMP_CAL1_TEMPC;

	return temperature;
}

/***
 * tmr callback
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {

		struct autonomous_t26_acu_t AS_data;

		AS_data.acu_cpu_temp = t24.chip_temp;
		AS_data.as_state = t24.Autonomous_State;
		AS_data.asms = t24.ASMS;
		AS_data.assi_state = t24.ASSI_state;
		AS_data.emergency = t24.Emergency;
		/* Rising-edge toggle: each press flips Ignition_Request */
		if (t24.ignition_pin_state == 1 && t24.prev_ign_pin_state == 0) {
			t24.Ignition_Request = !t24.Ignition_Request;
			t24.Ignition_Request &= t24.Ignition_enable;
		}
		t24.prev_ign_pin_state = t24.ignition_pin_state;
		AS_data.ign = t24.Ignition_Request;
		AS_data.mission_select = t24.Current_Mission;
		AS_data.acu_state = (uint8_t)Vehicle_state_machine;
		AS_data.emergency_cause = Emergency_cause;

		autonomous_t26_acu_pack(tx_data, &AS_data, AUTONOMOUS_T26_ACU_LENGTH);

		can_tx_header.StdId = AUTONOMOUS_T26_ACU_FRAME_ID;
		can_tx_header.RTR = CAN_RTR_DATA;
		can_tx_header.DLC = AUTONOMOUS_T26_ACU_LENGTH;

		add_can_message(TX_MAILBOX, can_tx_header, tx_data);

		struct autonomous_t26_dv_status_t dv_data;

		dv_data.as_status = t24.Autonomous_State;
		dv_data.ami_state = t24.Jetson_mission;
		//dv_data.asb_ebs_state

		autonomous_t26_dv_status_pack(tx_data, &dv_data, AUTONOMOUS_T26_DV_STATUS_LENGTH);

		can_tx_header.StdId = AUTONOMOUS_T26_DV_STATUS_FRAME_ID;
		can_tx_header.RTR = CAN_RTR_DATA;
		can_tx_header.DLC = AUTONOMOUS_T26_DV_STATUS_LENGTH;
		add_can_message(TX_MAILBOX, can_tx_header, tx_data);

		struct autonomous_t26_asf_signals_t asf_signals;
		asf_signals.brake_pressure_front = t24.Front_Pressure.Hydraulic * 10;
		asf_signals.brake_pressure_rear = t24.Rear_Pressure.Hydraulic * 10;
		asf_signals.ebs_pressure_tank_front = t24.Front_Pressure.Pneumatic * 10;
		asf_signals.ebs_pressure_tank_rear = t24.Rear_Pressure.Pneumatic * 10;
		autonomous_t26_asf_signals_pack(tx_data, &asf_signals, AUTONOMOUS_T26_ASF_SIGNALS_LENGTH);
		can_tx_header.StdId = AUTONOMOUS_T26_ASF_SIGNALS_FRAME_ID;
		can_tx_header.RTR = CAN_RTR_DATA;
		can_tx_header.DLC = AUTONOMOUS_T26_ASF_SIGNALS_LENGTH;

		add_can_message(TX_MAILBOX, can_tx_header, tx_data);

		if (activate_res){
			can_tx_header.StdId = 0x00;
			can_tx_header.RTR = CAN_RTR_DATA;
			can_tx_header.DLC = 2;
			tx_data[0] = 1;
			tx_data[1] = 0;
			add_can_message(TX_MAILBOX, can_tx_header, tx_data);
			activate_res = 0;
		}

		/* ── BLE telemetry: 15-byte binary packet via ble_handler ── */
		if (!ble_module_config_is_done()) {
		    return;
		}
		if (ble_tx_busy) {
		    return;                 /* still sending previous packet, skip this tick */
		}
		static ble_telemetry_packet_t pkt;
		pkt.state_machine      = (uint8_t)Vehicle_state_machine;
		pkt.assi_status        = t24.ASSI_state;
		pkt.mission            = (uint8_t)t24.Current_Mission;
		{   float _v = t24.Front_Pressure.Hydraulic * 100.0f;
		    pkt.hydraulic_p1 = (_v > 65535.0f) ? 65535 : (uint16_t)_v; }
		{   float _v = t24.Rear_Pressure.Hydraulic * 100.0f;
		    pkt.hydraulic_p2 = (_v > 65535.0f) ? 65535 : (uint16_t)_v; }
		{   float _v = t24.Front_Pressure.Pneumatic * 100.0f;
		    pkt.pneumatic_p1 = (_v > 65535.0f) ? 65535 : (uint16_t)_v; }
		{   float _v = t24.Rear_Pressure.Pneumatic * 100.0f;
		    pkt.pneumatic_p2 = (_v > 65535.0f) ? 65535 : (uint16_t)_v; }
		{   float _v = t24.chip_temp * 100.0f;
		    pkt.chip_temp = (_v > 32767.0f) ? 32767 : (_v < -32768.0f) ? -32768 : (int16_t)_v; }
		pkt.solenoid_front   = t24.front_solenoid;
		pkt.solenoid_rear    = t24.rear_solenoid;
		ble_send_binary((uint8_t*)&pkt, sizeof(pkt));
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
		can_rx_buffer_push(&can_rx_ringbuffer, rx_header, rx_data);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MS_BTN_Pin && mission_selector_enable == 1) {
		t24.Current_Mission++;
		if (t24.Current_Mission > AUTOCROSS) {
			t24.Current_Mission = MANUAL;
		}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		char cmd[RX_BUFFER_SIZE] = { 0 };

		uint8_t j = 0;

		for (uint8_t i = 0; i < Size && j < RX_BUFFER_SIZE - 1; i++) {
			char c = (char) rx_buffer[i];
			if (c == '\r' || c == '\n' || c == ' ')
				continue;
			cmd[j++] = tolower(c);
		}

		if (strcmp(cmd, "log") == 0)
			uart_log_dump = 1;
		if (strcmp(cmd, "resume") == 0)
			uart_log_dump = 0;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
