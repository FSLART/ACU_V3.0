/*
 * Host stand-in for the STM32 HAL, used only by the PC test harness (tests/host).
 * It declares just what the ACU logic files touch; sim.c implements it.
 */
#ifndef STM32F4XX_HAL_HOST_STUB_H
#define STM32F4XX_HAL_HOST_STUB_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

typedef struct { int index; } GPIO_TypeDef;
extern GPIO_TypeDef sim_gpio_ports[8];
#define GPIOA (&sim_gpio_ports[0])
#define GPIOB (&sim_gpio_ports[1])
#define GPIOC (&sim_gpio_ports[2])
#define GPIOD (&sim_gpio_ports[3])
#define GPIOE (&sim_gpio_ports[4])
#define GPIOH (&sim_gpio_ports[7])

#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_3  ((uint16_t)0x0008)
#define GPIO_PIN_4  ((uint16_t)0x0010)
#define GPIO_PIN_5  ((uint16_t)0x0020)
#define GPIO_PIN_6  ((uint16_t)0x0040)
#define GPIO_PIN_7  ((uint16_t)0x0080)
#define GPIO_PIN_8  ((uint16_t)0x0100)
#define GPIO_PIN_9  ((uint16_t)0x0200)
#define GPIO_PIN_10 ((uint16_t)0x0400)
#define GPIO_PIN_11 ((uint16_t)0x0800)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)

typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } RTC_HandleTypeDef;
typedef struct { int unused; } CAN_HandleTypeDef;

typedef struct {
	uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct {
	uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp, FilterMatchIndex;
} CAN_RxHeaderTypeDef;

#define CAN_ID_STD   0x00000000U
#define CAN_ID_EXT   0x00000004U
#define CAN_RTR_DATA 0x00000000U

typedef struct { volatile uint32_t KR, PR, RLR, SR; } IWDG_TypeDef;
extern IWDG_TypeDef sim_iwdg;
#define IWDG (&sim_iwdg)

static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t primask) { (void)primask; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}

uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *data, uint32_t length);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);

#endif
