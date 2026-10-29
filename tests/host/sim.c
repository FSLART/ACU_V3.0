/*
 * PC test harness for the ACU logic.
 *
 * build.py compiles the real APP.c, state_machine.c, Autonomous_functions.c,
 * hardware_abstaction.c, ring_buffer.c, ble_handler.c and the T26 DBC code
 * against stub/stm32f4xx_hal.h. This file stands in for the hardware: GPIO pins,
 * the millisecond tick, the CAN RX interrupt, the TIM2 (10 Hz) and ADC interrupts
 * from main.c, and USART1 to the RN4871. acu_sim.py drives it through sim_*.
 */
#include "main.h"
#include "APP.h"
#include "state_machine.h"
#include "can.h"
#include "ble_handler.h"
#include "autonomous_t26.h"

#define SIM_API __declspec(dllexport)

/* Must match the RES case in dbc_decode() (APP.c). */
#define RES_RX_ID 0x18Bu

GPIO_TypeDef sim_gpio_ports[8] = { {0}, {1}, {2}, {3}, {4}, {5}, {6}, {7} };
IWDG_TypeDef sim_iwdg;

/* Defined in main.c / can.c / ble_handler.c on the target. */
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim2, htim8;
UART_HandleTypeDef huart1, huart2;
uint32_t ADC_Samples[4];
uint8_t mission_selector_enable;
struct ring can_tx_ringbuffer;
struct ring can_rx_ringbuffer;

extern uint8_t activate_res;
extern cant_acu_state_t ACU_STATE;

static uint32_t now_ms;
static float pneumatic_front, pneumatic_rear;
static uint32_t res_nmt_starts;

typedef struct {
	const char *name;
	GPIO_TypeDef *port;
	uint16_t pin;
	int level;
	uint32_t changes;
} sim_pin_t;

#define PIN(n) { #n, n##_GPIO_Port, n##_Pin, 0, 0 }
static sim_pin_t pins[] = {
	PIN(ASMS), PIN(SDC_FEEDBACK), PIN(IGN_BTN), PIN(MS_BTN), PIN(WDT_PULSE),
	PIN(Front_Solenoid), PIN(Rear_Solenoid), PIN(ASSI_YELLOW), PIN(ASSI_BLUE),
	PIN(LED1), PIN(LED2), PIN(HB), PIN(CAN_INDICATO),
};
#define PIN_COUNT (sizeof(pins) / sizeof(pins[0]))

static sim_pin_t *pin_at(GPIO_TypeDef *port, uint16_t pin)
{
	for (size_t i = 0; i < PIN_COUNT; i++)
		if (pins[i].port == port && pins[i].pin == pin)
			return &pins[i];
	return NULL;
}

static sim_pin_t *pin_named(const char *name)
{
	for (size_t i = 0; i < PIN_COUNT; i++)
		if (strcmp(pins[i].name, name) == 0)
			return &pins[i];
	return NULL;
}

/* ---- HAL ---------------------------------------------------------------- */

uint32_t HAL_GetTick(void) { return now_ms; }

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
	sim_pin_t *p = pin_at(port, pin);
	return (p && p->level) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
	sim_pin_t *p = pin_at(port, pin);
	int level = state != GPIO_PIN_RESET;
	if (!p)
		return;
	if (level != p->level)
		p->changes++;
	p->level = level;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
	sim_pin_t *p = pin_at(port, pin);
	if (!p)
		return;
	p->level = !p->level;
	p->changes++;
}

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *d, uint32_t n) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) { return HAL_OK; }

/* USART1 = RN4871. ble_out is what the phone receives; DMA transfers take
 * the real time at 115200 baud before the TX complete interrupt fires. */
static char ble_out[65536];
static size_t ble_out_len;
static int ble_tx_pending;
static uint32_t ble_tx_done_ms;
static DMA_HandleTypeDef ble_rx_dma;
static uint8_t *ble_rx_buf;
static uint16_t ble_rx_size;
static char ble_commands[1024];
static size_t ble_commands_len;

/* Bytes from the RN4871 into the circular RX DMA buffer. */
static void ble_rx_write(const char *text)
{
	for (; *text && ble_rx_buf; text++) {
		ble_rx_buf[ble_rx_size - ble_rx_dma.remaining] = (uint8_t)*text;
		ble_rx_dma.remaining = ble_rx_dma.remaining > 1 ? ble_rx_dma.remaining - 1 : ble_rx_size;
	}
}

/* Blocking sends on USART1 are the RN4871 configuration commands (APP.c).
 * The module answers like the real one: CMD> after $$$, AOK per command. */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n, uint32_t t)
{
	if (h != &huart1)
		return HAL_OK;
	if (ble_commands_len + n < sizeof(ble_commands)) {
		memcpy(ble_commands + ble_commands_len, d, n);
		ble_commands_len += n;
	}
	if (n == 3 && memcmp(d, "$$$", 3) == 0)
		ble_rx_write("CMD> ");
	else if (n >= 3 && memcmp(d, "R,1", 3) == 0)
		ble_rx_write("Rebooting\r\n");
	else
		ble_rx_write("AOK\r\nCMD> ");
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n)
{
	size_t room = sizeof(ble_out) - ble_out_len;

	if (h != &huart1)
		return HAL_OK;
	if (ble_tx_pending)
		return HAL_BUSY;
	memcpy(ble_out + ble_out_len, d, n < room ? n : room);
	ble_out_len += n < room ? n : room;
	ble_tx_pending = 1;
	ble_tx_done_ms = now_ms + (n * 10u * 1000u + 115199u) / 115200u;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n)
{
	if (h == &huart1) {
		ble_rx_buf = d;
		ble_rx_size = n;
		ble_rx_dma.remaining = n;
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h)
{
	if (h == &huart1)
		ble_tx_pending = 0;
	return HAL_OK;
}

/* ---- Modules not under test --------------------------------------------- */

void CAN_Service(CAN_HandleTypeDef *hcan) {}
HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *header, const uint8_t data[8]) { return HAL_OK; }

/* ---- Simulation API ------------------------------------------------------ */

typedef struct {
	uint32_t time_ms;
	int32_t vehicle_state, autonomous_state, acu_state, startup_seq, emergency_cause;
	int32_t as_state, current_mission, jetson_mission;
	int32_t front_solenoid, rear_solenoid;
	int32_t ignition_request, ignition_enable, ignition_status, ignition_pin;
	int32_t hw_wdt_enable, asms, sdc_feedback, vcu_sdc, assi_state, emergency, rpm;
	float front_pneumatic, front_hydraulic, rear_pneumatic, rear_hydraulic;
	uint32_t vcu_last, jetson_last, res_last, rear_pressure_last, dir_last;
	int32_t seq_fail_step;
	uint32_t seq_fail_count, seq_fail_ms;
	uint32_t res_nmt_starts, can_rx_pending;
} sim_state_t;

SIM_API uint32_t sim_state_size(void) { return sizeof(sim_state_t); }

/* Power-on: what main() does after the MX_*_Init() calls. */
SIM_API void sim_init(void)
{
	now_ms = 0;
	huart1.hdmarx = &ble_rx_dma;
	app_init();
}

/* Text the ACU sent to the phone since the last call. */
SIM_API uint32_t sim_ble_read(char *dst, uint32_t max)
{
	uint32_t n = ble_out_len < max ? (uint32_t)ble_out_len : max;

	memcpy(dst, ble_out, n);
	memmove(ble_out, ble_out + n, ble_out_len - n);
	ble_out_len -= n;
	return n;
}

/* Every configuration command sent to the RN4871 since power-on. */
SIM_API uint32_t sim_ble_commands(char *dst, uint32_t max)
{
	uint32_t n = ble_commands_len < max ? (uint32_t)ble_commands_len : max;

	memcpy(dst, ble_commands, n);
	return n;
}

/* Text typed on the phone, written into the circular RX DMA buffer. */
SIM_API void sim_ble_receive(const char *text)
{
	ble_rx_write(text);
}

SIM_API void sim_set_pin(const char *name, int level)
{
	sim_pin_t *p = pin_named(name);
	if (p)
		p->level = level != 0;
}

SIM_API int sim_get_pin(const char *name)
{
	sim_pin_t *p = pin_named(name);
	return p ? p->level : -1;
}

/* Number of level changes, e.g. the external watchdog pulses on WDT_PULSE. */
SIM_API uint32_t sim_pin_changes(const char *name)
{
	sim_pin_t *p = pin_named(name);
	return p ? p->changes : 0;
}

/* Tank pressures as the ADC callback would write them (no EMA filter). */
SIM_API void sim_set_pneumatic(float front_bar, float rear_bar)
{
	pneumatic_front = front_bar;
	pneumatic_rear = rear_bar;
}

/* The mission-select button EXTI in main.c. */
SIM_API void sim_set_mission(int mission) { t24.Current_Mission = (current_mission_t)mission; }

/* What HAL_CAN_RxFifo0MsgPendingCallback() does with a received frame. */
SIM_API void sim_can_rx(uint32_t id, int extended, const uint8_t *data, uint32_t dlc)
{
	CAN_RxHeaderTypeDef header = { 0 };
	uint8_t payload[8] = { 0 };

	if (extended) {
		header.ExtId = id;
		header.IDE = CAN_ID_EXT;
	} else {
		header.StdId = id;
		header.IDE = CAN_ID_STD;
	}
	header.DLC = dlc > 8 ? 8 : dlc;
	memcpy(payload, data, header.DLC);
	can_rx_buffer_push(&can_rx_ringbuffer, header, payload);
}

SIM_API void sim_can_rx_jetson(int as_state, int mission)
{
	struct autonomous_t26_jetson_t frame = { 0 };
	uint8_t data[8] = { 0 };

	frame.as_state = autonomous_t26_jetson_as_state_encode(as_state);
	frame.as_mission = autonomous_t26_jetson_as_mission_encode(mission);
	autonomous_t26_jetson_pack(data, &frame, AUTONOMOUS_T26_JETSON_LENGTH);
	sim_can_rx(AUTONOMOUS_T26_JETSON_FRAME_ID, 0, data, AUTONOMOUS_T26_JETSON_LENGTH);
}

SIM_API void sim_can_rx_vcu(int ignition_auto, int shutdown_signal)
{
	struct autonomous_t26_vcu_ign_r2_d_t frame = { 0 };
	uint8_t data[8] = { 0 };

	frame.ignition_auto = autonomous_t26_vcu_ign_r2_d_ignition_auto_encode(ignition_auto);
	frame.shutdown_signal = autonomous_t26_vcu_ign_r2_d_shutdown_signal_encode(shutdown_signal);
	autonomous_t26_vcu_ign_r2_d_pack(data, &frame, AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH);
	sim_can_rx(AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID, 0, data, AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH);
}

/* AQT1 (front) and AQT7 (rear) brake pressure: 0.1 bar per bit, little endian. */
SIM_API void sim_can_rx_brake_pressures(float front_bar, float rear_bar)
{
	uint16_t front = (uint16_t)(front_bar * 10.0f + 0.5f);
	uint16_t rear = (uint16_t)(rear_bar * 10.0f + 0.5f);
	uint8_t aqt1[3] = { (uint8_t)front, (uint8_t)(front >> 8), 0 };
	uint8_t aqt7[2] = { (uint8_t)rear, (uint8_t)(rear >> 8) };

	sim_can_rx(AUTONOMOUS_T26_AQT1_FRAME_ID, 0, aqt1, AUTONOMOUS_T26_AQT1_LENGTH);
	sim_can_rx(AUTONOMOUS_T26_AQT7_FRAME_ID, 0, aqt7, AUTONOMOUS_T26_AQT7_LENGTH);
}

SIM_API void sim_can_rx_res(void)
{
	uint8_t data[8] = { 0 };
	sim_can_rx(RES_RX_ID, 0, data, 8);
}

SIM_API void sim_can_rx_steering(void)
{
	uint8_t data[8] = { 0 };
	sim_can_rx(AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID, 1, data, 8);
}

/* Advance time. Per millisecond: the ADC interrupt, the TIM2 interrupt every 100 ms,
 * and two passes of the main loop (the target runs ~2.6 per ms). */
SIM_API void sim_step_ms(uint32_t ms)
{
	for (uint32_t i = 0; i < ms; i++) {
		now_ms++;
		if (ble_tx_pending && now_ms >= ble_tx_done_ms) {
			ble_tx_pending = 0;
			HAL_UART_TxCpltCallback(&huart1);
		}
		t24.Front_Pressure.Pneumatic = pneumatic_front;
		t24.Rear_Pressure.Pneumatic = pneumatic_rear;

		if (now_ms % 100 == 0) {
			/* TIM2: RES NMT start (CAN_Send always succeeds here), then the ignition button */
			if (activate_res) {
				res_nmt_starts++;
				activate_res = 0;
			}
			update_ignition_request();
		}

		app();
		app();
	}
}

SIM_API void sim_get_state(sim_state_t *s)
{
	s->time_ms = now_ms;
	s->vehicle_state = Vehicle_state_machine;
	s->autonomous_state = Autonomous_state;
	s->acu_state = ACU_STATE;
	s->startup_seq = startup_sequence_state;
	s->emergency_cause = Emergency_cause;
	s->as_state = t24.Autonomous_State;
	s->current_mission = t24.Current_Mission;
	s->jetson_mission = t24.Jetson_mission;
	s->front_solenoid = t24.front_solenoid;
	s->rear_solenoid = t24.rear_solenoid;
	s->ignition_request = t24.Ignition_Request;
	s->ignition_enable = t24.Ignition_enable;
	s->ignition_status = t24.Ignition_Status;
	s->ignition_pin = t24.ignition_pin_state;
	s->hw_wdt_enable = t24.HW_WDT_Enable;
	s->asms = t24.ASMS;
	s->sdc_feedback = t24.SDC_feedback;
	s->vcu_sdc = t24.vcu_sdc;
	s->assi_state = t24.ASSI_state;
	s->emergency = t24.Emergency;
	s->rpm = t24.rpm;
	s->front_pneumatic = t24.Front_Pressure.Pneumatic;
	s->front_hydraulic = t24.Front_Pressure.Hydraulic;
	s->rear_pneumatic = t24.Rear_Pressure.Pneumatic;
	s->rear_hydraulic = t24.Rear_Pressure.Hydraulic;
	s->vcu_last = t24.VCU_LAST_TX;
	s->jetson_last = t24.JETSON_LAST_TX;
	s->res_last = t24.RES_LAST_TX;
	s->rear_pressure_last = t24.REAR_PRESSURE_LAST_TX;
	s->dir_last = t24.DIR_ACTUATOR_LAST_TX;
	s->seq_fail_step = seq_fail.step;
	s->seq_fail_count = seq_fail.count;
	s->seq_fail_ms = seq_fail.ms;
	s->res_nmt_starts = res_nmt_starts;
	s->can_rx_pending = can_rx_ringbuffer.counter;
}
