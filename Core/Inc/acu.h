/*
 * acu.h
 *
 * The whole state of the ACU in one global struct: add `acu` to the Live Expressions
 * of STM32CubeIDE and every input, state machine, fault and output is visible.
 *
 * Who writes what:
 *   - the control task (1 ms) writes almost everything,
 *   - interrupts write acu.inputs.front_tank_bar / rear_tank_bar / chip_temp_c (ADC)
 *     and acu.inputs.mission_button_presses (EXTI),
 *   - the telemetry and logger tasks only read, except acu.ble and acu.log.
 */
#ifndef ACU_H
#define ACU_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "can.h"
#include "acu_config.h"

/* ── Values shared with other nodes (CAN encoding, see autonomous_t26.dbc) ── */

/* AS status, T 14.8.3. ACU.as_state and DV_status.AS_status. */
typedef enum {
	AS_STATE_OFF = 1,
	AS_STATE_READY = 2,
	AS_STATE_DRIVING = 3,
	AS_STATE_EMERGENCY = 4,
	AS_STATE_FINISHED = 5
} AS_STATE_t;

/* ACU.mission_select, JETSON.as_mission and DV_status.AMI_state. */
typedef enum {
	MISSION_MANUAL = 0,
	MISSION_ACCELERATION = 1,
	MISSION_SKIDPAD = 2,
	MISSION_TRACKDRIVE = 3,
	MISSION_EBS_TEST = 4,
	MISSION_INSPECTION = 5,
	MISSION_AUTOCROSS = 6
} mission_t;
#define MISSION_LAST MISSION_AUTOCROSS

/* ACU.emergency_cause: the coarse cause the Jetson already understands.
 * The detailed reason is acu.fault.reason. */
typedef enum {
	CAN_CAUSE_NONE = 0,
	CAN_CAUSE_SDC_OPEN = 1,
	CAN_CAUSE_RES = 2,
	CAN_CAUSE_PRESSURE_CHECKS = 3,
	CAN_CAUSE_VCU_TIMEOUT = 4,
	CAN_CAUSE_JETSON_TIMEOUT = 5,
	CAN_CAUSE_ACU_WATCHDOG = 6,
	CAN_CAUSE_STEERING_TIMEOUT = 7,
	CAN_CAUSE_PRESSURE_TIMEOUT = 8,
	CAN_CAUSE_UNKNOWN = 9
} can_emergency_cause_t;

/* DV_status.ASB_EBS_state */
typedef enum {
	EBS_STATE_DEACTIVATED = 1,
	EBS_STATE_CHECKUP_PASSED = 2,
	EBS_STATE_ACTIVATED = 3
} ebs_state_t;

/* ── ACU state machines ─────────────────────────────────────────────────── */

/* Main state machine. Values are ACU.acu_state on CAN (unchanged, the Jetson reads them). */
typedef enum {
	ACU_MODE_STARTUP = 0,
	ACU_MODE_IDLE = 1,        /* AS Off: mission selection, manual driving */
	ACU_MODE_AUTONOMOUS = 2,  /* ASMS on with an autonomous mission: checkup, Ready, Driving, Finished */
	ACU_MODE_EMERGENCY = 3    /* a fault was found: EBS triggered, SDC open, waits for ASMS off + brakes released */
} acu_mode_t;

/* Initial checkup, T 15.3.1. Runs once per ASMS switch-on, in this order. */
typedef enum {
	CHECKUP_NOT_STARTED,
	CHECKUP_TANK_PRESSURE,       /* EBS tanks in range */
	CHECKUP_BRAKES_ENGAGED,      /* T 14.4.1: brake pressure built up before the SDC may close */
	CHECKUP_SDC_CLOSE,           /* watchdog pulses on: SDC must close */
	CHECKUP_WATCHDOG_OPENS_SDC,  /* pulses off: the external watchdog must open the SDC */
	CHECKUP_SDC_RECLOSE,         /* pulses on again: SDC must close again */
	CHECKUP_TS_ACTIVATION,       /* TS request allowed, waiting for the VCU to report TS active */
	CHECKUP_FRONT_BRAKE,         /* only the front EBS circuit engaged */
	CHECKUP_REAR_BRAKE,          /* only the rear EBS circuit engaged */
	CHECKUP_BOTH_BRAKES,         /* both engaged again */
	CHECKUP_PASSED,
	CHECKUP_FAILED
} checkup_step_t;

/* End of mission, T 14.8.3: AS Finished needs mission finished + standstill + EBS activated. */
typedef enum {
	FINISH_NOT_FINISHED,
	FINISH_WAITING_FOR_STANDSTILL,  /* Jetson reported the mission finished */
	FINISH_EBS_ACTIVATED            /* at standstill: the ACU opened the SDC and engaged the EBS */
} finish_step_t;

/* Why the ACU went to EMERGENCY. Logged to Bluetooth and EEPROM. */
typedef enum {
	FAULT_NONE = 0,
	/* initial checkup, T 15.3.1 */
	FAULT_CHECKUP_TANK_PRESSURE,
	FAULT_CHECKUP_BRAKES_NOT_ENGAGED,
	FAULT_CHECKUP_SDC_DID_NOT_CLOSE,
	FAULT_CHECKUP_WATCHDOG_DID_NOT_OPEN_SDC,
	FAULT_CHECKUP_SDC_DID_NOT_RECLOSE,
	FAULT_CHECKUP_SDC_OPENED,
	FAULT_CHECKUP_FRONT_BRAKE,
	FAULT_CHECKUP_REAR_BRAKE,
	FAULT_CHECKUP_BOTH_BRAKES,
	/* continuous monitoring, T 15.3.2 */
	FAULT_SDC_OPENED,
	FAULT_ASMS_SWITCHED_OFF,
	FAULT_VCU_TIMEOUT,
	FAULT_JETSON_TIMEOUT,
	FAULT_RES_TIMEOUT,
	FAULT_TANK_PRESSURE,
	FAULT_BRAKE_PRESSURE_LOST,
	FAULT_TS_DEACTIVATED,
	FAULT_MISSION_MISMATCH,
	FAULT_JETSON_EMERGENCY,
	/* AS state rules, T 14.8 */
	FAULT_R2D_BEFORE_5S,
	FAULT_MOVED_BEFORE_3S,
	FAULT_FINISH_NOT_AT_STANDSTILL,
	FAULT_TS_STILL_ON_AFTER_FINISH,
	/* software */
	FAULT_INVALID_STATE,
	FAULT_STACK_OVERFLOW,
	FAULT_OUT_OF_MEMORY,
	FAULT_HARD_FAULT,
	FAULT_COUNT
} fault_reason_t;

/* ── Pieces of the global struct ────────────────────────────────────────── */

typedef struct {
	volatile bool asms_on;
	volatile bool sdc_closed;               /* SDC feedback after the ACU watchdog relay */
	volatile bool ignition_button;          /* IGN button pressed */
	volatile float front_tank_bar;          /* EBS pneumatic tanks (ADC, filtered) */
	volatile float rear_tank_bar;
	volatile float chip_temp_c;
	volatile uint32_t mission_button_presses; /* counted in the EXTI interrupt, debounced */
	volatile uint32_t mission_button_last_ms;
} acu_inputs_t;

typedef struct {
	float front_bar;           /* brake line pressure, AQT1 */
	float rear_bar;            /* brake line pressure, AQT7 */
	uint8_t aqt1_res_bit;      /* AQT1.RES and AQT1.BOTS as received, not used by the logic yet */
	uint8_t aqt1_bots_bit;
} brakes_t;

typedef struct {
	bool ts_active;            /* VCU_IGN_R2D.ignition_auto */
	uint8_t shutdown_signal;   /* VCU_IGN_R2D.shutdown_signal */
	uint8_t vcu_state;
	uint16_t motor_rpm_left;   /* VCU_RPM */
	uint16_t motor_rpm_right;
} vcu_t;

typedef struct {
	uint8_t as_state;          /* JETSON.as_state as received (AS_STATE_t value) */
	uint8_t mission;           /* JETSON.as_mission */
	uint8_t emergency_cause;
	uint8_t temperature;
	uint8_t cpu;
	uint8_t gpu;
	uint8_t lap_counter;       /* SLAM_STATS_CAN, forwarded in DV_status */
	uint8_t cones_count_actual;
	uint32_t cones_count_all;
} jetson_t;

typedef struct {
	uint8_t payload[8];        /* last RES frame, raw: watch it here to find the stop/go bits */
	uint8_t length;
} res_t;

typedef struct {
	uint32_t last_rx_ms;
	uint32_t age_ms;           /* refreshed every control cycle */
	uint32_t rx_count;
} can_node_t;

typedef struct {
	can_node_t vcu;            /* VCU_IGN_R2D */
	can_node_t vcu_rpm;        /* VCU_RPM */
	can_node_t jetson;
	can_node_t res;
	can_node_t steering;       /* CubeMars feedback */
	can_node_t brake_front;    /* AQT1 */
	can_node_t brake_rear;     /* AQT7 */
} can_nodes_t;

typedef struct {
	mission_t selected;
	bool selection_allowed;    /* only with the ASMS off */
	uint32_t presses_handled;
	bool mismatch;             /* Jetson mission != selected mission */
	uint32_t mismatch_since_ms;
} mission_status_t;

typedef struct {
	checkup_step_t step;
	uint32_t step_since_ms;
	checkup_step_t failed_step;  /* last step that failed */
	uint32_t passed_count;
	uint32_t failed_count;
} checkup_t;

/* Inputs of the AS status flowchart, Figure 16 of the rules. */
typedef struct {
	bool ebs_activated;          /* T 14.8.1 */
	bool mission_finished;
	bool vehicle_at_standstill;
	bool sdc_opened_by_res;
	bool mission_selected;       /* an autonomous mission */
	bool asms_on;
	bool asb_checks_ok;          /* initial checkup passed */
	bool ts_active;
	bool r2d;
	bool brakes_engaged;
} as_flowchart_inputs_t;

typedef struct {
	AS_STATE_t state;            /* result of the flowchart, what the ASSIs and CAN show */
	uint32_t state_since_ms;
	as_flowchart_inputs_t flowchart;
	finish_step_t finish;
	uint32_t finish_since_ms;
	bool speed_known;            /* VCU_RPM fresh */
	bool moving;                 /* debounced, STANDSTILL_* in acu_config.h */
	uint32_t moving_since_ms;     /* when the speed first went above STANDSTILL_MAX_RPM */
	uint32_t speed_change_since_ms;
	uint32_t brakes_released_since_ms;
} as_status_t;

typedef struct {
	bool active;                 /* monitoring runs in AS Ready and AS Driving */
	uint32_t ts_off_since_ms;
	fault_reason_t last_detected;
} monitoring_t;

typedef struct {
	bool active;                 /* ACU in EMERGENCY because of `reason` */
	fault_reason_t reason;       /* first fault, the one that caused the EMERGENCY */
	uint32_t since_ms;
	fault_reason_t last_reason;  /* reason of the last EMERGENCY, kept after leaving it */
	uint32_t count;              /* emergencies since boot */
	/* snapshot taken when the fault was raised */
	acu_mode_t mode;
	AS_STATE_t as_state;
	checkup_step_t checkup_step;
	float front_tank_bar;
	float rear_tank_bar;
	float front_brake_bar;
	float rear_brake_bar;
	bool sdc_closed;
	bool asms_on;
	bool ts_active;
} fault_t;

typedef struct {
	bool front_ebs_released;     /* front solenoid energised */
	bool rear_ebs_released;      /* rear solenoid energised */
	bool watchdog_pulses_on;     /* the external watchdog keeps the SDC closed while it gets pulses */
	bool ignition_enable;        /* the ACU allows a TS request */
	bool ignition_request;       /* ACU.ign sent to the VCU, toggled by the IGN button */
	bool assi_yellow;
	bool assi_blue;
	bool res_start_pending;      /* CANopen NMT start for the RES still to be sent */
} outputs_t;

typedef struct {
	bool eeprom_ok;
	uint8_t stored;              /* records in the EEPROM */
	uint32_t written;
	uint32_t dropped;            /* queue full */
	uint32_t write_errors;
} log_status_t;

typedef struct {
	uint8_t module_config_step;  /* RN4871 configuration, 5 = done */
	bool paused;                 /* "stop" received */
	uint32_t lines_sent;
} ble_status_t;

typedef struct {
	uint32_t uptime_ms;
	uint32_t control_cycles;
	uint32_t control_late;         /* control cycles that started more than 2 ms late */
	uint32_t reset_flags;          /* RCC->CSR at boot */
	const char *reset_cause;
	uint32_t stack_free_control;   /* words never used, per task */
	uint32_t stack_free_telemetry;
	uint32_t stack_free_logger;
	bool stack_overflow;
	char stack_overflow_task[16];
	bool out_of_memory;
	bool bootloader_requested;
} system_status_t;

typedef struct {
	acu_mode_t mode;
	uint32_t mode_since_ms;
	acu_inputs_t inputs;
	brakes_t brakes;
	vcu_t vcu;
	jetson_t jetson;
	res_t res;
	can_nodes_t nodes;
	mission_status_t mission;
	checkup_t checkup;
	as_status_t as;
	monitoring_t monitor;
	fault_t fault;
	outputs_t outputs;
	log_status_t log;
	ble_status_t ble;
	system_status_t system;
	const CAN_BusStatus *can_bus;  /* expand it in Live Expressions */
} acu_t;

extern acu_t acu;

void acu_init(uint32_t now_ms);

/* true once `duration_ms` has passed since `since_ms` (wrap-around safe) */
static inline bool time_elapsed(uint32_t since_ms, uint32_t now_ms, uint32_t duration_ms) {
	return (uint32_t)(now_ms - since_ms) >= duration_ms;
}

const char *acu_mode_name(acu_mode_t mode);
const char *as_state_name(AS_STATE_t state);
const char *checkup_step_name(checkup_step_t step);
const char *finish_step_name(finish_step_t step);
const char *mission_name(mission_t mission);

#endif /* ACU_H */
