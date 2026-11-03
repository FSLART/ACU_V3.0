/*
 * acu_config.h
 *
 * Every threshold, timeout and test flag of the ACU in one place.
 * Rule references are to FS Rules 2027 v1.0 (FS_Rules_2027_v1.0.pdf).
 */
#ifndef ACU_CONFIG_H
#define ACU_CONFIG_H

/* ── Test flags ─────────────────────────────────────────────────────────────
 * 1 = skip / bypass. All must be 0 on a rules-compliant car, except where noted.
 * Every flag set to 1 is listed in the Bluetooth status block.               */

/* Initial checkup steps (T 15.3.1) */
#define SKIP_WDT_CHECK            0   /* watchdog test: SDC must open when the pulses stop */
#define SKIP_PNEUMATIC_CHECK      0   /* EBS tank pressures in range */
#define SKIP_PRESSURE_CHECK1      0   /* EBS builds brake pressure on both axles */
#define SKIP_IGNITION_CHECK       0   /* wait for the TS to be active */
#define SKIP_PRESSURE_FRONT_CHECK 0   /* front EBS circuit alone */
#define SKIP_PRESSURE_REAR_CHECK  0   /* rear EBS circuit alone */
#define SKIP_PRESSURE_CHECK2      0   /* both circuits again before AS Ready */

/* Bench: ignore the hydraulic CAN reading and use a value that follows the solenoid
 * (front line bleeds down within ~2 s on the bench, so the real reading can't hold). */
#define BYPASS_FRONT_HYD_PRESSURE 0
#define BYPASS_REAR_HYD_PRESSURE  0
#define BYPASS_HYD_LOADED_BAR     150.0f
#define BYPASS_HYD_UNLOADED_BAR   0.0f

/* Bench: a silent Jetson does not trigger an emergency. Set to 0 before running on track. */
#define BYPASS_JETSON_TIMEOUT     1

/* ── CAN node timeouts (T 15.3.2 continuous monitoring) ──────────────────────
 * NOTE T 11.9: a system critical signal may be delayed by at most 500 ms.
 * The values below are the ones validated on the car; the rules-compliant value is 500 ms. */
#define VCU_TIMEOUT_MS            1000   /* rules-compliant: 500 */
#define RES_TIMEOUT_MS            1000   /* rules-compliant: 500 */
#define JETSON_TIMEOUT_MS         2000   /* rules-compliant: 500. Linux side stalls longer than MCU nodes */
#define VCU_RPM_VALID_MS          500    /* VCU_RPM older than this: vehicle speed unknown */

/* ── EBS pressures ──────────────────────────────────────────────────────────── */
#define EBS_FRONT_TANK_MIN_BAR    5.0f
#define EBS_REAR_TANK_MIN_BAR     2.5f
#define EBS_TANK_MAX_BAR          10.0f

/* Brakes count as engaged when hydraulic >= gain x tank pressure.
 * Calibrated with the EBS applied after the brake balance was moved for regen:
 * front 72.3 bar hyd / 7.66 bar pneu = 9.4x, rear 24.2 / 3.36 = 7.2x.
 * Gains ~80 % of the measured ratio; re-measure if the brake balance changes again. */
#define EBS_FRONT_HYD_GAIN        7.5f
#define EBS_REAR_HYD_GAIN         5.8f
#define BRAKE_RELEASED_MAX_BAR    5.5f   /* hydraulic at or below this: that axle is released */

/* ── Initial checkup timing (T 15.3.1) ──────────────────────────────────────── */
#define CHECKUP_BRAKES_ENGAGED_TIMEOUT_MS 5000  /* brake pressure must build before the SDC closes */
#define CHECKUP_SDC_CLOSE_TIMEOUT_MS      5000  /* SDC must close once the watchdog pulses run */
#define CHECKUP_SDC_OPEN_TIMEOUT_MS       1000  /* watchdog must open the SDC after the pulses stop */
#define CHECKUP_BRAKE_SETTLE_MS           3000  /* a single EBS circuit is judged after this ... */
#define CHECKUP_BRAKE_TIMEOUT_MS          5000  /* ... and must pass before this */

/* ── AS state rules (T 14.8) ────────────────────────────────────────────────── */
#define READY_MIN_TIME_BEFORE_R2D_MS 5000  /* T 14.8.4: R2D only after >= 5 s in AS Ready */
#define DRIVING_NO_MOVE_TIME_MS      3000  /* T 14.8.5: no movement during the first 3 s of AS Driving */
#define STANDSTILL_MAX_RPM           10    /* both motors at or below this: not moving */
#define STANDSTILL_CONFIRM_MS        200   /* speed must stay on one side of the limit this long */
#define FINISH_STANDSTILL_TIMEOUT_MS 5000  /* mission finished but still moving after this: emergency */
#define FINISH_TS_OFF_TIMEOUT_MS     1000  /* after AS Finished the TS must be off within this */
#define MISSION_MISMATCH_TIMEOUT_MS  1000  /* Jetson mission != selected mission for this long */
#define TS_LOST_DEBOUNCE_MS          300   /* TS off during AS Ready/Driving for this long */
#define BRAKE_RELEASE_CONFIRM_MS     500   /* T 14.8.2: brakes released this long ends "EBS activated" */
#define FAULT_SETTLE_MS              100   /* reason open this long before it is logged (RES frame every ~30 ms) */

/* ── Outputs ─────────────────────────────────────────────────────────────────── */
#define WATCHDOG_PULSE_PERIOD_MS  10    /* external watchdog opens the SDC ~100 ms after the last pulse */
#define ASSI_HALF_PERIOD_MS       166   /* T 14.9.1: 2-5 Hz, 50 % duty. 166 ms on / 166 ms off = 3 Hz */
#define HEARTBEAT_LED_PERIOD_MS   1000
#define MISSION_BUTTON_DEBOUNCE_MS 200

/* ── CAN ─────────────────────────────────────────────────────────────────────── */
#define CAN_TX_PERIOD_MS          100   /* ACU, DV_status and ASF_SIGNALS frames */
#define CAN_SERVICE_PERIOD_MS     10
#define RES_PDO_FRAME_ID          0x18B /* RES frame the ACU always used for the timeout (both IDs count) */

/* RES status, autonomous_t26.dbc RES (0x191) byte 0. Seen on the car on 2026-10-31:
 * 1 = running, 5 = GO pressed, 0 = emergency stop pressed. */
#define RES_SIGNAL_RUN_BIT        0x01  /* cleared: emergency stop, the RES opened the SDC */
#define RES_SIGNAL_GO_BIT         0x04
#define CANOPEN_NMT_FRAME_ID      0x000 /* [0x01, 0x00] = start all nodes: wakes the RES up */

/* CAN programming (bootloader, see mcu_boot/.../docs/CAN_PROTOCOL.md).
 * ENTER_BOOT = 0x601, DLC 8, byte 0 = 1. Accepted only with the car safe (ACU idle, ASMS off, TS off). */
#define BOOTLOADER_ENTER_FRAME_ID 0x601
#define BOOTLOADER_MAGIC          0xB00710ADu
#define BOOTLOADER_FLAG_ADDRESS   0x2003FFFCu /* last word of RAM, kept out of the linker map */

#endif /* ACU_CONFIG_H */
