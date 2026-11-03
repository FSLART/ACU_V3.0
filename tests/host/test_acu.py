"""Unit tests of the ACU firmware logic, run on the PC against the real C code.

    py -m unittest -v test_acu        (from tests/host)

Every test also checks the safety invariants below after EVERY simulated millisecond,
so any scenario that drives an output outside the rules fails, whatever the test
itself looks at. Rule references: FS Rules 2027 v1.0.
"""
import random
import unittest

from acu_sim import CONFIG, ENUMS, FAULT, AcuSim, World, mission_value

BYPASS_JETSON = CONFIG["BYPASS_JETSON_TIMEOUT"] == 1

PULSE_STEPS = {"CHECKUP_SDC_CLOSE", "CHECKUP_SDC_RECLOSE", "CHECKUP_TS_ACTIVATION", "CHECKUP_FRONT_BRAKE",
               "CHECKUP_REAR_BRAKE", "CHECKUP_BOTH_BRAKES", "CHECKUP_PASSED"}
TS_STEPS = {"CHECKUP_TS_ACTIVATION", "CHECKUP_FRONT_BRAKE", "CHECKUP_REAR_BRAKE", "CHECKUP_BOTH_BRAKES",
            "CHECKUP_PASSED"}

# ACU.emergency_cause values the Jetson already knows (autonomous_t26.dbc, VAL_ 81 EMERGENCY_cause)
CAN_CAUSE = {"NONE": 0, "SDC_OPEN": 1, "RES": 2, "PRESSURE_CHECKS": 3, "VCU_TIMEOUT": 4, "JETSON_TIMEOUT": 5,
             "ACU_WATCHDOG": 6, "UNKNOWN": 9}


def figure16(ebs_activated, mission_finished, standstill, sdc_opened_by_res, mission_selected, asms_on,
             asb_checks_ok, ts_active, r2d, brakes_engaged):
    """T 14.8.3, Figure 16, written independently of as_state.c."""
    if ebs_activated:
        if mission_finished and standstill:
            return "AS_STATE_EMERGENCY" if sdc_opened_by_res else "AS_STATE_FINISHED"
        return "AS_STATE_EMERGENCY"
    if mission_selected and asms_on and asb_checks_ok and ts_active:
        if r2d:
            return "AS_STATE_DRIVING"
        return "AS_STATE_READY" if brakes_engaged else "AS_STATE_OFF"
    return "AS_STATE_OFF"


def safety_violations(s):
    """What must never be seen at the end of a control cycle."""
    found = []
    autonomous_ok = s.mode == "ACU_MODE_AUTONOMOUS" and not s.fault_active
    driving = s.as_state == "AS_STATE_DRIVING"

    if s.front_ebs_released and not (autonomous_ok and (driving or s.checkup_step == "CHECKUP_REAR_BRAKE")):
        found.append("front EBS released outside AS Driving / the rear-only checkup step")
    if s.rear_ebs_released and not (autonomous_ok and (driving or s.checkup_step == "CHECKUP_FRONT_BRAKE")):
        found.append("rear EBS released outside AS Driving / the front-only checkup step")

    manual_sdc = s.mode == "ACU_MODE_IDLE" and s.mission == "MISSION_MANUAL" and not s.asms_on
    autonomous_sdc = (autonomous_ok and s.asms_on and s.checkup_step in PULSE_STEPS
                      and s.finish_step != "FINISH_EBS_ACTIVATED")
    if s.watchdog_pulses_on and not (manual_sdc or autonomous_sdc):
        found.append("SDC kept closed outside the T 14.4.1 conditions")

    if s.ignition_request and not s.ignition_enable:
        found.append("TS request without the enable")
    if s.ignition_enable and not (autonomous_ok and s.checkup_step in TS_STEPS
                                  and s.finish_step != "FINISH_EBS_ACTIVATED"):
        found.append("TS request enabled outside the checkup TS step / AS Ready / AS Driving")

    if s.mode == "ACU_MODE_EMERGENCY" and (s.watchdog_pulses_on or s.front_ebs_released or s.rear_ebs_released
                                           or s.ignition_request):
        found.append("EMERGENCY without the safe outputs (SDC open, EBS engaged, no TS request)")
    if bool(s.fault_active) != (s.mode == "ACU_MODE_EMERGENCY"):
        found.append("fault latched and EMERGENCY mode disagree")

    expected = figure16(s.ebs_activated, s.mission_finished, s.standstill, False, s.mission != "MISSION_MANUAL",
                        s.asms_on, s.asb_checks_ok, s.ts_active or CONFIG["SKIP_IGNITION_CHECK"], s.r2d,
                        s.brakes_engaged)
    if s.as_state != expected and not (s.ebs_activated and s.as_state == "AS_STATE_EMERGENCY"):
        found.append(f"AS state {s.as_state} but the flowchart gives {expected}")

    assi = {"AS_STATE_OFF": (False, False), "AS_STATE_READY": (True, False), "AS_STATE_FINISHED": (False, True)}
    if s.as_state in assi and (bool(s.assi_yellow), bool(s.assi_blue)) != assi[s.as_state]:
        found.append(f"ASSI wrong for {s.as_state}")
    if (driving and s.assi_blue) or (s.as_state == "AS_STATE_EMERGENCY" and s.assi_yellow):
        found.append(f"ASSI wrong colour for {s.as_state}")
    return found


class AcuTest(unittest.TestCase):
    """Fresh ACU per test, safety invariants checked every millisecond."""

    def setUp(self):
        self.w = World()
        self.w.checks.append(self.check_safety)
        self._as_state = "AS_STATE_OFF"
        self._as_since = 0

    def tearDown(self):
        self.w.close()

    def restart(self):
        self.tearDown()
        AcuTest.setUp(self)

    def check_safety(self, w):
        s = w.state
        problems = safety_violations(s)
        if s.as_state != self._as_state:
            if s.as_state == "AS_STATE_DRIVING":
                time_in_ready = w.now - self._as_since
                if self._as_state != "AS_STATE_READY" or time_in_ready < 5000:
                    problems.append(f"AS Driving after {time_in_ready} ms in {self._as_state} (T 14.8.4)")
            self._as_state, self._as_since = s.as_state, w.now
        if problems:
            self.fail(f"t={w.now} ms: " + "; ".join(problems) + "\n" + w.log())

    # helpers
    def ok(self, error):
        self.assertIsNone(error, error)

    def assert_emergency(self, reason, as_state="AS_STATE_EMERGENCY"):
        s = self.w.state
        self.assertEqual(s.mode, "ACU_MODE_EMERGENCY", self.w.log())
        self.assertEqual(s.fault_reason, reason, self.w.log())
        self.assertEqual(s.as_state, as_state)
        self.assertFalse(s.watchdog_pulses_on)
        self.assertFalse(s.front_ebs_released or s.rear_ebs_released)
        self.assertFalse(s.ignition_request)

    def run_until_fault(self, timeout_ms):
        return self.w.run_until(lambda w: w.state.fault_active, timeout_ms)


# ── T 14.8.3 flowchart ─────────────────────────────────────────────────────────

class TestFlowchart(AcuTest):
    def test_every_input_combination_matches_figure_16(self):
        for bits in range(1 << 10):
            inputs = [bool(bits >> i & 1) for i in range(10)]
            self.assertEqual(self.w.acu.flowchart(bits), figure16(*inputs), inputs)


# ── Boot and AS Off ──────────────────────────────────────────────────────────

class TestBoot(AcuTest):
    def test_boots_to_idle_as_off(self):
        self.w.run(300)
        s = self.w.state
        self.assertEqual(s.mode, "ACU_MODE_IDLE")
        self.assertEqual(s.as_state, "AS_STATE_OFF")
        self.assertEqual(s.mission, "MISSION_MANUAL")
        self.assertFalse(s.front_ebs_released or s.rear_ebs_released or s.ignition_request)

    def test_manual_mission_with_asms_off_closes_sdc(self):
        """T 14.4.1 manual driving."""
        self.w.run(300)
        self.assertTrue(self.w.state.watchdog_pulses_on)
        self.assertTrue(self.w.state.sdc_closed)

    def test_autonomous_mission_with_asms_off_keeps_sdc_open(self):
        """T 14.4.1: autonomous driving needs ASMS on and brakes engaged first."""
        self.w.set_mission("TRACKDRIVE")
        self.w.run(300)
        self.assertFalse(self.w.state.watchdog_pulses_on)
        self.assertFalse(self.w.state.sdc_closed)

    def test_can_frames_at_10_hz_with_as_off(self):
        self.w.run(1000)
        self.assertIn(self.w.acu.can_tx_count(0x51), range(9, 12))
        frame = self.w.acu.acu_frame()
        self.assertEqual((frame.as_state, frame.acu_state, frame.emergency, frame.emergency_cause), (1, 1, 0, 0))
        dv = self.w.acu.dv_status()
        self.assertEqual((dv.as_status, dv.asb_ebs_state), (1, 1))

    def test_boot_record_in_eeprom(self):
        self.w.run(100)
        records = self.w.acu.log_records()
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0].event, 1)  # LOG_EVENT_BOOT
        self.assertEqual(records[0].boot_number, 1)


class TestMissionSelection(AcuTest):
    def press(self, times=1):
        for _ in range(times):
            self.w.acu.lib.sim_press_mission_button()
            self.w.run(250)

    def test_button_cycles_through_missions_and_wraps(self):
        self.w.run(300)
        names = [n for _, n in sorted(ENUMS["mission_t"].items())]
        for expected in names[1:] + names[:1]:
            self.press()
            self.assertEqual(self.w.state.mission, expected)

    def test_button_ignored_with_asms_on(self):
        self.w.run(300)
        self.w.set_asms(True)
        self.w.run(10)
        self.press(2)
        self.assertEqual(self.w.state.mission, "MISSION_MANUAL")

    def test_bounces_within_200_ms_count_once(self):
        self.w.run(300)
        for _ in range(5):
            self.w.acu.lib.sim_press_mission_button()
            self.w.run(20)
        self.assertEqual(self.w.state.mission, "MISSION_ACCELERATION")


class TestStartConditions(AcuTest):
    def test_asms_on_with_manual_mission_does_not_start_and_opens_sdc(self):
        self.w.run(300)
        self.w.set_asms(True)
        self.w.run(500)
        s = self.w.state
        self.assertEqual(s.mode, "ACU_MODE_IDLE")
        self.assertEqual(s.checkup_step, "CHECKUP_NOT_STARTED")
        self.assertFalse(s.watchdog_pulses_on)

    def test_asms_on_with_ign_button_pressed_does_not_start(self):
        self.w.set_mission("TRACKDRIVE")
        self.w.run(300)
        self.w.acu.set_pin("IGN_BTN", 1)
        self.w.set_asms(True)
        self.w.run(500)
        self.assertEqual(self.w.state.mode, "ACU_MODE_IDLE")

    def test_asms_already_on_at_power_up_does_not_start(self):
        self.w.set_mission("TRACKDRIVE")
        self.w.set_asms(True)
        self.w.run(2000)
        self.assertEqual(self.w.state.mode, "ACU_MODE_IDLE")

    def test_res_started_once_when_the_as_starts(self):
        self.w.start_checkup()
        self.w.run(300)
        self.assertEqual(self.w.acu.can_tx_count(0x000), 1)
        self.assertEqual(self.w.acu.can_tx_last(0x000), b"\x01\x00")


# ── Initial checkup, T 15.3.1 ────────────────────────────────────────────────

class TestCheckupPasses(AcuTest):
    def test_sequence_order_and_ready(self):
        self.ok(self.w.start_to_ready())
        steps = [new for _, f, _, new in self.w.events if f == "checkup_step"]
        self.assertEqual(steps, ["CHECKUP_TANK_PRESSURE", "CHECKUP_BRAKES_ENGAGED", "CHECKUP_SDC_CLOSE",
                                 "CHECKUP_WATCHDOG_OPENS_SDC", "CHECKUP_SDC_RECLOSE", "CHECKUP_TS_ACTIVATION",
                                 "CHECKUP_FRONT_BRAKE", "CHECKUP_REAR_BRAKE", "CHECKUP_BOTH_BRAKES",
                                 "CHECKUP_PASSED"])
        s = self.w.state
        self.assertEqual(s.as_state, "AS_STATE_READY")
        self.assertTrue(s.assi_yellow and not s.assi_blue)
        self.assertTrue(s.sdc_closed and s.ts_active)
        self.w.run(200)
        self.assertEqual(self.w.acu.dv_status().asb_ebs_state, 2)

    def test_watchdog_really_opens_the_sdc(self):
        self.ok(self.w.start_to_ready())
        test_start = self.w.first_time("checkup_step", "CHECKUP_WATCHDOG_OPENS_SDC")
        test_end = self.w.first_time("checkup_step", "CHECKUP_SDC_RECLOSE")
        opened = [t for t, f, _, new in self.w.events if f == "sdc_closed" and new == 0 and test_start <= t <= test_end]
        self.assertTrue(opened, self.w.log())

    def test_sdc_closes_only_after_brakes_engaged(self):
        """T 14.4.1"""
        self.w.plant.front_ratio = 0.0  # no brake pressure yet
        self.w.run(400)
        self.w.start_checkup()
        self.w.run(1000)
        self.assertEqual(self.w.state.checkup_step, "CHECKUP_BRAKES_ENGAGED")
        self.assertFalse(self.w.state.watchdog_pulses_on)
        self.w.plant.front_ratio = 9.4
        self.assertTrue(self.w.run_until(lambda w: w.state.watchdog_pulses_on, 2000))

    def test_ign_button_before_the_ts_step_does_nothing(self):
        self.w.plant.front_ratio = 0.0
        self.w.run(400)
        self.w.start_checkup()
        self.w.press_ign()
        self.w.run(1000)
        self.assertFalse(self.w.state.ignition_request)

    def test_single_circuit_steps_wait_3_s(self):
        self.ok(self.w.start_to_ready())
        front = self.w.first_time("checkup_step", "CHECKUP_FRONT_BRAKE")
        rear = self.w.first_time("checkup_step", "CHECKUP_REAR_BRAKE")
        both = self.w.first_time("checkup_step", "CHECKUP_BOTH_BRAKES")
        self.assertGreaterEqual(rear - front, 3000)
        self.assertGreaterEqual(both - rear, 3000)

    def test_waits_for_the_ts_without_timeout(self):
        self.w.start_checkup()
        self.w.run(60000)
        self.assertEqual(self.w.state.checkup_step, "CHECKUP_TS_ACTIVATION")
        self.assertFalse(self.w.state.fault_active)


class TestCheckupFails(AcuTest):
    """Each failure: EMERGENCY with the reason, but AS Off (the EBS is not armed yet, T 14.8.1)."""

    def fail_checkup(self, reason, can_cause, setup=None, during=None, press_ign=True):
        if setup:
            setup(self.w.plant)
        if during:
            step, action = during
            fired = []

            def act(w):
                if not fired and w.state.checkup_step == step:
                    fired.append(1)
                    action(w.plant)
            self.w.checks.append(act)
        self.w.start_checkup()
        if press_ign:
            self.w.run_until(lambda w: w.state.checkup_step == "CHECKUP_TS_ACTIVATION" or w.state.fault_active, 10000)
            self.w.press_ign()
        self.assertTrue(self.run_until_fault(20000), self.w.log())
        self.w.run(5)
        self.assert_emergency(reason, as_state="AS_STATE_OFF")
        self.assertEqual(self.w.state.checkup_step, "CHECKUP_FAILED")
        self.w.run(200)
        frame = self.w.acu.acu_frame()
        self.assertEqual((frame.emergency, frame.acu_state, frame.as_state), (1, 3, 1))
        self.assertEqual(frame.emergency_cause, CAN_CAUSE[can_cause])
        self.w.run(100)
        record = self.w.acu.log_records()[-1]
        self.assertEqual(record.reason, FAULT[reason])
        self.assertEqual(record.event, 2)
        # leaving: ASMS off is enough, the EBS was never activated
        self.w.set_asms(False)
        self.w.run(10)
        self.assertEqual(self.w.state.mode, "ACU_MODE_IDLE")
        self.assertEqual(self.w.state.fault_last_reason, reason)

    def test_front_tank_low(self):
        self.fail_checkup("FAULT_CHECKUP_TANK_PRESSURE", "PRESSURE_CHECKS", lambda p: setattr(p, "front_tank_bar", 4.0))

    def test_rear_tank_low(self):
        self.fail_checkup("FAULT_CHECKUP_TANK_PRESSURE", "PRESSURE_CHECKS", lambda p: setattr(p, "rear_tank_bar", 2.0))

    def test_tank_too_high(self):
        self.fail_checkup("FAULT_CHECKUP_TANK_PRESSURE", "PRESSURE_CHECKS", lambda p: setattr(p, "front_tank_bar", 11.0))

    def test_brakes_never_engage(self):
        self.fail_checkup("FAULT_CHECKUP_BRAKES_NOT_ENGAGED", "PRESSURE_CHECKS",
                          lambda p: setattr(p, "rear_ratio", 1.0))

    def test_sdc_never_closes(self):
        self.fail_checkup("FAULT_CHECKUP_SDC_DID_NOT_CLOSE", "SDC_OPEN", lambda p: setattr(p, "sdc_chain_closed", 0))

    def test_watchdog_does_not_open_the_sdc(self):
        self.fail_checkup("FAULT_CHECKUP_WATCHDOG_DID_NOT_OPEN_SDC", "SDC_OPEN",
                          lambda p: setattr(p, "watchdog_stuck_closed", 1))

    def test_sdc_does_not_reclose(self):
        self.fail_checkup("FAULT_CHECKUP_SDC_DID_NOT_RECLOSE", "SDC_OPEN",
                          during=("CHECKUP_WATCHDOG_OPENS_SDC", lambda p: setattr(p, "sdc_chain_closed", 0)))

    def test_sdc_opens_during_ts_activation(self):
        self.fail_checkup("FAULT_CHECKUP_SDC_OPENED", "SDC_OPEN", press_ign=False,
                          during=("CHECKUP_TS_ACTIVATION", lambda p: setattr(p, "sdc_chain_closed", 0)))

    def test_front_circuit_too_weak(self):
        self.fail_checkup("FAULT_CHECKUP_FRONT_BRAKE", "PRESSURE_CHECKS",
                          during=("CHECKUP_FRONT_BRAKE", lambda p: setattr(p, "front_ratio", 3.0)))

    def test_rear_does_not_release_in_the_front_step(self):
        self.fail_checkup("FAULT_CHECKUP_FRONT_BRAKE", "PRESSURE_CHECKS",
                          during=("CHECKUP_FRONT_BRAKE", lambda p: setattr(p, "rear_brake_alive", 0)))

    def test_rear_circuit_too_weak(self):
        self.fail_checkup("FAULT_CHECKUP_REAR_BRAKE", "PRESSURE_CHECKS",
                          during=("CHECKUP_REAR_BRAKE", lambda p: setattr(p, "rear_ratio", 2.0)))

    def test_both_circuits_too_weak(self):
        self.fail_checkup("FAULT_CHECKUP_BOTH_BRAKES", "PRESSURE_CHECKS",
                          during=("CHECKUP_BOTH_BRAKES", lambda p: setattr(p, "front_ratio", 3.0)))


class TestCheckupAbort(AcuTest):
    def test_asms_off_at_any_step_goes_back_to_idle_without_fault(self):
        for step in ["CHECKUP_WATCHDOG_OPENS_SDC", "CHECKUP_TS_ACTIVATION", "CHECKUP_FRONT_BRAKE",
                     "CHECKUP_REAR_BRAKE", "CHECKUP_BOTH_BRAKES"]:
            with self.subTest(step=step):
                self.restart()
                w = self.w
                w.start_checkup()
                if step not in ("CHECKUP_WATCHDOG_OPENS_SDC", "CHECKUP_TS_ACTIVATION"):
                    w.run_until(lambda w: w.state.checkup_step == "CHECKUP_TS_ACTIVATION", 5000)
                    w.press_ign()
                self.assertTrue(w.run_until(lambda w: w.state.checkup_step == step, 10000), w.log())
                w.set_asms(False)
                w.run(20)
                self.assertEqual(w.state.mode, "ACU_MODE_IDLE")
                self.assertFalse(w.state.fault_active)
                self.assertEqual(w.state.as_state, "AS_STATE_OFF")


# ── AS Ready and R2D, T 14.8.4 ─────────────────────────────────────────────────

class TestReady(AcuTest):
    def setUp(self):
        super().setUp()
        self.ok(self.w.start_to_ready())

    def test_ready_holds_the_car(self):
        self.w.run(2000)
        s = self.w.state
        self.assertEqual(s.as_state, "AS_STATE_READY")
        self.assertFalse(s.front_ebs_released or s.rear_ebs_released)
        self.assertTrue(s.monitor_active)

    def test_jetson_echo_of_off_and_ready_is_ignored(self):
        for state in ("OFF", "READY"):
            self.w.jetson(state)
            self.w.run(6000)
            self.assertEqual(self.w.state.as_state, "AS_STATE_READY")

    def test_driving_request_before_5_s_is_an_emergency(self):
        self.w.run(4500)
        self.w.jetson("DRIVING")
        self.assertTrue(self.run_until_fault(200))
        self.assert_emergency("FAULT_R2D_BEFORE_5S")

    def test_driving_request_after_5_s_releases_the_brakes(self):
        self.ok(self.w.ready_to_driving())
        s = self.w.state
        self.assertTrue(s.front_ebs_released and s.rear_ebs_released)
        self.w.run(200)
        self.assertEqual(self.w.acu.acu_frame().as_state, 3)
        self.assertEqual(self.w.acu.dv_status().steering_state, 1)

    def test_monitoring_starts_immediately(self):
        """T 15.3.2: no blind time after the checkup."""
        self.w.plant.sdc_chain_closed = 0
        self.assertTrue(self.run_until_fault(20), self.w.log())
        self.assert_emergency("FAULT_SDC_OPENED")


class TestDriving(AcuTest):
    def setUp(self):
        super().setUp()
        self.ok(self.w.start_to_driving())

    def test_moving_in_the_first_3_s_is_an_emergency(self):
        """T 14.8.5"""
        self.w.run(1000)
        self.w.plant.motor_rpm = 300
        self.assertTrue(self.run_until_fault(1000))
        self.assert_emergency("FAULT_MOVED_BEFORE_3S")

    def test_moving_after_3_s_is_fine(self):
        self.w.run(3100)
        self.w.plant.motor_rpm = 300
        self.w.run(2000)
        self.assertEqual(self.w.state.as_state, "AS_STATE_DRIVING")
        self.assertFalse(self.w.state.fault_active)

    def test_assi_yellow_flashes_between_2_and_5_hz_with_50_percent_duty(self):
        """T 14.9.1"""
        samples = []
        for _ in range(3000):
            self.w.step()
            samples.append(self.w.state.assi_yellow)
        rising = sum(1 for a, b in zip(samples, samples[1:]) if not a and b)
        self.assertTrue(6 <= rising <= 15, rising)  # 2..5 Hz over 3 s
        self.assertAlmostEqual(sum(samples) / len(samples), 0.5, delta=0.05)


class TestMonitoring(AcuTest):
    """T 15.3.2 in AS Ready and AS Driving: fault -> AS Emergency, EBS engaged, SDC open."""

    CASES = {
        "sdc_opens": ("FAULT_SDC_OPENED", "SDC_OPEN", lambda w: setattr(w.plant, "sdc_chain_closed", 0), 50),
        "asms_off": ("FAULT_ASMS_SWITCHED_OFF", "SDC_OPEN", lambda w: w.set_asms(False), 50),
        "vcu_silent": ("FAULT_VCU_TIMEOUT", "VCU_TIMEOUT", lambda w: setattr(w.plant, "vcu_alive", 0), 1200),
        "res_silent": ("FAULT_RES_TIMEOUT", "RES", lambda w: setattr(w.plant, "res_alive", 0), 1200),
        "tank_low": ("FAULT_TANK_PRESSURE", "PRESSURE_CHECKS", lambda w: setattr(w.plant, "front_tank_bar", 4.0), 200),
        "ts_lost": ("FAULT_TS_DEACTIVATED", "UNKNOWN", lambda w: setattr(w.plant, "ts_forced_off", 1), 600),
        "jetson_emergency": ("FAULT_JETSON_EMERGENCY", "UNKNOWN", lambda w: w.jetson("EMERGENCY"), 100),
        "mission_mismatch": ("FAULT_MISSION_MISMATCH", "UNKNOWN",
                             lambda w: setattr(w.plant, "jetson_mission", mission_value("SKIDPAD")), 1300),
    }

    def check_case(self, name, driving):
        reason, cause, trigger, within_ms = self.CASES[name]
        self.ok(self.w.start_to_driving() if driving else self.w.start_to_ready())
        if driving:
            self.w.run(3500)
        trigger(self.w)
        self.assertTrue(self.run_until_fault(within_ms), f"{name}: no fault\n" + self.w.log())
        self.assert_emergency(reason)
        self.assertTrue(self.w.state.ebs_activated)
        self.w.run(300)
        frame = self.w.acu.acu_frame()
        self.assertEqual((frame.as_state, frame.emergency, frame.acu_state), (4, 1, 3))
        self.assertEqual(frame.emergency_cause, CAN_CAUSE[cause])
        self.assertFalse(self.w.state.sdc_closed)
        self.assertEqual(self.w.acu.dv_status().asb_ebs_state, 3)

    def test_ready(self):
        for name in self.CASES:
            with self.subTest(name=name):
                self.restart()
                self.check_case(name, driving=False)

    def test_driving(self):
        for name in self.CASES:
            with self.subTest(name=name):
                self.restart()
                self.check_case(name, driving=True)

    def test_brake_pressure_lost_in_ready(self):
        self.ok(self.w.start_to_ready())
        self.w.plant.front_ratio = 1.0
        self.assertTrue(self.run_until_fault(2000))
        self.assert_emergency("FAULT_BRAKE_PRESSURE_LOST")

    def test_mission_mismatch_needs_1_s(self):
        self.ok(self.w.start_to_ready())
        self.w.plant.jetson_mission = mission_value("SKIDPAD")
        self.w.run(900)
        self.assertFalse(self.w.state.fault_active)
        self.w.plant.jetson_mission = mission_value("TRACKDRIVE")
        self.w.run(2000)
        self.assertFalse(self.w.state.fault_active)

    def test_short_ts_glitch_is_tolerated(self):
        self.ok(self.w.start_to_ready())
        self.w.plant.ts_forced_off = 1
        self.w.run(150)
        self.w.plant.ts_forced_off = 0
        self.w.run(1000)
        self.assertFalse(self.w.state.fault_active)

    @unittest.skipUnless(BYPASS_JETSON, "BYPASS_JETSON_TIMEOUT = 0")
    def test_silent_jetson_is_bypassed(self):
        self.ok(self.w.start_to_ready())
        self.w.plant.jetson_alive = 0
        self.w.run(5000)
        self.assertFalse(self.w.state.fault_active)

    @unittest.skipIf(BYPASS_JETSON, "BYPASS_JETSON_TIMEOUT = 1")
    def test_silent_jetson_is_an_emergency(self):
        self.ok(self.w.start_to_ready())
        self.w.plant.jetson_alive = 0
        self.assertTrue(self.run_until_fault(CONFIG["JETSON_TIMEOUT_MS"] + 200))
        self.assert_emergency("FAULT_JETSON_TIMEOUT")

    def test_assi_blue_flashes_in_emergency(self):
        self.ok(self.w.start_to_ready())
        self.w.plant.sdc_chain_closed = 0
        self.run_until_fault(100)
        samples = []
        for _ in range(2000):
            self.w.step()
            samples.append(self.w.state.assi_blue)
        rising = sum(1 for a, b in zip(samples, samples[1:]) if not a and b)
        self.assertTrue(4 <= rising <= 10, rising)
        self.assertAlmostEqual(sum(samples) / len(samples), 0.5, delta=0.05)


# ── End of mission ───────────────────────────────────────────────────────────────

class TestFinish(AcuTest):
    def setUp(self):
        super().setUp()
        self.ok(self.w.start_to_driving())
        self.w.run(3500)

    def finish(self, timeout_ms=1000):
        self.w.jetson("FINISHED")
        return self.w.run_until(lambda w: w.state.as_state == "AS_STATE_FINISHED" or w.state.fault_active,
                                timeout_ms)

    def test_finished_at_standstill_opens_sdc_and_switches_ts_off(self):
        self.assertTrue(self.finish())
        s = self.w.state
        self.assertEqual(s.as_state, "AS_STATE_FINISHED")
        self.assertFalse(s.fault_active)
        self.assertFalse(s.watchdog_pulses_on or s.ignition_request)
        self.assertFalse(s.front_ebs_released or s.rear_ebs_released)
        self.w.run(500)
        self.assertFalse(self.w.state.ts_active)
        self.assertFalse(self.w.state.sdc_closed)
        self.assertTrue(self.w.state.assi_blue)
        self.assertEqual(self.w.acu.acu_frame().as_state, 5)
        self.assertEqual(self.w.acu.dv_status().asb_ebs_state, 3)

    def test_finished_lasts_until_asms_off_and_brakes_released(self):
        self.assertTrue(self.finish())
        self.w.run(3000)
        self.assertEqual(self.w.state.as_state, "AS_STATE_FINISHED")
        self.w.set_asms(False)
        self.w.run(3000)
        self.assertEqual(self.w.state.as_state, "AS_STATE_FINISHED")  # the EBS still holds the brakes
        self.w.plant.brakes_released_by_hand = 1
        self.assertTrue(self.w.run_until(lambda w: w.state.mode == "ACU_MODE_IDLE", 3000))
        self.assertEqual(self.w.state.as_state, "AS_STATE_OFF")

    def test_waits_for_standstill_with_the_brakes_released(self):
        self.w.plant.motor_rpm = 500
        self.w.run(500)
        self.w.jetson("FINISHED")
        self.w.run(2000)
        s = self.w.state
        self.assertEqual((s.as_state, s.finish_step), ("AS_STATE_DRIVING", "FINISH_WAITING_FOR_STANDSTILL"))
        self.w.plant.motor_rpm = 0
        self.assertTrue(self.w.run_until(lambda w: w.state.as_state == "AS_STATE_FINISHED", 1000))

    def test_never_stopping_is_an_emergency(self):
        self.w.plant.motor_rpm = 500
        self.w.run(500)
        self.w.jetson("FINISHED")
        self.assertTrue(self.run_until_fault(6000))
        self.assert_emergency("FAULT_FINISH_NOT_AT_STANDSTILL")

    def test_no_speed_from_the_vcu_is_an_emergency(self):
        self.w.plant.vcu_rpm_alive = 0
        self.w.run(600)
        self.w.jetson("FINISHED")
        self.assertTrue(self.run_until_fault(6000))
        self.assert_emergency("FAULT_FINISH_NOT_AT_STANDSTILL")

    def test_ts_still_on_after_finish_is_an_emergency(self):
        """The dangerous case seen on the car: mission over but HV still on."""
        self.w.plant.ts_stuck_on = 1
        self.assertTrue(self.finish())
        self.assertTrue(self.run_until_fault(CONFIG["FINISH_TS_OFF_TIMEOUT_MS"] + 100))
        self.assert_emergency("FAULT_TS_STILL_ON_AFTER_FINISH")

    def test_stays_finished_when_vcu_rpm_stops(self):
        self.assertTrue(self.finish())
        self.w.plant.vcu_rpm_alive = 0
        self.w.run(3000)
        self.assertEqual(self.w.state.as_state, "AS_STATE_FINISHED")


class TestFinishFromReady(AcuTest):
    def test_finished_without_driving(self):
        self.ok(self.w.start_to_ready())
        self.w.run(500)
        self.w.jetson("FINISHED")
        self.assertTrue(self.w.run_until(lambda w: w.state.as_state == "AS_STATE_FINISHED", 1000))


class TestEmergencyExit(AcuTest):
    def setUp(self):
        super().setUp()
        self.ok(self.w.start_to_driving())
        self.w.run(3500)
        self.w.plant.sdc_chain_closed = 0
        self.assertTrue(self.run_until_fault(100))
        self.w.plant.sdc_chain_closed = 1

    def test_asms_off_alone_is_not_enough(self):
        self.w.set_asms(False)
        self.w.run(5000)
        self.assertEqual(self.w.state.as_state, "AS_STATE_EMERGENCY")

    def test_brakes_released_alone_is_not_enough(self):
        self.w.plant.brakes_released_by_hand = 1
        self.w.run(5000)
        self.assertEqual(self.w.state.as_state, "AS_STATE_EMERGENCY")

    def test_asms_off_and_brakes_released_end_the_emergency(self):
        """T 14.8.2"""
        self.w.set_asms(False)
        self.w.plant.brakes_released_by_hand = 1
        self.assertTrue(self.w.run_until(lambda w: w.state.mode == "ACU_MODE_IDLE", 3000))
        s = self.w.state
        self.assertEqual((s.as_state, s.fault_reason, s.fault_last_reason),
                         ("AS_STATE_OFF", "FAULT_NONE", "FAULT_SDC_OPENED"))

    def test_a_new_run_can_start_afterwards(self):
        self.w.set_asms(False)
        self.w.plant.brakes_released_by_hand = 1
        self.w.run_until(lambda w: w.state.mode == "ACU_MODE_IDLE", 3000)
        self.w.plant.brakes_released_by_hand = 0
        self.w.jetson("OFF")
        self.w.run(500)
        self.ok(self.w.start_to_ready())


# ── Fault log, CAN, bootloader, robustness ─────────────────────────────────────

class TestFaultLog(AcuTest):
    def test_fault_record_has_the_details(self):
        self.ok(self.w.start_to_ready())
        self.w.plant.sdc_chain_closed = 0
        self.run_until_fault(100)
        self.w.run(100)
        record = self.w.acu.log_records()[-1]
        self.assertEqual(record.reason, FAULT["FAULT_SDC_OPENED"])
        self.assertEqual(ENUMS["acu_mode_t"][record.mode], "ACU_MODE_AUTONOMOUS")
        self.assertEqual(ENUMS["AS_STATE_t"][record.as_state], "AS_STATE_READY")
        self.assertEqual(ENUMS["mission_t"][record.mission], "MISSION_TRACKDRIVE")
        self.assertAlmostEqual(record.front_tank_bar, 7.66, delta=0.05)
        self.assertGreater(record.front_brake_bar, 50)

    def test_log_survives_a_power_cycle(self):
        self.w.plant.front_tank_bar = 4.0
        self.w.start_checkup()
        self.run_until_fault(100)
        self.w.run(100)
        eeprom = self.w.acu.eeprom()
        self.w.close()
        self.w = World(eeprom=eeprom)
        self.w.run(100)
        records = self.w.acu.log_records()
        self.assertEqual([r.event for r in records], [1, 2, 1])
        self.assertEqual(records[1].reason, FAULT["FAULT_CHECKUP_TANK_PRESSURE"])
        self.assertEqual(records[2].boot_number, 2)

    def test_ring_keeps_the_last_28_records(self):
        self.w.plant.front_tank_bar = 4.0
        self.w.set_mission("TRACKDRIVE")
        self.w.run(100)
        for _ in range(35):
            self.w.set_asms(True)
            self.w.run(30)
            self.w.set_asms(False)
            self.w.run(30)
        self.w.run(200)
        self.assertEqual(self.w.state.fault_count, 35)
        self.assertEqual(self.w.state.log_dropped, 0)
        eeprom = self.w.acu.eeprom()
        self.w.close()
        self.w = World(eeprom=eeprom)
        self.w.run(100)
        self.assertEqual(len(self.w.acu.log_records()), 28)

    def test_works_without_eeprom(self):
        acu = AcuSim(eeprom_present=False)
        acu.step(100)
        self.assertEqual(len(acu.log_records()), 1)  # kept in RAM
        self.assertEqual(acu.state().log_written, 0)
        acu.close()

    def test_bluetooth_flush_prints_the_records(self):
        self.w.run(4000)  # RN4871 configured
        self.w.plant.front_tank_bar = 4.0
        self.w.start_checkup()
        self.run_until_fault(100)
        self.w.run(1500)
        self.w.ble_text = ""
        self.w.acu.ble_send("flush\r\n")
        self.w.run(1500)
        self.assertIn("=== LOG DE FALHAS (2) ===", self.w.ble_text)
        self.assertIn("CHECKUP_TANK_PRESSURE", self.w.ble_text)
        self.assertIn("=== FIM DO LOG ===", self.w.ble_text)


class TestCanInterface(AcuTest):
    """What the Jetson and the VCU see must keep its meaning."""

    def test_acu_state_values(self):
        self.w.run(200)
        self.assertEqual(self.w.acu.acu_frame().acu_state, 1)
        self.w.start_checkup()
        self.w.run(200)
        self.assertEqual(self.w.acu.acu_frame().acu_state, 2)

    def test_ign_follows_the_button_only_when_enabled(self):
        self.w.start_checkup()
        self.w.run_until(lambda w: w.state.checkup_step == "CHECKUP_TS_ACTIVATION", 5000)
        self.w.press_ign()
        self.w.run(400)
        self.assertEqual(self.w.acu.acu_frame().ign, 1)

    def test_jetson_frame_too_short_is_ignored(self):
        self.w.plant.jetson_alive = 0
        self.w.run(300)
        self.w.acu.can_rx(0x61, b"\x03")
        self.w.run(1)
        self.assertGreater(self.w.state.age_jetson, 200)

    def test_extended_frame_with_a_jetson_id_is_ignored(self):
        self.w.plant.jetson_alive = 0
        self.w.run(300)
        self.w.acu.can_rx(0x61, b"\x33\x00\x00\x00\x00", extended=True)
        self.w.run(1)
        self.assertGreater(self.w.state.age_jetson, 200)

    def test_brake_pressures_on_asf_signals_are_clamped(self):
        self.w.run(300)
        data = self.w.acu.can_tx_last(0x511)
        self.assertEqual(data[2], 255)  # front hydraulic ~72 bar, signal max 25.5 bar
        self.assertEqual(data[0], 76)   # front tank 7.66 bar


class TestBootloader(AcuTest):
    ENTER = (0x601, b"\x01\x00\x00\x00\x00\x00\x00\x00")

    def test_enter_boot_resets_when_parked(self):
        self.w.run(100)
        self.w.acu.can_rx(*self.ENTER)
        self.w.run(2)
        self.assertEqual(self.w.state.bootloader_resets, 1)

    def test_ignored_with_asms_on_or_during_the_as(self):
        self.w.set_mission("TRACKDRIVE")
        self.w.run(100)
        self.w.set_asms(True)
        self.w.run(5)
        self.w.acu.can_rx(*self.ENTER)
        self.w.run(500)
        self.assertEqual(self.w.state.bootloader_resets, 0)

    def test_wrong_frames_are_ignored(self):
        self.w.run(100)
        self.w.acu.can_rx(0x601, b"\x01")
        self.w.acu.can_rx(0x601, b"\x00" * 8)
        self.w.run(5)
        self.assertEqual(self.w.state.bootloader_resets, 0)


class TestRobustness(AcuTest):
    """Redundancy: a corrupted state variable must end in EMERGENCY, never in an unknown state."""

    def corrupt(self, what, value):
        self.w.checks.clear()  # the invariants can't hold during the corrupted cycle itself
        self.w.acu.corrupt(what, value)
        self.w.run(2)

    def test_corrupted_main_state(self):
        self.ok(self.w.start_to_ready())
        self.corrupt("mode", 77)
        self.assert_emergency("FAULT_INVALID_STATE")

    def test_corrupted_checkup_step(self):
        self.w.start_checkup()
        self.w.run(5)
        self.corrupt("checkup", 99)
        self.assertEqual(self.w.state.mode, "ACU_MODE_EMERGENCY")
        self.assertEqual(self.w.state.fault_reason, "FAULT_INVALID_STATE")

    def test_corrupted_finish_step(self):
        self.ok(self.w.start_to_driving())
        self.corrupt("finish", 42)
        self.assert_emergency("FAULT_INVALID_STATE")

    def test_random_operation(self):
        """Random operator actions and failures, safety invariants checked every millisecond."""
        actions = [
            lambda w: w.set_asms(True), lambda w: w.set_asms(False), lambda w: w.press_ign(),
            lambda w: w.acu.lib.sim_press_mission_button(), lambda w: w.set_mission("TRACKDRIVE"),
            lambda w: w.jetson("DRIVING"), lambda w: w.jetson("FINISHED"), lambda w: w.jetson("OFF"),
            lambda w: w.jetson("EMERGENCY"),
            lambda w: setattr(w.plant, "sdc_chain_closed", random.random() < 0.8),
            lambda w: setattr(w.plant, "motor_rpm", random.choice([0, 0, 0, 200])),
            lambda w: setattr(w.plant, "front_tank_bar", random.choice([7.66, 7.66, 4.0])),
            lambda w: setattr(w.plant, "brakes_released_by_hand", random.random() < 0.3),
            lambda w: setattr(w.plant, "vcu_alive", random.random() < 0.9),
            lambda w: setattr(w.plant, "ts_forced_off", random.random() < 0.1),
        ]
        for seed in range(6):
            with self.subTest(seed=seed):
                self.restart()
                random.seed(seed)
                for _ in range(120):
                    random.choice(actions)(self.w)
                    self.w.run(random.choice([5, 50, 300, 1000, 3000]))


class TestBluetooth(AcuTest):
    def test_module_configuration(self):
        self.w.run(4000)
        commands = self.w.acu.ble_commands()
        self.assertTrue(commands.startswith("$$$SN,ACU TEK-26e\r\nSS,C0\r\nSR,01\r\nR,1\r\n"), commands)

    def test_status_block_and_event_lines(self):
        self.w.run(5000)
        text = self.w.ble_text
        self.assertIn("ACU TEK-26e ligada", text)
        self.assertIn("Modo: IDLE   AS: OFF", text)
        if BYPASS_JETSON:
            self.assertIn("BYPASS_JETSON_TIMEOUT = 1", text)
        self.w.plant.front_tank_bar = 4.0
        self.w.start_checkup()
        self.run_until_fault(100)
        self.w.run(300)
        self.assertIn("Falha NONE->CHECKUP_TANK_PRESSURE", self.w.ble_text)
        self.assertIn("pressao dos tanques EBS fora dos limites", self.w.ble_text)

    def test_stop_and_start(self):
        self.w.run(5000)  # RN4871 configured
        self.w.acu.ble_send("stop\r\n")
        self.w.run(100)
        self.w.ble_text = ""
        self.w.run(3000)
        self.assertEqual(self.w.ble_text, "")
        self.w.acu.ble_send("start\r\n")
        self.w.run(1500)
        self.assertIn("== ACU", self.w.ble_text)


if __name__ == "__main__":
    unittest.main()
