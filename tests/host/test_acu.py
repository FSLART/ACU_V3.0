"""Unit tests for the ACU state machines, run against the real firmware code.

    py -m unittest -v test_acu        (from tests/host)

Each test powers up a fresh ACU (acu_sim.World) and drives the car around it.
"""
import unittest

from acu_sim import Plant, World


class AcuTest(unittest.TestCase):
    plant_overrides = {}

    def setUp(self):
        self.w = World(Plant(**self.plant_overrides))

    def tearDown(self):
        self.w.close()

    def check(self, condition, message):
        self.assertTrue(condition, f"{message}\n--- last transitions ---\n{self.w.log()}")

    def start(self, mission="INSPECTION"):
        error = self.w.start_to_ready(mission)
        self.assertIsNone(error, error)

    def assert_safe_stop(self, within_ms):
        """HV off and EBS engaged within the given time."""
        w = self.w
        ok = w.run_until(lambda w: not w.hv_on and w.state.ignition_request == 0
                         and w.state.front_solenoid == 0 and w.state.rear_solenoid == 0, within_ms)
        self.check(ok, f"HV still {'ON' if w.hv_on else 'off'}, ign request {w.state.ignition_request}, "
                       f"solenoids {w.state.front_solenoid}/{w.state.rear_solenoid} "
                       f"{within_ms} ms later ({w.state.vehicle_state}/{w.state.autonomous_state})")


class Boot(AcuTest):
    def test_reaches_idle_with_watchdog_running(self):
        self.w.run(500)
        s = self.w.state
        self.assertEqual(s.vehicle_state, "IDLE")
        self.assertEqual(s.hw_wdt_enable, 1)
        self.assertGreaterEqual(self.w.acu.pin_changes("WDT_PULSE"), 40, "watchdog not pulsed every 10 ms")
        self.assertTrue(self.w.sdc_closed)

    def test_ign_button_in_idle_does_not_request_hv(self):
        self.w.run(1500)
        self.w.press_ign()
        self.w.run(1000)
        self.assertEqual(self.w.state.ignition_request, 0)
        self.assertFalse(self.w.hv_on)

    def test_asms_on_with_ign_held_does_not_start(self):
        self.w.run(1500)
        self.w.press_ign(hold_ms=2000)
        self.w.run(200)
        self.w.set_asms(True)
        self.w.run(500)
        self.assertEqual(self.w.state.vehicle_state, "IDLE")


class StartupSequence(AcuTest):
    def test_reaches_ready_with_ebs_engaged_and_hv_on(self):
        self.start()
        s = self.w.state
        self.assertEqual(s.as_state, "AS_STATE_READY")
        self.assertEqual((s.front_solenoid, s.rear_solenoid), (0, 0), "EBS not engaged in AS Ready")
        self.assertTrue(self.w.hv_on)
        self.assertEqual(s.res_nmt_starts, 1, "RES NMT start not sent once")

    def test_sequence_failure_cuts_hv(self):
        self.w.plant.front_ratio = 6.0   # front hydraulic too weak once the brakes are re-applied
        self.w.set_mission("INSPECTION")
        self.w.plant.jetson_mission = "INSPECTION"
        self.w.run(1500)
        self.w.set_asms(True)
        self.w.run(500)
        self.check(self.w.state.vehicle_state == "EMERGENCY", "weak front brake not detected at PRESSURE_CHECK1")
        self.assertEqual(self.w.state.seq_fail_step, "PRESSURE_CHECK1")
        self.assert_safe_stop(500)


class LowRearTank(AcuTest):
    plant_overrides = {"rear_pneumatic": 2.0}

    def test_fails_pneumatic_check(self):
        self.assertIsNotNone(self.w.start_to_ready())
        s = self.w.state
        self.assertEqual(s.vehicle_state, "EMERGENCY")
        self.assertEqual(s.seq_fail_step, "PNEUMATIC_CHECK")
        self.assertEqual(s.emergency_cause, "Pressure_checks")


class SdcNeverCloses(AcuTest):
    plant_overrides = {"sdc_chain_closed": False}

    def test_fails_with_sdc_open(self):
        self.w.run(1500)
        self.w.set_asms(True)
        self.w.run(6000)
        s = self.w.state
        self.assertEqual(s.vehicle_state, "EMERGENCY")
        self.assertEqual(s.seq_fail_step, "WDT_STP_TOGGLE_CHECK")
        self.assertEqual(s.emergency_cause, "SDC_OPEN")


class Monitoring(AcuTest):
    """AS Ready: every fault must give EMERGENCY with HV off and the EBS engaged."""

    def setUp(self):
        super().setUp()
        self.start()
        self.w.run(1000)

    def expect_emergency(self, cause, within_ms):
        ok = self.w.run_until(lambda w: w.state.vehicle_state == "EMERGENCY", within_ms)
        self.check(ok, f"no EMERGENCY within {within_ms} ms")
        self.assertEqual(self.w.state.emergency_cause, cause)
        self.assert_safe_stop(500)

    def test_sdc_open(self):
        self.w.plant.sdc_chain_closed = False
        self.expect_emergency("SDC_OPEN", 50)

    def test_vcu_lost(self):
        self.w.plant.vcu_alive = False
        self.expect_emergency("VCU_Timeout", 1200)

    def test_jetson_lost(self):
        self.w.plant.jetson_alive = False
        self.expect_emergency("Jetson_timeout", 1200)

    def test_res_lost(self):
        self.w.plant.res_alive = False
        self.expect_emergency("RES", 1200)

    def test_rear_tank_drops(self):
        self.w.plant.rear_pneumatic = 2.0
        self.expect_emergency("Pressure_checks", 50)

    def test_mission_mismatch(self):
        self.w.plant.jetson_mission = "TRACKDRIVE"
        ok = self.w.run_until(lambda w: w.state.vehicle_state == "EMERGENCY", 1300)
        self.check(ok, "mission mismatch not detected within 1.3 s")
        self.assert_safe_stop(500)

    def test_jetson_reports_emergency(self):
        self.w.plant.jetson_state = "AS_STATE_EMERGENCY"
        ok = self.w.run_until(lambda w: w.state.vehicle_state == "EMERGENCY", 200)
        self.check(ok, "Jetson reports AS Emergency but the ACU stays in AS Ready")
        self.assert_safe_stop(500)

    def test_emergency_stops_watchdog(self):
        self.w.plant.sdc_chain_closed = False
        self.w.run(100)
        before = self.w.acu.pin_changes("WDT_PULSE")
        self.w.run(500)
        self.assertEqual(self.w.state.hw_wdt_enable, 0)
        self.assertEqual(self.w.acu.pin_changes("WDT_PULSE"), before, "watchdog still pulsed in EMERGENCY")

    def test_asms_off_cuts_hv(self):
        self.w.set_asms(False)
        self.w.run(50)
        self.assertEqual(self.w.state.vehicle_state, "IDLE")
        self.assert_safe_stop(300)


class MissionEnd(AcuTest):
    """The inspection run: drive, then the Jetson reports AS Finished."""

    def setUp(self):
        super().setUp()
        self.start()
        self.w.plant.jetson_state = "AS_STATE_DRIVING"
        self.w.run(5000)

    def test_driving_releases_brakes_without_false_emergency(self):
        s = self.w.state
        self.assertNotEqual(s.vehicle_state, "EMERGENCY")
        self.assertEqual((s.front_solenoid, s.rear_solenoid), (1, 1))
        self.assertTrue(self.w.hv_on)

    def test_jetson_reports_emergency_while_driving(self):
        self.w.plant.jetson_state = "AS_STATE_EMERGENCY"
        ok = self.w.run_until(lambda w: w.state.vehicle_state == "EMERGENCY", 200)
        self.check(ok, "Jetson reports AS Emergency while driving but the ACU keeps driving")
        self.assert_safe_stop(500)

    def test_finished_turns_off_hv_and_engages_ebs(self):
        self.w.plant.jetson_state = "AS_STATE_FINISHED"
        self.assert_safe_stop(1500)
        self.assertEqual(self.w.state.autonomous_state, "Finish")
        self.assertNotEqual(self.w.state.vehicle_state, "EMERGENCY", "false emergency at mission end")

    def test_finished_escalates_when_vcu_keeps_hv(self):
        self.w.plant.vcu_ignores_hv_off = True
        self.w.plant.jetson_state = "AS_STATE_FINISHED"
        ok = self.w.run_until(lambda w: w.state.vehicle_state == "EMERGENCY", 3000)
        self.check(ok, "VCU kept HV on after AS Finished and the ACU did not open the SDC")
        self.assert_safe_stop(500)

    def test_asms_off_after_finish_keeps_hv_off(self):
        self.w.plant.jetson_state = "AS_STATE_FINISHED"
        self.w.run(2000)
        self.w.set_asms(False)
        self.w.run(500)
        self.assertEqual(self.w.state.vehicle_state, "IDLE")
        self.assert_safe_stop(300)


if __name__ == "__main__":
    unittest.main(verbosity=2)
