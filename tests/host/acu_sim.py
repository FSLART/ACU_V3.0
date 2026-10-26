"""Python side of the ACU test harness.

AcuSim  - one fresh instance of the real firmware logic (APP.c, state_machine.c,
          Autonomous_functions.c, ...) compiled for the PC by build.py.
World   - AcuSim plus simple models of what surrounds the ACU on the car:
          VCU (HV), external watchdog + SDC, EBS brakes, Jetson, RES, sensors.
ENUMS   - the state enums read straight from Core/Inc/main.h.

Used by test_acu.py (unit tests) and acu_gui.py (Tkinter interface).
"""
import ctypes
import itertools
import os
import re
import shutil
import tempfile
from ctypes import wintypes
from dataclasses import dataclass
from pathlib import Path

from build import ROOT, build


def parse_enums(header):
    text = header.read_text(encoding="utf-8", errors="replace")
    text = re.sub(r"//[^\n]*", "", text)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    enums = {}
    for body, name in re.findall(r"typedef\s+enum\s*\{(.*?)\}\s*(\w+)\s*;", text, flags=re.S):
        value, names = -1, {}
        for item in (i.strip() for i in body.split(",")):
            if not item:
                continue
            if "=" in item:
                item, raw = (s.strip() for s in item.split("=", 1))
                value = int(raw, 0)
            else:
                value += 1
            names[value] = item
        enums[name] = names
    return enums


ENUMS = parse_enums(ROOT / "Core/Inc/main.h")


def enum_value(enum, name):
    return next(v for v, n in ENUMS[enum].items() if n == name)


AS = {n: v for v, n in ENUMS["AS_STATE_t"].items()}
MISSION = {n: v for v, n in ENUMS["current_mission_t"].items()}

# sim_state_t in sim.c, field for field. Second item: enum used to show the value.
STATE_FIELDS = [
    ("time_ms", ctypes.c_uint32, None),
    ("vehicle_state", ctypes.c_int32, "Main_state_machine_t"),
    ("autonomous_state", ctypes.c_int32, "Autonomous_System_states_t"),
    ("acu_state", ctypes.c_int32, "cant_acu_state_t"),
    ("startup_seq", ctypes.c_int32, "startup_sequence_state_t"),
    ("emergency_cause", ctypes.c_int32, "Emergency_cause_t"),
    ("as_state", ctypes.c_int32, "AS_STATE_t"),
    ("current_mission", ctypes.c_int32, "current_mission_t"),
    ("jetson_mission", ctypes.c_int32, "current_mission_t"),
    ("front_solenoid", ctypes.c_int32, None),
    ("rear_solenoid", ctypes.c_int32, None),
    ("ignition_request", ctypes.c_int32, None),
    ("ignition_enable", ctypes.c_int32, None),
    ("ignition_status", ctypes.c_int32, None),
    ("ignition_pin", ctypes.c_int32, None),
    ("hw_wdt_enable", ctypes.c_int32, None),
    ("asms", ctypes.c_int32, None),
    ("sdc_feedback", ctypes.c_int32, None),
    ("vcu_sdc", ctypes.c_int32, None),
    ("assi_state", ctypes.c_int32, "AS_STATE_t"),
    ("emergency", ctypes.c_int32, None),
    ("rpm", ctypes.c_int32, None),
    ("front_pneumatic", ctypes.c_float, None),
    ("front_hydraulic", ctypes.c_float, None),
    ("rear_pneumatic", ctypes.c_float, None),
    ("rear_hydraulic", ctypes.c_float, None),
    ("vcu_last", ctypes.c_uint32, None),
    ("jetson_last", ctypes.c_uint32, None),
    ("res_last", ctypes.c_uint32, None),
    ("rear_pressure_last", ctypes.c_uint32, None),
    ("dir_last", ctypes.c_uint32, None),
    ("seq_fail_step", ctypes.c_int32, "startup_sequence_state_t"),
    ("seq_fail_count", ctypes.c_uint32, None),
    ("seq_fail_ms", ctypes.c_uint32, None),
    ("res_nmt_starts", ctypes.c_uint32, None),
    ("can_rx_pending", ctypes.c_uint32, None),
]
ENUM_OF = {name: enum for name, _, enum in STATE_FIELDS}


class SimState(ctypes.Structure):
    _fields_ = [(name, ctype) for name, ctype, _ in STATE_FIELDS]


class Snapshot:
    """State at one instant. Enum fields read as their C names, e.g. s.vehicle_state == "EMERGENCY"."""

    def __init__(self, raw):
        self.raw = {name: getattr(raw, name) for name, _, _ in STATE_FIELDS}

    def __getattr__(self, name):
        value = self.raw[name]
        enum = ENUM_OF[name]
        return ENUMS[enum].get(value, f"?{value}") if enum else value


_instances = itertools.count()
_kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
_kernel32.FreeLibrary.argtypes = [wintypes.HMODULE]


class AcuSim:
    """A powered-on ACU. Each instance loads its own copy of the DLL, so every
    static variable in the firmware starts from zero, like after a reset."""

    def __init__(self):
        dll = build()
        self._path = Path(tempfile.gettempdir()) / f"acu_sim_{os.getpid()}_{next(_instances)}.dll"
        shutil.copyfile(dll, self._path)
        lib = self.lib = ctypes.CDLL(str(self._path))
        lib.sim_set_pin.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.sim_get_pin.argtypes = [ctypes.c_char_p]
        lib.sim_pin_changes.argtypes = [ctypes.c_char_p]
        lib.sim_pin_changes.restype = ctypes.c_uint32
        lib.sim_set_pneumatic.argtypes = [ctypes.c_float, ctypes.c_float]
        lib.sim_can_rx_brake_pressures.argtypes = [ctypes.c_float, ctypes.c_float]
        lib.sim_step_ms.argtypes = [ctypes.c_uint32]
        lib.sim_get_state.argtypes = [ctypes.POINTER(SimState)]
        if lib.sim_state_size() != ctypes.sizeof(SimState):
            raise RuntimeError("STATE_FIELDS in acu_sim.py does not match sim_state_t in sim.c")
        self._raw = SimState()
        lib.sim_init()

    def close(self):
        _kernel32.FreeLibrary(self.lib._handle)
        try:
            self._path.unlink()
        except OSError:
            pass

    def set_pin(self, name, level):
        self.lib.sim_set_pin(name.encode(), int(level))

    def pin(self, name):
        return self.lib.sim_get_pin(name.encode())

    def pin_changes(self, name):
        return self.lib.sim_pin_changes(name.encode())

    def state(self):
        self.lib.sim_get_state(ctypes.byref(self._raw))
        return Snapshot(self._raw)


@dataclass
class Plant:
    """Everything around the ACU. Defaults: the car as measured on the bench."""
    front_pneumatic: float = 7.66     # bar, EBS tanks
    rear_pneumatic: float = 3.36
    front_ratio: float = 9.4          # hydraulic / pneumatic with the EBS applied
    rear_ratio: float = 7.2
    brake_tau_ms: float = 150.0       # hydraulic pressure response
    sdc_chain_closed: bool = True     # rest of the shutdown circuit (ASMS, RES, TSMS, ...)
    wdt_timeout_ms: int = 100         # external watchdog opens the SDC without pulses
    vcu_precharge_ms: int = 300
    vcu_ignores_hv_off: bool = False  # fault: VCU keeps HV on after the ACU drops `ign`
    vcu_alive: bool = True
    jetson_alive: bool = True
    res_alive: bool = True
    steering_alive: bool = True
    pressures_alive: bool = True
    jetson_state: str = "AS_STATE_OFF"
    jetson_mission: str = "MANUAL"


TRACKED = ["vehicle_state", "autonomous_state", "acu_state", "startup_seq",
           "emergency_cause", "as_state", "ignition_request", "ignition_status"]


class World:
    """The ACU on the car, advanced one millisecond at a time."""

    def __init__(self, plant=None, mission="MANUAL"):
        self.plant = plant or Plant()
        self.acu = AcuSim()
        self.acu.set_pin("ASMS", 0)
        self.acu.set_pin("IGN_BTN", 0)
        self.acu.lib.sim_set_mission(MISSION[mission])
        self.hv_on = False
        self.sdc_closed = True
        self.watchdog_pulsing = True
        self.front_hyd = self.plant.front_ratio * self.plant.front_pneumatic
        self.rear_hyd = self.plant.rear_ratio * self.plant.rear_pneumatic
        self._precharge_ms = 0
        self._wdt_changes = 0
        self._wdt_last_ms = 0
        self._ign_release_ms = None
        self.events = []          # (time_ms, field, old, new)
        self.state = self.acu.state()
        self._last = {f: getattr(self.state, f) for f in TRACKED}

    def close(self):
        self.acu.close()

    @property
    def now(self):
        return self.state.time_ms

    # -- operator ----------------------------------------------------------

    def set_asms(self, on):
        self.acu.set_pin("ASMS", 1 if on else 0)

    def press_ign(self, hold_ms=300):
        self.acu.set_pin("IGN_BTN", 1)
        self._ign_release_ms = self.now + hold_ms

    def set_mission(self, mission):
        self.acu.lib.sim_set_mission(MISSION[mission])

    # -- time ----------------------------------------------------------------

    def step(self):
        p, acu, lib, now = self.plant, self.acu, self.acu.lib, self.now

        if self._ign_release_ms is not None and now >= self._ign_release_ms:
            acu.set_pin("IGN_BTN", 0)
            self._ign_release_ms = None

        changes = acu.pin_changes("WDT_PULSE")
        if changes != self._wdt_changes:
            self._wdt_changes, self._wdt_last_ms = changes, now
        self.watchdog_pulsing = now - self._wdt_last_ms <= p.wdt_timeout_ms
        self.sdc_closed = p.sdc_chain_closed and self.watchdog_pulsing
        acu.set_pin("SDC_FEEDBACK", 0 if self.sdc_closed else 1)   # active low

        hv_requested = self.state.raw["ignition_request"] == 1
        if not self.sdc_closed:
            self.hv_on, self._precharge_ms = False, 0               # SDC open: TS off by hardware
        elif hv_requested and not self.hv_on:
            self._precharge_ms += 1
            self.hv_on = self._precharge_ms >= p.vcu_precharge_ms
        elif not hv_requested and not p.vcu_ignores_hv_off:
            self.hv_on, self._precharge_ms = False, 0

        k = 1.0 / p.brake_tau_ms
        front_target = p.front_ratio * p.front_pneumatic if acu.pin("Front_Solenoid") == 0 else 0.0
        rear_target = p.rear_ratio * p.rear_pneumatic if acu.pin("Rear_Solenoid") == 0 else 0.0
        self.front_hyd += (front_target - self.front_hyd) * k
        self.rear_hyd += (rear_target - self.rear_hyd) * k

        lib.sim_set_pneumatic(p.front_pneumatic, p.rear_pneumatic)
        if p.pressures_alive and now % 10 == 0:
            lib.sim_can_rx_brake_pressures(self.front_hyd, self.rear_hyd)
        if p.steering_alive and now % 10 == 5:
            lib.sim_can_rx_steering()
        if p.jetson_alive and now % 20 == 2:
            lib.sim_can_rx_jetson(AS[p.jetson_state], MISSION[p.jetson_mission])
        if p.vcu_alive and now % 20 == 7:
            lib.sim_can_rx_vcu(int(self.hv_on), int(self.sdc_closed))
        if p.res_alive and now % 50 == 13:
            lib.sim_can_rx_res()

        lib.sim_step_ms(1)
        self.state = acu.state()
        for f in TRACKED:
            value = getattr(self.state, f)
            if value != self._last[f]:
                self.events.append((self.now, f, self._last[f], value))
                self._last[f] = value

    def run(self, ms):
        for _ in range(int(ms)):
            self.step()

    def run_until(self, condition, timeout_ms):
        """Step until condition(world) is true. Returns False on timeout."""
        end = self.now + timeout_ms
        while self.now < end:
            self.step()
            if condition(self):
                return True
        return False

    def log(self, last=25):
        return "\n".join(f"{t:7d} ms  {f}: {a} -> {b}" for t, f, a, b in self.events[-last:])

    # -- scenarios -------------------------------------------------------------

    def start_to_ready(self, mission="INSPECTION"):
        """Mission select, ASMS on, IGN at HV activation, until the ACU monitors in AS Ready.
        Returns an error string, or None on success."""
        self.set_mission(mission)
        self.plant.jetson_mission = mission
        self.run(1500)
        self.set_asms(True)
        if not self.run_until(lambda w: w.state.startup_seq in ("HV_ACTIVATION", "SEQUENCE_ERROR"), 10000):
            return "never reached HV_ACTIVATION\n" + self.log()
        if self.state.startup_seq == "SEQUENCE_ERROR":
            self.run(10)   # EMERGENCY is set on the next pass through the sequence
            return f"sequence failed at {self.state.seq_fail_step}\n" + self.log()
        self.press_ign()
        if not self.run_until(lambda w: w.state.autonomous_state == "Monitor_sequence"
                              or w.state.vehicle_state == "EMERGENCY", 20000):
            return "never reached Monitor_sequence\n" + self.log()
        if self.state.vehicle_state == "EMERGENCY":
            return f"emergency during start ({self.state.emergency_cause})\n" + self.log()
        return None
