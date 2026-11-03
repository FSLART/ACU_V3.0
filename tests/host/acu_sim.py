"""Python side of the ACU test harness.

AcuSim  - one fresh instance of the real firmware logic compiled for the PC by build.py,
          plus the model of the car around it (sim_plant in sim.c).
World   - AcuSim with the operator actions (ASMS, IGN, mission button), the Jetson and
          event log, advanced one millisecond at a time.
ENUMS   - the state enums read straight from Core/Inc/acu.h.
CONFIG  - every #define of Core/Inc/acu_config.h.

Used by test_acu.py (unit tests) and acu_gui.py (Tkinter interface).
"""
import ctypes
import itertools
import os
import re
import shutil
import tempfile
from ctypes import wintypes
from pathlib import Path

from build import ROOT, build


def _strip_comments(text):
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"/\*.*?\*/", "", text, flags=re.S)


def parse_enums(header):
    text = _strip_comments(header.read_text(encoding="utf-8", errors="replace"))
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


def parse_config(header):
    """#define NAME value -> {NAME: number or text}."""
    config = {}
    for name, value in re.findall(r"^#define\s+(\w+)\s+([^\n/]+)", header.read_text(encoding="utf-8"), flags=re.M):
        value = value.strip()
        try:
            config[name] = int(value.rstrip("uU"), 0)
        except ValueError:
            try:
                config[name] = float(value.rstrip("fF"))
            except ValueError:
                config[name] = value
    return config


ENUMS = parse_enums(ROOT / "Core/Inc/acu.h")
CONFIG = parse_config(ROOT / "Core/Inc/acu_config.h")


def value_of(enum, name):
    return next(v for v, n in ENUMS[enum].items() if n == name)


AS = {n: v for v, n in ENUMS["AS_STATE_t"].items()}
MISSION = {n: v for v, n in ENUMS["mission_t"].items()}
FAULT = {n: v for v, n in ENUMS["fault_reason_t"].items()}


def mission_value(name):
    """"TRACKDRIVE" or "MISSION_TRACKDRIVE" -> 3"""
    return MISSION[name if name.startswith("MISSION_") else "MISSION_" + name]


def as_value(name):
    """"DRIVING" or "AS_STATE_DRIVING" -> 3"""
    return AS[name if name.startswith("AS_STATE_") else "AS_STATE_" + name]

# sim_state_t in sim.c, field for field. Third item: enum used to show the value.
STATE_FIELDS = [
    ("time_ms", ctypes.c_uint32, None),
    ("mode", ctypes.c_int32, "acu_mode_t"),
    ("as_state", ctypes.c_int32, "AS_STATE_t"),
    ("checkup_step", ctypes.c_int32, "checkup_step_t"),
    ("checkup_failed_step", ctypes.c_int32, "checkup_step_t"),
    ("finish_step", ctypes.c_int32, "finish_step_t"),
    ("fault_active", ctypes.c_int32, None),
    ("fault_reason", ctypes.c_int32, "fault_reason_t"),
    ("fault_last_reason", ctypes.c_int32, "fault_reason_t"),
    ("fault_count", ctypes.c_uint32, None),
    ("mission", ctypes.c_int32, "mission_t"),
    ("jetson_mission", ctypes.c_int32, "mission_t"),
    ("jetson_as_state", ctypes.c_int32, "AS_STATE_t"),
    ("asms_on", ctypes.c_int32, None),
    ("sdc_closed", ctypes.c_int32, None),
    ("ignition_button", ctypes.c_int32, None),
    ("ts_active", ctypes.c_int32, None),
    ("front_ebs_released", ctypes.c_int32, None),
    ("rear_ebs_released", ctypes.c_int32, None),
    ("watchdog_pulses_on", ctypes.c_int32, None),
    ("ignition_enable", ctypes.c_int32, None),
    ("ignition_request", ctypes.c_int32, None),
    ("assi_yellow", ctypes.c_int32, None),
    ("assi_blue", ctypes.c_int32, None),
    ("res_start_pending", ctypes.c_int32, None),
    ("ebs_activated", ctypes.c_int32, None),
    ("mission_finished", ctypes.c_int32, None),
    ("standstill", ctypes.c_int32, None),
    ("r2d", ctypes.c_int32, None),
    ("asb_checks_ok", ctypes.c_int32, None),
    ("brakes_engaged", ctypes.c_int32, None),
    ("speed_known", ctypes.c_int32, None),
    ("moving", ctypes.c_int32, None),
    ("monitor_active", ctypes.c_int32, None),
    ("front_tank_bar", ctypes.c_float, None),
    ("rear_tank_bar", ctypes.c_float, None),
    ("front_brake_bar", ctypes.c_float, None),
    ("rear_brake_bar", ctypes.c_float, None),
    ("age_vcu", ctypes.c_uint32, None),
    ("age_jetson", ctypes.c_uint32, None),
    ("age_res", ctypes.c_uint32, None),
    ("checkup_passed_count", ctypes.c_uint32, None),
    ("checkup_failed_count", ctypes.c_uint32, None),
    ("log_stored", ctypes.c_uint32, None),
    ("log_written", ctypes.c_uint32, None),
    ("log_dropped", ctypes.c_uint32, None),
    ("bootloader_resets", ctypes.c_uint32, None),
    ("eeprom_writes", ctypes.c_uint32, None),
    ("plant_ts_active", ctypes.c_int32, None),
    ("plant_sdc_closed", ctypes.c_int32, None),
    ("ebs_state", ctypes.c_int32, "ebs_state_t"),
]
ENUM_OF = {name: enum for name, _, enum in STATE_FIELDS}


class SimState(ctypes.Structure):
    _fields_ = [(name, ctype) for name, ctype, _ in STATE_FIELDS]


class Plant(ctypes.Structure):
    """sim_plant_t in sim.c: the car around the ACU. Write parameters, read the state."""
    _fields_ = [
        ("front_tank_bar", ctypes.c_float), ("rear_tank_bar", ctypes.c_float),
        ("front_ratio", ctypes.c_float), ("rear_ratio", ctypes.c_float),
        ("brake_tau_ms", ctypes.c_float),
        ("sdc_chain_closed", ctypes.c_int32), ("watchdog_timeout_ms", ctypes.c_int32),
        ("watchdog_stuck_closed", ctypes.c_int32), ("vcu_precharge_ms", ctypes.c_int32),
        ("ts_stuck_on", ctypes.c_int32), ("brakes_released_by_hand", ctypes.c_int32),
        ("ts_forced_off", ctypes.c_int32),
        ("motor_rpm", ctypes.c_float),
        ("vcu_alive", ctypes.c_int32), ("vcu_rpm_alive", ctypes.c_int32), ("jetson_alive", ctypes.c_int32),
        ("res_alive", ctypes.c_int32), ("steering_alive", ctypes.c_int32),
        ("front_brake_alive", ctypes.c_int32), ("rear_brake_alive", ctypes.c_int32),
        ("jetson_as_state", ctypes.c_int32), ("jetson_mission", ctypes.c_int32),
        ("front_brake_bar", ctypes.c_float), ("rear_brake_bar", ctypes.c_float),
        ("ts_active", ctypes.c_int32), ("sdc_closed", ctypes.c_int32), ("precharge_ms", ctypes.c_int32),
        ("last_pulse_ms", ctypes.c_uint32), ("pulse_changes", ctypes.c_uint32),
    ]


class AcuFrame(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int32) for n in (
        "as_state", "asms", "assi_state", "emergency", "ign", "mission_select", "acu_state",
        "emergency_cause", "cpu_temp")]


class DvStatus(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int32) for n in (
        "as_status", "asb_ebs_state", "ami_state", "steering_state", "asb_redundancy_state", "lap_counter")]


class LogRecord(ctypes.Structure):
    """log_record_t in logger.h."""
    _fields_ = [
        ("uptime_ms", ctypes.c_uint32), ("detail", ctypes.c_uint32),
        ("front_tank_bar", ctypes.c_float), ("rear_tank_bar", ctypes.c_float),
        ("front_brake_bar", ctypes.c_float), ("rear_brake_bar", ctypes.c_float),
        ("boot_number", ctypes.c_uint16),
        ("event", ctypes.c_uint8), ("reason", ctypes.c_uint8), ("mode", ctypes.c_uint8),
        ("as_state", ctypes.c_uint8), ("checkup_step", ctypes.c_uint8), ("mission", ctypes.c_uint8),
        ("sdc_closed", ctypes.c_uint8), ("asms_on", ctypes.c_uint8), ("ts_active", ctypes.c_uint8),
        ("spare", ctypes.c_uint8),
    ]


class Snapshot:
    """State at one instant. Enum fields read as their C names, e.g. s.mode == "ACU_MODE_EMERGENCY"."""

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
    static variable in the firmware starts from zero, like after a reset.
    `eeprom` (bytes) is the EEPROM content left by a previous power-on."""

    def __init__(self, eeprom=None, eeprom_present=True):
        dll = build()
        self._path = Path(tempfile.gettempdir()) / f"acu_sim_{os.getpid()}_{next(_instances)}.dll"
        shutil.copyfile(dll, self._path)
        lib = self.lib = ctypes.CDLL(str(self._path))
        lib.sim_set_pin.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.sim_get_pin.argtypes = [ctypes.c_char_p]
        lib.sim_pin_changes.argtypes = [ctypes.c_char_p]
        lib.sim_pin_changes.restype = ctypes.c_uint32
        lib.sim_step_ms.argtypes = [ctypes.c_uint32]
        lib.sim_get_state.argtypes = [ctypes.POINTER(SimState)]
        lib.sim_ble_read.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        lib.sim_ble_read.restype = ctypes.c_uint32
        lib.sim_ble_receive.argtypes = [ctypes.c_char_p]
        lib.sim_ble_commands.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        lib.sim_ble_commands.restype = ctypes.c_uint32
        lib.sim_can_rx_raw.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_can_tx_count.argtypes = [ctypes.c_uint32]
        lib.sim_can_tx_count.restype = ctypes.c_uint32
        lib.sim_can_tx_last.argtypes = [ctypes.c_uint32, ctypes.c_char_p]
        lib.sim_last_acu_frame.argtypes = [ctypes.POINTER(AcuFrame)]
        lib.sim_last_dv_status.argtypes = [ctypes.POINTER(DvStatus)]
        lib.sim_flowchart.argtypes = [ctypes.c_uint32]
        lib.sim_corrupt.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.sim_eeprom_dump.argtypes = [ctypes.c_char_p]
        lib.sim_eeprom_load.argtypes = [ctypes.c_char_p]
        lib.sim_log_record.argtypes = [ctypes.c_uint32, ctypes.POINTER(LogRecord)]
        self._buf = ctypes.create_string_buffer(262144)
        if lib.sim_state_size() != ctypes.sizeof(SimState):
            raise RuntimeError("STATE_FIELDS in acu_sim.py does not match sim_state_t in sim.c")
        if lib.sim_log_record_size() != ctypes.sizeof(LogRecord):
            raise RuntimeError("LogRecord in acu_sim.py does not match log_record_t in logger.h")
        self._raw = SimState()
        if eeprom is not None:
            lib.sim_eeprom_load(eeprom)
        lib.sim_eeprom_present(int(eeprom_present))
        lib.sim_init()
        self.plant = Plant.in_dll(lib, "sim_plant")

    def close(self):
        _kernel32.FreeLibrary(self.lib._handle)
        try:
            self._path.unlink()
        except OSError:
            pass

    def step(self, ms=1):
        self.lib.sim_step_ms(ms)

    def set_pin(self, name, level):
        self.lib.sim_set_pin(name.encode(), int(level))

    def pin(self, name):
        return self.lib.sim_get_pin(name.encode())

    def pin_changes(self, name):
        return self.lib.sim_pin_changes(name.encode())

    def state(self):
        self.lib.sim_get_state(ctypes.byref(self._raw))
        return Snapshot(self._raw)

    def can_rx(self, frame_id, data=b"", extended=False):
        data = bytes(data)
        self.lib.sim_can_rx_raw(frame_id, int(extended), data.ljust(8, b"\0"), len(data))

    def can_tx_count(self, frame_id):
        return self.lib.sim_can_tx_count(frame_id)

    def can_tx_last(self, frame_id):
        out = ctypes.create_string_buffer(8)
        length = self.lib.sim_can_tx_last(frame_id, out)
        return None if length < 0 else out.raw[:length]

    def acu_frame(self):
        frame = AcuFrame()
        return frame if self.lib.sim_last_acu_frame(ctypes.byref(frame)) == 0 else None

    def dv_status(self):
        frame = DvStatus()
        return frame if self.lib.sim_last_dv_status(ctypes.byref(frame)) == 0 else None

    def flowchart(self, bits):
        return ENUMS["AS_STATE_t"][self.lib.sim_flowchart(bits)]

    def corrupt(self, what, value):
        self.lib.sim_corrupt(what.encode(), value)

    def eeprom(self):
        out = ctypes.create_string_buffer(1024)
        self.lib.sim_eeprom_dump(out)
        return out.raw

    def log_records(self):
        records, index = [], 0
        while True:
            record = LogRecord()
            if self.lib.sim_log_record(index, ctypes.byref(record)) != 0:
                return records
            records.append(record)
            index += 1

    def ble_read(self):
        """Text the ACU sent over Bluetooth since the last call."""
        n = self.lib.sim_ble_read(self._buf, len(self._buf))
        return ctypes.string_at(self._buf, n).decode("latin-1") if n else ""

    def ble_commands(self):
        """Configuration commands sent to the RN4871 since power-on."""
        n = self.lib.sim_ble_commands(self._buf, len(self._buf))
        return ctypes.string_at(self._buf, n).decode("latin-1")

    def ble_send(self, text):
        """Text typed in the phone's Bluetooth terminal."""
        self.lib.sim_ble_receive(text.encode("latin-1"))


TRACKED = ["mode", "as_state", "checkup_step", "finish_step", "fault_reason", "ignition_request",
           "ts_active", "sdc_closed", "watchdog_pulses_on", "front_ebs_released", "rear_ebs_released"]


class World:
    """The ACU on the car, advanced one millisecond at a time."""

    def __init__(self, eeprom=None):
        self.acu = AcuSim(eeprom)
        self.plant = self.acu.plant
        self.acu.set_pin("ASMS", 0)
        self.acu.set_pin("IGN_BTN", 0)
        self._ign_release_ms = None
        self.events = []        # (time_ms, field, old, new)
        self.ble_text = ""      # everything the phone received
        self.checks = []        # functions(world) run after every millisecond
        self.state = self.acu.state()
        self._last = {f: getattr(self.state, f) for f in TRACKED}

    def close(self):
        self.acu.close()

    @property
    def now(self):
        return self.state.time_ms

    # -- operator and other nodes ------------------------------------------------
    def set_asms(self, on):
        self.acu.set_pin("ASMS", 1 if on else 0)

    def press_ign(self, hold_ms=300):
        self.acu.set_pin("IGN_BTN", 1)
        self._ign_release_ms = self.now + hold_ms

    def set_mission(self, mission):
        """Mission on the ACU and the same one on the Jetson."""
        self.acu.lib.sim_set_mission(mission_value(mission))
        self.plant.jetson_mission = mission_value(mission)

    def jetson(self, as_state):
        self.plant.jetson_as_state = as_value(as_state)

    # -- time ----------------------------------------------------------------------
    def step(self):
        if self._ign_release_ms is not None and self.now >= self._ign_release_ms:
            self.acu.set_pin("IGN_BTN", 0)
            self._ign_release_ms = None
        self.acu.step(1)
        self.ble_text += self.acu.ble_read()
        self.state = self.acu.state()
        for f in TRACKED:
            value = getattr(self.state, f)
            if value != self._last[f]:
                self.events.append((self.now, f, self._last[f], value))
                self._last[f] = value
        for check in self.checks:
            check(self)

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

    def first_time(self, field, value):
        """Time of the first event that set `field` to `value`, or None."""
        return next((t for t, f, _, new in self.events if f == field and new == value), None)

    def log(self, last=30):
        return "\n".join(f"{t:7d} ms  {f}: {a} -> {b}" for t, f, a, b in self.events[-last:])

    # -- scenarios -------------------------------------------------------------------
    def start_checkup(self, mission="TRACKDRIVE"):
        self.set_mission(mission)
        self.run(500)
        self.set_asms(True)
        self.run(1)

    def start_to_ready(self, mission="TRACKDRIVE"):
        """Mission, ASMS on, IGN at the TS step, until AS Ready. Returns an error string or None."""
        self.start_checkup(mission)
        if not self.run_until(lambda w: w.state.checkup_step in ("CHECKUP_TS_ACTIVATION", "CHECKUP_FAILED"), 10000):
            return "never reached TS_ACTIVATION\n" + self.log()
        if self.state.checkup_step == "CHECKUP_FAILED":
            return f"checkup failed: {self.state.fault_reason}\n" + self.log()
        self.press_ign()
        if not self.run_until(lambda w: w.state.as_state == "AS_STATE_READY" or w.state.fault_active, 20000):
            return "never reached AS Ready\n" + self.log()
        if self.state.fault_active:
            return f"fault during the checkup: {self.state.fault_reason}\n" + self.log()
        return None

    def ready_to_driving(self, wait_ms=5100):
        """Wait in AS Ready, then the Jetson asks for Driving (RES Go)."""
        self.run(wait_ms)
        self.jetson("AS_STATE_DRIVING")
        if not self.run_until(lambda w: w.state.as_state == "AS_STATE_DRIVING" or w.state.fault_active, 1000):
            return "never reached AS Driving\n" + self.log()
        if self.state.fault_active:
            return f"fault going to AS Driving: {self.state.fault_reason}\n" + self.log()
        return None

    def start_to_driving(self, mission="TRACKDRIVE"):
        return self.start_to_ready(mission) or self.ready_to_driving()
