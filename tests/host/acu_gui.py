"""Tkinter interface for the ACU simulator: the real firmware logic running on the PC.

    py acu_gui.py          (from tests/host; needs `py -m pip install ziglang` once)

Left: the car around the ACU (operator, SDC, Jetson, VCU, sensors, faults).
Middle: the four state machines, the active state highlighted, and what makes each state move on.
Right: live signals, transition log, and the unit tests.
"""
import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk

from acu_sim import ENUMS, MISSION, AS, Plant, World
from build import ROOT

HERE = Path(__file__).resolve().parent
TICK_MS = 20


def parse_defines(*files):
    values = {}
    for f in files:
        text = f.read_text(encoding="utf-8", errors="replace")
        for name, value in re.findall(r"^#define\s+(\w+)\s+([\d.]+)f?\b", text, re.M):
            values[name] = float(value)
    return values


D = parse_defines(ROOT / "Core/Src/Autonomous_functions.c", ROOT / "Core/Inc/Autonomous_functions.h",
                  ROOT / "Core/Src/state_machine.c")


def d(name, scale=1.0):
    return f"{D[name] / scale:g}" if name in D else "?"


# What makes each state move on (mirrors state_machine.c / Autonomous_functions.c).
EXITS = {
    "Main_state_machine_t": {
        "Start": "-> IDLE at boot",
        "IDLE": "-> AS_ON: ASMS on\n(IGN released)",
        "AS_ON": "-> IDLE: ASMS off\n-> EMERGENCY: fault",
        "EMERGENCY": "EBS on, HV off, WDT stopped\n-> IDLE: ASMS off, rpm < 10",
    },
    "Autonomous_System_states_t": {
        "OFF": "-> Initial_Sequence\nwhen AS_ON",
        "Initial_Sequence": "-> Monitor: AS Ready\nfor 500 ms",
        "Monitor_sequence": "-> Finish: Jetson FINISHED\n-> EMERGENCY: SDC, timeouts,\ntanks, brakes, mission,\nJetson EMERGENCY",
        "Finish": f"EBS on, HV off\n-> EMERGENCY: VCU keeps HV\n> {d('FINISH_HV_OFF_TIMEOUT_MS', 1000)} s",
        "AS_Emergency": "(not used)",
    },
    "startup_sequence_state_t": {
        "WDT_TOGGLE_CHECK": "stop the WDT pulses",
        "WDT_STP_TOGGLE_CHECK": f"SDC closed -> WDT on\nelse error after {d('TIMEOUT_WDT_MS', 1000)} s",
        "PNEUMATIC_CHECK": f"tanks F {d('EBS_MIN_BAR')}-{d('EBS_MAX_BAR')}\nR {d('EBS_REAR_MIN_BAR')}-{d('EBS_MAX_BAR')} bar",
        "PRESSURE_CHECK1": f"hyd >= gain x pneu\nF {d('EBS_FRONT_HYD_GAIN')}x  R {d('EBS_REAR_HYD_GAIN_INITIAL')}x",
        "HV_ACTIVATION": "IGN button, then\nVCU confirms HV",
        "PRESSURE_CHECK_FRONT": f"front held, rear <= {d('EBS_HYD_UNLOADED_BAR')}\n{d('SOLENOID_MIN_DELAY_MS', 1000)}-{d('TIMEOUT_SOLENOID_MS', 1000)} s",
        "PRESSURE_CHECK_REAR": f"rear held, front <= {d('EBS_HYD_UNLOADED_BAR')}\n{d('SOLENOID_MIN_DELAY_MS', 1000)}-{d('TIMEOUT_SOLENOID_MS', 1000)} s",
        "PRESSURE_CHECK2": "both held -> AS Ready",
        "SEQUENCE_ERROR": "-> EMERGENCY",
    },
    "AS_STATE_t": {
        "AS_STATE_OFF": "ASSI off",
        "AS_STATE_READY": "yellow",
        "AS_STATE_DRIVING": "yellow flashing",
        "AS_STATE_EMERGENCY": "blue flashing",
        "AS_STATE_FINISHED": "blue",
    },
}

MACHINES = [  # (enum, state field, title, boxes per row, draw arrows between neighbours)
    ("Main_state_machine_t", "vehicle_state", "Vehicle_state_machine", 4, True),
    ("Autonomous_System_states_t", "autonomous_state", "Autonomous_state", 5, True),
    ("startup_sequence_state_t", "startup_seq", "startup_sequence_state", 5, True),
    ("AS_STATE_t", "as_state", "t24.Autonomous_State (Jetson / ACU)", 5, False),
]
DANGER = {"EMERGENCY", "SEQUENCE_ERROR", "AS_STATE_EMERGENCY", "AS_Emergency"}


def inspection_script(app):
    """Inspection run: start, drive, finish, ASMS off. Yields between checks."""
    w, p = app.world, app.world.plant
    app.note("script: INSPECTION selected, ASMS on in 1.5 s")
    app.set_mission("INSPECTION")
    p.jetson_state = "AS_STATE_OFF"
    app.sync_controls()
    start = w.now
    while w.now - start < 1500:
        yield
    app.asms.set(True)
    app.apply_controls()
    while w.state.startup_seq not in ("HV_ACTIVATION", "SEQUENCE_ERROR"):
        yield
    if w.state.startup_seq == "SEQUENCE_ERROR":
        app.note(f"script: sequence failed at {w.state.seq_fail_step}")
        return
    app.note("script: IGN pressed")
    w.press_ign()
    while w.state.autonomous_state != "Monitor_sequence":
        if w.state.vehicle_state == "EMERGENCY":
            app.note("script: emergency during start")
            return
        yield
    start = w.now
    while w.now - start < 1000:
        yield
    app.note("script: Jetson DRIVING for 8 s")
    p.jetson_state = "AS_STATE_DRIVING"
    app.sync_controls()
    start = w.now
    while w.now - start < 8000:
        yield
    app.note("script: Jetson FINISHED")
    p.jetson_state = "AS_STATE_FINISHED"
    app.sync_controls()
    start = w.now
    while w.now - start < 3000:
        yield
    app.note(f"script: 3 s after FINISHED, HV is {'ON  <-- DANGER' if w.hv_on else 'off'}; ASMS off")
    app.asms.set(False)
    app.apply_controls()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ACU V3 simulator - firmware state machines")
        self.geometry("1500x900")
        self.world = None
        self.running = False
        self.script = None
        self.events_shown = 0
        self.test_queue = queue.Queue()

        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)
        self._toolbar()
        self._controls()
        self._diagram()
        self._side()
        self.reset()
        self.after(TICK_MS, self.tick)

    # -- layout ---------------------------------------------------------------

    def _toolbar(self):
        bar = ttk.Frame(self, padding=4)
        bar.grid(row=0, column=0, columnspan=3, sticky="ew")
        ttk.Button(bar, text="Power cycle", command=self.reset).pack(side="left")
        self.run_button = ttk.Button(bar, text="Run", command=self.toggle_run)
        self.run_button.pack(side="left", padx=4)
        ttk.Button(bar, text="Step 100 ms", command=lambda: self.advance(100)).pack(side="left")
        ttk.Label(bar, text="  speed").pack(side="left")
        self.speed = tk.StringVar(value="x1")
        ttk.Combobox(bar, textvariable=self.speed, values=["x0.2", "x1", "x5", "x20"], width=5,
                     state="readonly").pack(side="left")
        ttk.Button(bar, text="Inspection run (script)", command=self.start_script).pack(side="left", padx=12)
        ttk.Button(bar, text="Run unit tests", command=self.run_tests).pack(side="left")
        self.clock = ttk.Label(bar, text="", font=("Consolas", 12, "bold"))
        self.clock.pack(side="right")

    def _controls(self):
        f = ttk.Frame(self, padding=6)
        f.grid(row=1, column=0, sticky="ns")
        self.asms = tk.BooleanVar()
        self.mission = tk.StringVar(value="INSPECTION")
        self.sdc_chain = tk.BooleanVar(value=True)
        self.jetson_alive = tk.BooleanVar(value=True)
        self.jetson_state = tk.StringVar(value="AS_STATE_OFF")
        self.jetson_mission = tk.StringVar(value="INSPECTION")
        self.vcu_alive = tk.BooleanVar(value=True)
        self.vcu_stuck = tk.BooleanVar(value=False)
        self.res_alive = tk.BooleanVar(value=True)
        self.steering_alive = tk.BooleanVar(value=True)
        self.pressures_alive = tk.BooleanVar(value=True)
        self.pneu_front = tk.DoubleVar(value=Plant.front_pneumatic)
        self.pneu_rear = tk.DoubleVar(value=Plant.rear_pneumatic)
        self.ratio_front = tk.DoubleVar(value=Plant.front_ratio)
        self.ratio_rear = tk.DoubleVar(value=Plant.rear_ratio)

        missions = [ENUMS["current_mission_t"][k] for k in sorted(ENUMS["current_mission_t"])]
        as_states = [ENUMS["AS_STATE_t"][k] for k in sorted(ENUMS["AS_STATE_t"])]

        def section(text):
            ttk.Label(f, text=text, font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(10, 2))

        def check(text, var):
            ttk.Checkbutton(f, text=text, variable=var, command=self.apply_controls).pack(anchor="w")

        def combo(text, var, values):
            row = ttk.Frame(f)
            row.pack(anchor="w", fill="x")
            ttk.Label(row, text=text, width=12).pack(side="left")
            box = ttk.Combobox(row, textvariable=var, values=values, state="readonly", width=20)
            box.pack(side="left")
            box.bind("<<ComboboxSelected>>", lambda e: self.apply_controls())

        def scale(text, var, low, high):
            ttk.Label(f, text=text).pack(anchor="w")
            row = ttk.Frame(f)
            row.pack(anchor="w", fill="x")
            shown = ttk.Label(row, text=f"{var.get():.2f}", width=6)

            def moved(value):
                shown.config(text=f"{float(value):.2f}")
                self.apply_controls()

            ttk.Scale(row, from_=low, to=high, variable=var, command=moved, length=170).pack(side="left")
            shown.pack(side="left")

        section("Operator")
        combo("ACU mission", self.mission, missions)
        check("ASMS on", self.asms)
        ttk.Button(f, text="Press IGN button (300 ms)", command=lambda: self.world.press_ign()).pack(anchor="w", pady=2)
        section("Shutdown circuit")
        check("SDC chain closed (RES, TSMS, ...)", self.sdc_chain)
        section("Jetson")
        check("alive (sends frames)", self.jetson_alive)
        combo("as_state", self.jetson_state, as_states)
        combo("as_mission", self.jetson_mission, missions)
        section("VCU")
        check("alive (sends frames)", self.vcu_alive)
        check("FAULT: keeps HV on when ign drops", self.vcu_stuck)
        section("Other nodes")
        check("RES alive", self.res_alive)
        check("steering (CubeMars) alive", self.steering_alive)
        check("brake pressure sensors alive", self.pressures_alive)
        section("EBS")
        scale("front tank [bar]", self.pneu_front, 0, 10)
        scale("rear tank [bar]", self.pneu_rear, 0, 10)
        scale("front hyd/pneu ratio", self.ratio_front, 0, 12)
        scale("rear hyd/pneu ratio", self.ratio_rear, 0, 12)

    def _diagram(self):
        self.canvas = tk.Canvas(self, bg="white", highlightthickness=0)
        self.canvas.grid(row=1, column=1, sticky="nsew")
        self.boxes = {}
        self.canvas.bind("<Configure>", lambda e: self.draw_machines())

    def _side(self):
        side = ttk.Frame(self, padding=6)
        side.grid(row=1, column=2, sticky="ns")
        self.signals = tk.Text(side, width=46, height=28, font=("Consolas", 10), relief="flat", bg="#f4f4f4")
        self.signals.pack(fill="x")
        for tag, color in (("bad", "#c00000"), ("good", "#007a00"), ("head", "#000080")):
            self.signals.tag_configure(tag, foreground=color)
        notebook = ttk.Notebook(side)
        notebook.pack(fill="both", expand=True, pady=(6, 0))
        self.log = tk.Text(notebook, width=46, font=("Consolas", 9))
        self.tests = tk.Text(notebook, width=46, font=("Consolas", 9))
        phone = ttk.Frame(notebook)
        self.ble = tk.Text(phone, width=46, font=("Consolas", 9), bg="#101010", fg="#d0ffd0")
        self.ble.pack(fill="both", expand=True)
        command_row = ttk.Frame(phone)
        command_row.pack(fill="x")
        self.ble_command = tk.StringVar(value="stop")
        ttk.Entry(command_row, textvariable=self.ble_command, width=20).pack(side="left")
        ttk.Button(command_row, text="Send from phone",
                   command=lambda: self.world.acu.ble_send(self.ble_command.get() + "\r\n")).pack(side="left")
        for widget in (self.log, self.tests):
            widget.tag_configure("bad", foreground="#c00000")
            widget.tag_configure("good", foreground="#007a00")
        notebook.add(self.log, text="Transitions")
        notebook.add(phone, text="Bluetooth (phone)")
        notebook.add(self.tests, text="Unit tests")

    def draw_machines(self):
        c = self.canvas
        c.delete("all")
        self.boxes.clear()
        width = max(c.winfo_width(), 600)
        y = 10
        for enum, field, title, per_row, arrows in MACHINES:
            c.create_text(10, y, text=title, anchor="nw", font=("Segoe UI", 11, "bold"), fill="#000080")
            y += 24
            names = [ENUMS[enum][k] for k in sorted(ENUMS[enum])]
            box_w = (width - 20) / per_row
            for i, name in enumerate(names):
                row, col = divmod(i, per_row)
                x0 = 10 + col * box_w
                y0 = y + row * 120
                rect = c.create_rectangle(x0 + 4, y0, x0 + box_w - 8, y0 + 34, fill="#eeeeee", outline="#888888", width=2)
                c.create_text(x0 + box_w / 2 - 2, y0 + 17, text=name, font=("Segoe UI", 9, "bold"))
                c.create_text(x0 + 8, y0 + 40, text=EXITS[enum].get(name, ""), anchor="nw",
                              font=("Segoe UI", 8), fill="#444444")
                if arrows and col and name not in DANGER:
                    c.create_line(x0 - 8, y0 + 17, x0 + 4, y0 + 17, arrow="last", fill="#888888")
                self.boxes[(enum, name)] = rect
            y += ((len(names) - 1) // per_row + 1) * 120 + 6
        self.refresh()

    # -- simulation ------------------------------------------------------------

    def reset(self):
        if self.world:
            self.world.close()
        self.world = World(Plant())
        self.script = None
        self.events_shown = 0
        self.log.delete("1.0", "end")
        self.ble.delete("1.0", "end")
        self.apply_controls()
        self.note("power on")
        self.refresh()

    def set_mission(self, mission):
        self.mission.set(mission)
        self.jetson_mission.set(mission)

    def sync_controls(self):
        """Script changed the plant: show it in the controls."""
        self.jetson_state.set(self.world.plant.jetson_state)
        self.apply_controls()

    def apply_controls(self):
        w, p = self.world, self.world.plant
        w.set_asms(self.asms.get())
        w.set_mission(self.mission.get())
        p.sdc_chain_closed = self.sdc_chain.get()
        p.jetson_alive = self.jetson_alive.get()
        p.jetson_state = self.jetson_state.get()
        p.jetson_mission = self.jetson_mission.get()
        p.vcu_alive = self.vcu_alive.get()
        p.vcu_ignores_hv_off = self.vcu_stuck.get()
        p.res_alive = self.res_alive.get()
        p.steering_alive = self.steering_alive.get()
        p.pressures_alive = self.pressures_alive.get()
        p.front_pneumatic = round(self.pneu_front.get(), 2)
        p.rear_pneumatic = round(self.pneu_rear.get(), 2)
        p.front_ratio = round(self.ratio_front.get(), 2)
        p.rear_ratio = round(self.ratio_rear.get(), 2)

    def toggle_run(self):
        self.running = not self.running
        self.run_button.config(text="Pause" if self.running else "Run")

    def start_script(self):
        self.reset()
        self.script = inspection_script(self)
        if not self.running:
            self.toggle_run()

    def advance(self, ms):
        for _ in range(int(ms)):
            self.world.step()
            if self.script is not None:
                try:
                    next(self.script)
                except StopIteration:
                    self.script = None
        self.refresh()

    def tick(self):
        if self.running:
            self.advance(TICK_MS * float(self.speed.get()[1:]))
        self.poll_tests()
        self.after(TICK_MS, self.tick)

    # -- display ---------------------------------------------------------------

    def note(self, text):
        self.log.insert("end", f"{self.world.now:8d} ms  {text}\n", "good")
        self.log.see("end")

    def refresh(self):
        w = self.world
        if w is None:
            return
        s = w.state
        self.clock.config(text=f"t = {s.time_ms / 1000:8.3f} s")

        for enum, field, _, _, _ in MACHINES:
            current = getattr(s, field)
            for name in ENUMS[enum].values():
                rect = self.boxes.get((enum, name))
                if rect is None:
                    continue
                if name == current:
                    color = "#ff6b6b" if name in DANGER else "#7cd67c"
                    self.canvas.itemconfig(rect, fill=color, outline="#000000")
                else:
                    self.canvas.itemconfig(rect, fill="#eeeeee", outline="#888888")

        for t, field, old, new in w.events[self.events_shown:]:
            tag = "bad" if new in DANGER else ""
            self.log.insert("end", f"{t:8d} ms  {field}: {old} -> {new}\n", tag)
        if len(w.events) > self.events_shown:
            self.log.see("end")
        self.events_shown = len(w.events)

        if w.ble_text:
            self.ble.insert("end", w.ble_text.replace("\r", ""))
            self.ble.see("end")
            w.ble_text = ""

        now = s.time_ms
        rows = [
            ("HIGH VOLTAGE", "ON" if w.hv_on else "off", "bad" if w.hv_on else "good"),
            ("ACU ign request (to VCU)", s.ignition_request, "bad" if s.ignition_request else ""),
            ("Ignition_enable", s.ignition_enable, ""),
            ("VCU ignition_auto", s.ignition_status, ""),
            ("SDC", "closed" if w.sdc_closed else "OPEN", "good" if w.sdc_closed else "bad"),
            ("HW_WDT_Enable / pulsing", f"{s.hw_wdt_enable} / {'yes' if w.watchdog_pulsing else 'NO'}", ""),
            ("EBS front / rear", f"{'released' if s.front_solenoid else 'ENGAGED'} / "
                                 f"{'released' if s.rear_solenoid else 'ENGAGED'}", ""),
            ("ASSI yellow / blue", f"{w.acu.pin('ASSI_YELLOW')} / {w.acu.pin('ASSI_BLUE')}", ""),
            ("", "", ""),
            ("ACU_STATE", s.acu_state, "head"),
            ("Emergency_cause", s.emergency_cause, "bad" if s.emergency_cause != "NONE" else ""),
            ("mission ACU / Jetson", f"{s.current_mission} / {s.jetson_mission}",
             "bad" if s.current_mission != s.jetson_mission else ""),
            ("seq_fail", f"{s.seq_fail_step} x{s.seq_fail_count}" if s.seq_fail_count else "-", ""),
            ("RES NMT start sent", s.res_nmt_starts, ""),
            ("", "", ""),
            ("front pneu / hyd [bar]", f"{s.front_pneumatic:5.2f} / {s.front_hydraulic:6.1f}", ""),
            ("rear  pneu / hyd [bar]", f"{s.rear_pneumatic:5.2f} / {s.rear_hydraulic:6.1f}", ""),
            ("", "", ""),
            ("age VCU / Jetson [ms]", f"{now - s.vcu_last} / {now - s.jetson_last}", ""),
            ("age RES / AQT7 [ms]", f"{now - s.res_last} / {now - s.rear_pressure_last}", ""),
            ("age steering [ms]", f"{now - s.dir_last}", ""),
            ("rpm (not decoded!)", s.rpm, ""),
        ]
        self.signals.delete("1.0", "end")
        for label, value, tag in rows:
            self.signals.insert("end", f"{label:<27}" if label else "\n")
            if label:
                self.signals.insert("end", f"{value}\n", tag)

    # -- unit tests ------------------------------------------------------------

    def run_tests(self):
        self.tests.delete("1.0", "end")
        self.tests.insert("end", "running...\n")

        def worker():
            result = subprocess.run([sys.executable, "-m", "unittest", "-v", "test_acu"], cwd=HERE,
                                    capture_output=True, text=True)
            self.test_queue.put(result.stderr + result.stdout)

        threading.Thread(target=worker, daemon=True).start()

    def poll_tests(self):
        try:
            output = self.test_queue.get_nowait()
        except queue.Empty:
            return
        self.tests.delete("1.0", "end")
        for line in output.splitlines():
            tag = "bad" if ("FAIL" in line or "ERROR" in line) else "good" if line.endswith("ok") or line == "OK" else ""
            self.tests.insert("end", line + "\n", tag)


if __name__ == "__main__":
    App().mainloop()
