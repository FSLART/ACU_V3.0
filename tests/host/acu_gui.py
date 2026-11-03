"""Tkinter interface for the ACU simulator: the real firmware logic running on the PC.

    py acu_gui.py          (from tests/host; needs `py -m pip install ziglang` once)

Left: the car around the ACU (operator, Jetson, VCU, SDC, sensors, faults to inject).
Middle: the state machines with the active state highlighted, what moves each state on,
        and the inputs of the AS status flowchart (T 14.8.3).
Right: live signals, transition log, Bluetooth text and the unit tests.
"""
import queue
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk

from acu_sim import CONFIG, ENUMS, World, as_value, mission_value
from build import HERE

C = CONFIG

# What makes each state move on, from acu_config.h.
EXITS = {
    "acu_mode_t": {
        "ACU_MODE_STARTUP": "-> IDLE at the first cycle",
        "ACU_MODE_IDLE": "ASMS on + autonomous mission + IGN released -> AUTONOMOUS",
        "ACU_MODE_AUTONOMOUS": "fault -> EMERGENCY\nASMS off in checkup / Finished over -> IDLE",
        "ACU_MODE_EMERGENCY": f"ASMS off + brakes released {C['BRAKE_RELEASE_CONFIRM_MS']} ms -> IDLE",
    },
    "checkup_step_t": {
        "CHECKUP_TANK_PRESSURE": f"front {C['EBS_FRONT_TANK_MIN_BAR']}-{C['EBS_TANK_MAX_BAR']} bar, "
                                 f"rear {C['EBS_REAR_TANK_MIN_BAR']}-{C['EBS_TANK_MAX_BAR']} bar",
        "CHECKUP_BRAKES_ENGAGED": f"hyd >= {C['EBS_FRONT_HYD_GAIN']}/{C['EBS_REAR_HYD_GAIN']} x tank, "
                                  f"{C['CHECKUP_BRAKES_ENGAGED_TIMEOUT_MS']} ms",
        "CHECKUP_SDC_CLOSE": f"pulses on, SDC closed within {C['CHECKUP_SDC_CLOSE_TIMEOUT_MS']} ms",
        "CHECKUP_WATCHDOG_OPENS_SDC": f"pulses off, SDC open within {C['CHECKUP_SDC_OPEN_TIMEOUT_MS']} ms",
        "CHECKUP_SDC_RECLOSE": f"pulses on, SDC closed within {C['CHECKUP_SDC_CLOSE_TIMEOUT_MS']} ms",
        "CHECKUP_TS_ACTIVATION": "IGN button -> VCU reports TS active",
        "CHECKUP_FRONT_BRAKE": f"front only, {C['CHECKUP_BRAKE_SETTLE_MS']}-{C['CHECKUP_BRAKE_TIMEOUT_MS']} ms",
        "CHECKUP_REAR_BRAKE": f"rear only, {C['CHECKUP_BRAKE_SETTLE_MS']}-{C['CHECKUP_BRAKE_TIMEOUT_MS']} ms",
        "CHECKUP_BOTH_BRAKES": f"both, within {C['CHECKUP_BRAKE_TIMEOUT_MS']} ms",
    },
    "AS_STATE_t": {
        "AS_STATE_READY": f"Jetson DRIVING after {C['READY_MIN_TIME_BEFORE_R2D_MS']} ms -> DRIVING",
        "AS_STATE_DRIVING": f"no movement for {C['DRIVING_NO_MOVE_TIME_MS']} ms\nJetson FINISHED + standstill",
        "AS_STATE_EMERGENCY": "ASMS off + brakes released",
        "AS_STATE_FINISHED": "ASMS off + brakes released",
    },
    "finish_step_t": {
        "FINISH_WAITING_FOR_STANDSTILL": f"rpm <= {C['STANDSTILL_MAX_RPM']} within "
                                         f"{C['FINISH_STANDSTILL_TIMEOUT_MS']} ms",
        "FINISH_EBS_ACTIVATED": f"TS off within {C['FINISH_TS_OFF_TIMEOUT_MS']} ms",
    },
}
MACHINES = [("ACU mode", "acu_mode", "acu_mode_t"), ("Initial checkup (T 15.3.1)", "checkup_step", "checkup_step_t"),
            ("AS state (T 14.8.3)", "as_state", "AS_STATE_t"), ("End of mission", "finish_step", "finish_step_t")]
FLOWCHART = ["ebs_activated", "mission_finished", "standstill", "asms_on", "asb_checks_ok", "ts_active", "r2d",
             "brakes_engaged"]


def short(name):
    for prefix in ("ACU_MODE_", "CHECKUP_", "AS_STATE_", "FINISH_", "MISSION_", "FAULT_"):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ACU TEK-26e simulator")
        self.geometry("1500x900")
        self.world = None
        self.running = False
        self.speed = tk.IntVar(value=1)
        self.test_output = queue.Queue()
        self._toolbar()
        body = ttk.Frame(self)
        body.pack(fill="both", expand=True)
        self._controls(body)
        self._diagram(body)
        self._side(body)
        self.reset()
        self.after(50, self.tick)

    # -- layout -------------------------------------------------------------------
    def _toolbar(self):
        bar = ttk.Frame(self, padding=4)
        bar.pack(fill="x")
        ttk.Button(bar, text="Power cycle", command=self.reset).pack(side="left")
        self.run_button = ttk.Button(bar, text="Run", command=self.toggle_run)
        self.run_button.pack(side="left", padx=4)
        ttk.Button(bar, text="+100 ms", command=lambda: self.advance(100)).pack(side="left")
        ttk.Button(bar, text="+1 s", command=lambda: self.advance(1000)).pack(side="left", padx=4)
        ttk.Label(bar, text="speed x").pack(side="left", padx=(12, 0))
        ttk.Combobox(bar, textvariable=self.speed, values=[1, 5, 20], width=4, state="readonly").pack(side="left")
        ttk.Button(bar, text="Script: full mission", command=self.start_script).pack(side="left", padx=12)
        ttk.Button(bar, text="Run unit tests", command=self.run_tests).pack(side="left")
        self.time_label = ttk.Label(bar, text="")
        self.time_label.pack(side="right")

    def _controls(self, parent):
        f = ttk.Frame(parent, padding=6)
        f.pack(side="left", fill="y")
        self.vars = {}

        def section(text):
            ttk.Label(f, text=text, font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(8, 2))

        def check(text, key, on_change):
            var = self.vars[key] = tk.BooleanVar()
            ttk.Checkbutton(f, text=text, variable=var, command=lambda: on_change(var.get())).pack(anchor="w")

        def combo(text, key, values, on_change):
            row = ttk.Frame(f)
            row.pack(anchor="w", fill="x")
            ttk.Label(row, text=text, width=12).pack(side="left")
            var = self.vars[key] = tk.StringVar()
            box = ttk.Combobox(row, textvariable=var, values=values, width=16, state="readonly")
            box.bind("<<ComboboxSelected>>", lambda e: on_change(var.get()))
            box.pack(side="left")

        def scale(text, key, low, high, on_change):
            row = ttk.Frame(f)
            row.pack(anchor="w", fill="x")
            ttk.Label(row, text=text, width=12).pack(side="left")
            var = self.vars[key] = tk.DoubleVar()
            value = ttk.Label(row, width=6)
            value.pack(side="right")

            def moved(_):
                value.config(text=f"{var.get():.1f}")
                on_change(var.get())
            ttk.Scale(row, from_=low, to=high, variable=var, command=moved, length=140).pack(side="left")

        def plant(field):
            return lambda value: setattr(self.world.plant, field, type(getattr(self.world.plant, field))(value))

        missions = [short(n) for _, n in sorted(ENUMS["mission_t"].items())]
        as_states = [short(n) for _, n in sorted(ENUMS["AS_STATE_t"].items())]

        section("Operator")
        check("ASMS on", "asms", lambda on: self.world.set_asms(on))
        ttk.Button(f, text="IGN button (300 ms)", command=lambda: self.world.press_ign()).pack(anchor="w", pady=2)
        ttk.Button(f, text="Mission button", command=lambda: self.world.acu.lib.sim_press_mission_button()
                   ).pack(anchor="w")
        combo("mission (both)", "mission", missions, lambda m: self.world.set_mission(m))
        check("brakes released by hand", "brakes_released_by_hand", plant("brakes_released_by_hand"))

        section("Jetson")
        check("alive", "jetson_alive", plant("jetson_alive"))
        combo("as_state", "jetson_as_state", as_states, lambda s: setattr(self.world.plant, "jetson_as_state",
                                                                         as_value(s)))
        combo("as_mission", "jetson_mission", missions, lambda m: setattr(self.world.plant, "jetson_mission",
                                                                         mission_value(m)))

        section("Car")
        scale("tank front", "front_tank_bar", 0, 12, plant("front_tank_bar"))
        scale("tank rear", "rear_tank_bar", 0, 12, plant("rear_tank_bar"))
        scale("hyd/tank front", "front_ratio", 0, 12, plant("front_ratio"))
        scale("hyd/tank rear", "rear_ratio", 0, 12, plant("rear_ratio"))
        scale("motor rpm", "motor_rpm", 0, 2000, plant("motor_rpm"))
        for field in ("sdc_chain_closed", "vcu_alive", "vcu_rpm_alive", "res_alive", "front_brake_alive",
                      "rear_brake_alive", "steering_alive"):
            check(field.replace("_", " "), field, plant(field))

        section("Faults")
        for field in ("watchdog_stuck_closed", "ts_stuck_on", "ts_forced_off"):
            check(field.replace("_", " "), field, plant(field))

    def _diagram(self, parent):
        self.canvas = tk.Canvas(parent, width=640, background="white")
        self.canvas.pack(side="left", fill="both", expand=True)

    def _side(self, parent):
        tabs = ttk.Notebook(parent, width=420)
        tabs.pack(side="left", fill="both")
        self.signals = ttk.Treeview(tabs, columns=("value",), show="tree headings", height=40)
        self.signals.heading("#0", text="signal")
        self.signals.heading("value", text="value")
        self.signals.column("#0", width=210)
        self.signals.tag_configure("bad", foreground="red")
        tabs.add(self.signals, text="Signals")
        self.log_text = tk.Text(tabs, font=("Consolas", 9))
        tabs.add(self.log_text, text="Transitions")
        self.ble_view = tk.Text(tabs, font=("Consolas", 9))
        tabs.add(self.ble_view, text="Bluetooth")
        self.tests_text = tk.Text(tabs, font=("Consolas", 9))
        tabs.add(self.tests_text, text="Unit tests")

    # -- simulation --------------------------------------------------------------------
    def reset(self):
        if self.world:
            self.world.close()
        self.world = World()
        self.script = None
        self._log_len = 0
        self.log_text.delete("1.0", "end")
        self.ble_view.delete("1.0", "end")
        p = self.world.plant
        for key, var in self.vars.items():
            if key == "asms":
                var.set(False)
            elif key == "mission":
                var.set("MANUAL")
            elif key == "jetson_as_state":
                var.set("OFF")
            elif key == "jetson_mission":
                var.set("MANUAL")
            else:
                var.set(getattr(p, key))
        self.refresh()

    def toggle_run(self):
        self.running = not self.running
        self.run_button.config(text="Pause" if self.running else "Run")

    def advance(self, ms):
        for _ in range(int(ms)):
            if self.script:
                try:
                    next(self.script)
                except StopIteration:
                    self.script = None
            self.world.step()
        self.refresh()

    def tick(self):
        if self.running or self.script:
            self.advance(50 * self.speed.get())
        while not self.test_output.empty():
            self.tests_text.insert("end", self.test_output.get())
            self.tests_text.see("end")
        self.after(50, self.tick)

    def start_script(self):
        """Full mission: checkup, Ready, R2D after 5 s, drive, finish, ASMS off, release brakes."""
        def script():
            w = self.world
            w.set_mission("INSPECTION")
            self.vars["mission"].set("INSPECTION")
            yield from (None for _ in range(500))
            w.set_asms(True)
            self.vars["asms"].set(True)
            while w.state.checkup_step != "CHECKUP_TS_ACTIVATION":
                if w.state.fault_active:
                    return
                yield
            w.press_ign()
            while w.state.as_state != "AS_STATE_READY":
                if w.state.fault_active:
                    return
                yield
            yield from (None for _ in range(5500))
            w.jetson("DRIVING")
            yield from (None for _ in range(3500))
            w.plant.motor_rpm = 800
            yield from (None for _ in range(5000))
            w.plant.motor_rpm = 0
            yield from (None for _ in range(1000))
            w.jetson("FINISHED")
            yield from (None for _ in range(3000))
            w.set_asms(False)
            self.vars["asms"].set(False)
            yield from (None for _ in range(1000))
            w.plant.brakes_released_by_hand = 1
            self.vars["brakes_released_by_hand"].set(True)
        self.script = script()

    def note(self, text):
        self.log_text.insert("end", f"{self.world.now:7d} ms  {text}\n")

    # -- drawing ---------------------------------------------------------------------------
    def draw_machines(self, s):
        c = self.canvas
        c.delete("all")
        y = 10
        for title, field, enum in MACHINES:
            c.create_text(10, y, text=title, anchor="nw", font=("Segoe UI", 11, "bold"))
            y += 22
            names = [n for _, n in sorted(ENUMS[enum].items())]
            current = getattr(s, "mode" if field == "acu_mode" else field)
            x = 10
            for name in names:
                width = 150
                if x + width > 630:
                    x, y = 10, y + 64
                active = name == current
                c.create_rectangle(x, y, x + width - 6, y + 58, fill="#ffd54f" if active else "#eeeeee",
                                   outline="#444" if active else "#bbb", width=2 if active else 1)
                c.create_text(x + 6, y + 4, text=short(name), anchor="nw", font=("Segoe UI", 9, "bold"))
                c.create_text(x + 6, y + 20, text=EXITS.get(enum, {}).get(name, ""), anchor="nw",
                              font=("Segoe UI", 7), width=width - 16)
                x += width
            y += 74
        c.create_text(10, y, text="Flowchart inputs (Figure 16)", anchor="nw", font=("Segoe UI", 11, "bold"))
        y += 22
        for i, name in enumerate(FLOWCHART):
            on = bool(getattr(s, name))
            c.create_text(10 + (i % 4) * 155, y + (i // 4) * 18, anchor="nw", text=("[x] " if on else "[ ] ") + name,
                          fill="#1b5e20" if on else "#777", font=("Consolas", 9))

    def refresh(self):
        w = self.world
        s = w.state
        self.time_label.config(text=f"t = {w.now / 1000:.3f} s")
        self.draw_machines(s)
        rows = [
            ("mode / AS", f"{short(s.mode)} / {short(s.as_state)}", ""),
            ("fault", f"{short(s.fault_reason)} (last {short(s.fault_last_reason)}, {s.fault_count}x)",
             "bad" if s.fault_active else ""),
            ("checkup / finish", f"{short(s.checkup_step)} / {short(s.finish_step)}", ""),
            ("mission ACU / Jetson", f"{short(s.mission)} / {short(s.jetson_mission)}",
             "bad" if s.mission != s.jetson_mission else ""),
            ("Jetson asks", short(s.jetson_as_state), ""),
            ("ASMS / SDC / TS", f"{s.asms_on} / {s.sdc_closed} / {s.ts_active}", ""),
            ("watchdog pulses", s.watchdog_pulses_on, ""),
            ("EBS released front / rear", f"{s.front_ebs_released} / {s.rear_ebs_released}", ""),
            ("TS request / enable", f"{s.ignition_request} / {s.ignition_enable}", ""),
            ("ASSI yellow / blue", f"{s.assi_yellow} / {s.assi_blue}", ""),
            ("tank front / rear [bar]", f"{s.front_tank_bar:.2f} / {s.rear_tank_bar:.2f}", ""),
            ("brake front / rear [bar]", f"{s.front_brake_bar:.1f} / {s.rear_brake_bar:.1f}", ""),
            ("speed known / moving", f"{s.speed_known} / {s.moving}", ""),
            ("age VCU / Jetson / RES [ms]", f"{s.age_vcu} / {s.age_jetson} / {s.age_res}", ""),
            ("monitoring active", s.monitor_active, ""),
            ("EBS state (DV_status)", short(s.ebs_state), ""),
            ("checkups passed / failed", f"{s.checkup_passed_count} / {s.checkup_failed_count}", ""),
            ("log records / written", f"{s.log_stored} / {s.log_written}", ""),
        ]
        self.signals.delete(*self.signals.get_children())
        for name, value, tag in rows:
            self.signals.insert("", "end", text=name, values=(str(value),), tags=(tag,))
        for t, f, a, b in w.events[self._log_len:]:
            self.log_text.insert("end", f"{t:7d} ms  {f}: {short(str(a))} -> {short(str(b))}\n")
        self._log_len = len(w.events)
        self.log_text.see("end")
        if w.ble_text:
            self.ble_view.insert("end", w.ble_text)
            self.ble_view.see("end")
            w.ble_text = ""

    # -- unit tests -----------------------------------------------------------------------------
    def run_tests(self):
        self.tests_text.delete("1.0", "end")

        def worker():
            process = subprocess.Popen([sys.executable, "-m", "unittest", "-v", "test_acu"], cwd=HERE,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in process.stdout:
                self.test_output.put(line)
        threading.Thread(target=worker, daemon=True).start()


if __name__ == "__main__":
    App().mainloop()
