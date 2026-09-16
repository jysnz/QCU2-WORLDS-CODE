#!/usr/bin/env python3
"""
PID Tuner -- the laptop half of the brain's PID TUNING screen.

Opened automatically by tools/remote_touch.py when the brain enters that
screen (HOME -> PID TUNING), the same way the path planner is. It talks
to the robot through remote_touch's link and mirrors the tuner's state
from the same RUI| lines the mirror draws; the run traces arrive as the
"PID|..." / "CSV,..." lines the tuner prints while a test is running
(see src/pid_tuner.cpp, "FROM A LAPTOP").

WHAT'S ON IT
    Left      one card per controller -- ANGULAR (turns) and LATERAL
              (drive) -- with kP / kI / kD each in a text box: type a
              number and press Enter (or Set) and the brain takes it
              straight away. The - / + buttons nudge a gain by the step
              picked underneath (100 ... 0.001), which is the same digit
              cursor the controller's L1/L2 move, so the brain screen
              follows along. Under each card are that controller's
              tests: the small one (90 deg / 24 in), the big one (180
              deg / 48 in), the return, and the 4-leg sweep -- the same
              runs the controller's A / X / B / R1 / R2 start.
    Right     the graph: the setpoint (pink) and the actual heading or
              position chasing it (green), against time. For a sweep,
              all four legs sit side by side, divided by grey lines and
              labelled with each leg's target -- the brain's graph, but
              bigger, and it stays put until the next run. Under it,
              every leg's overshoot / settle time / final error.
    STOP      (or Esc) cancels the run in progress.
    The physical controller keeps working the whole time: what it
    changes shows up here, what's changed here shows up on the brain.

SAVING THE RESULT
    Gains only live in the brain's RAM until they're written into
    src/main.cpp, so:
    Copy as C++       the two lemlib::ControllerSettings lines with the
                      current gains (the other parameters -- windup,
                      exit ranges, slew -- are kept as main.cpp has them),
                      to the clipboard and a window.
    Write to main.cpp rewrites those two lines in src/main.cpp in place,
                      after asking.
    Save / Load       the gains as JSON (tools/gains/*.json), to keep a
                      set around or compare two.
    Save run CSV      the last run's samples (ms, target, actual, error),
                      for plotting elsewhere.

RUN ON ITS OWN
    python tools/pid_tuner.py    opens the window against a fake robot
    that plays back a made-up sweep, to check the layout without a
    brain plugged in. Nothing is sent anywhere.
"""

import json
import math
import os
import re
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN_CPP = os.path.join(HERE, "..", "src", "main.cpp")
GAINS_DIR = os.path.join(HERE, "gains")

# Palette -- the brain's ui:: namespace (pid_tuner.cpp) / the planner's.
BG = "#14141a"
CARD = "#161B22"
CARD_SEL = "#1c2838"
CYAN = "#00F0FF"
ORANGE = "#FF8C00"
GREEN = "#2ECC71"
RED = "#FF3131"
GRAY = "#777777"
DIM = "#3a3a46"
GRID = "#1e1e26"
AXIS = "#2d2d38"
TARGET_LINE = "#FF6B81"
ACTUAL_LINE = "#2EFF8C"
WHITE = "#FFFFFF"

GAIN_NAMES = ("kP", "kI", "kD")
MODES = ("angular", "lateral")           # index == the brain's mode number
MODE_TITLES = ("ANGULAR (turns)", "LATERAL (drive)")
MODE_COLORS = (CYAN, ORANGE)
# Test buttons per mode: (label, test index) -- matches angularTests /
# lateralTests in pid_tuner.cpp (A / X / B).
TESTS = ((("Turn 90", 0), ("Turn 180", 1), ("Back to 0", 2)),
         (("Drive 24in", 0), ("Drive 48in", 1), ("Back 24in", 2)))
UNITS = ("deg", "in")
DIGIT_STEPS = (2, 1, 0, -1, -2, -3)       # digitExp values L1/L2 walk through

GRAPH_W, GRAPH_H = 640, 340
GRAPH_PAD_L, GRAPH_PAD_R, GRAPH_PAD_T, GRAPH_PAD_B = 46, 12, 22, 22


def fmt_gain(v):
    """Gains as the brain shows them (3 decimals), trailing zeros kept so
    the boxes don't jump around as values change."""
    return f"{v:.3f}"


def nice_step(rough):
    """Rounds a rough grid spacing up to a 1/2/5 x 10^n step (niceStep()
    on the brain)."""
    if rough <= 0:
        return 1.0
    mag = 10 ** math.floor(math.log10(rough))
    norm = rough / mag
    return (1 if norm < 1.5 else 2 if norm < 3 else 5 if norm < 7 else 10) * mag


def compute_scale(lo, hi, divisions=6):
    """[minVal, maxVal, step] covering lo..hi with headroom, snapped to nice
    grid lines (computeScale() on the brain, with more divisions since
    there's more room)."""
    if hi < lo:
        lo, hi = hi, lo
    span = max(1.0, hi - lo)
    pad = span * 0.15
    step = nice_step((span + 2 * pad) / divisions)
    minVal = math.floor((lo - pad) / step) * step
    maxVal = math.ceil((hi + pad) / step) * step
    if maxVal <= minVal:
        maxVal = minVal + step
    return minVal, maxVal, step


def read_controller_lines():
    """The two lemlib::ControllerSettings lines in src/main.cpp, as
    {name: [9 floats]} -- the 6 non-gain parameters are needed to write a
    complete line back out. Missing file / no match -> zeros."""
    out = {"lateral_controller": [0.0] * 9, "angular_controller": [0.0] * 9}
    try:
        src = open(MAIN_CPP, encoding="utf-8").read()
    except OSError:
        return out
    for name in out:
        m = re.search(r"lemlib::ControllerSettings\s+" + name + r"\s*\(([^)]*)\)", src)
        if m:
            try:
                vals = [float(v) for v in m.group(1).split(",")]
            except ValueError:
                continue
            if len(vals) == 9:
                out[name] = vals
    return out


def controller_code(gains):
    """gains = {"angular": [p, i, d], "lateral": [p, i, d]} -> the two C++
    lines, with everything but kP/kI/kD as main.cpp has it."""
    base = read_controller_lines()
    lines = []
    for name, mode in (("lateral_controller", "lateral"), ("angular_controller", "angular")):
        vals = list(base[name])
        vals[0:3] = gains[mode]
        lines.append(f"lemlib::ControllerSettings {name}({', '.join(f'{v:g}' for v in vals)});")
    return "\n".join(lines) + "\n"


class Run:
    """One test or sweep as the brain streams it: the schedule up front,
    then each leg's samples and result."""

    def __init__(self, angular, legs):
        self.angular = angular
        self.legs = legs                 # [{"label", "level", "timeout"}] from PID|RUN
        for leg in self.legs:
            leg.update({"target": leg["level"], "start": None, "samples": [], "result": None})
        self.current = -1
        self.done = False
        self.cancelled = False
        self.startedAt = time.time()

    def total_timeout(self):
        return sum(max(1, leg["timeout"]) for leg in self.legs) or 1


class PidTunerWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.title("PID Tuner -- V5")
        self.configure(bg=BG)
        self.brain = None              # last "pid" dict from the brain
        self.onBrainScreen = False
        self.busy = False
        self.run = None                # Run in progress / last finished
        self.editing = None            # entry widget being typed in
        self.pendingSet = {}           # (mode, gain) -> (value, sent at)
        self.pendingDigit = None       # (digitExp, sent at) until the brain echoes it
        self._redrawPending = False
        self.protocol("WM_DELETE_WINDOW", self.withdraw)
        self._buildUi()
        self.bind("<Escape>", lambda e: self.stop())
        self.redrawGraph()
        self.update_idletasks()
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())

    # -- layout --
    def _buildUi(self):
        top = tk.Frame(self, bg=BG)
        top.pack(side="top", fill="x")
        self.brainStatus = tk.Label(top, text="brain: waiting...", anchor="w", bg=BG,
                                    fg=ORANGE, font=("Consolas", 10))
        self.brainStatus.pack(side="left", fill="x", expand=True, padx=10, pady=(8, 2))
        tk.Label(top, text="Esc = stop", bg=BG, fg=GRAY, font=("Segoe UI", 8)).pack(
            side="right", padx=10)

        body = tk.Frame(self, bg=BG)
        body.pack(side="top", fill="both", expand=True)

        # Left: the two controller cards + step + stop + files ----------
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="y", padx=(10, 6), pady=6)

        self.cards = []
        self.modeTag = []              # "controller editing" tag per card
        self.entries = {}              # (mode, gain) -> Entry
        self.entryVars = {}
        self.rowFrames = {}            # (mode, gain) -> Frame (highlighted when selected)
        self.testButtons = []
        for mode in range(2):
            card = tk.Frame(left, bg=CARD, highlightthickness=2, highlightbackground=AXIS)
            card.pack(fill="x", pady=(0, 8))
            self.cards.append(card)
            head = tk.Frame(card, bg=CARD)
            head.pack(fill="x", padx=8, pady=(6, 2))
            tk.Label(head, text=MODE_TITLES[mode], bg=CARD, fg=MODE_COLORS[mode],
                     font=("Segoe UI", 10, "bold")).pack(side="left")
            tag = tk.Label(head, text="", bg=CARD, fg=GRAY, font=("Segoe UI", 8))
            tag.pack(side="right")
            self.modeTag.append(tag)
            for gain in range(3):
                row = tk.Frame(card, bg=CARD, highlightthickness=1, highlightbackground=CARD)
                row.pack(fill="x", padx=8, pady=1)
                self.rowFrames[(mode, gain)] = row
                tk.Label(row, text=GAIN_NAMES[gain], width=3, anchor="w", bg=CARD, fg=GRAY,
                         font=("Consolas", 11)).pack(side="left", padx=(4, 2))
                var = tk.StringVar(value="0.000")
                ent = tk.Entry(row, textvariable=var, width=9, font=("Consolas", 12),
                               bg="#0f1218", fg=WHITE, insertbackground=WHITE, relief="flat",
                               highlightthickness=1, highlightbackground=AXIS,
                               highlightcolor=CYAN, justify="right")
                ent.pack(side="left", padx=2, ipady=2)
                ent.bind("<Return>", lambda e, m=mode, g=gain: self.setFromEntry(m, g))
                ent.bind("<FocusIn>", lambda e, m=mode, g=gain: self.onEntryFocus(m, g))
                ent.bind("<FocusOut>", lambda e, m=mode, g=gain: self.onEntryBlur(m, g))
                ent.bind("<Up>", lambda e, m=mode, g=gain: self.nudge(m, g, +1))
                ent.bind("<Down>", lambda e, m=mode, g=gain: self.nudge(m, g, -1))
                self.entries[(mode, gain)] = ent
                self.entryVars[(mode, gain)] = var
                self._btn(row, "-", lambda m=mode, g=gain: self.nudge(m, g, -1), width=2)
                self._btn(row, "+", lambda m=mode, g=gain: self.nudge(m, g, +1), width=2)
                self._btn(row, "Set", lambda m=mode, g=gain: self.setFromEntry(m, g), width=4,
                          fg=CYAN)
            tests = tk.Frame(card, bg=CARD)
            tests.pack(fill="x", padx=8, pady=(4, 8))
            for label, idx in TESTS[mode]:
                b = self._btn(tests, label, lambda m=mode, i=idx: self.runTest(m, i), width=9)
                self.testButtons.append(b)
            b = self._btn(tests, "Sweep", lambda m=mode: self.runSweep(m), width=7,
                          fg=MODE_COLORS[mode])
            self.testButtons.append(b)

        stepRow = tk.Frame(left, bg=BG)
        stepRow.pack(fill="x", pady=(0, 6))
        tk.Label(stepRow, text="step (- / +):", bg=BG, fg=GRAY, font=("Segoe UI", 9)).pack(
            side="left")
        self.stepVar = tk.IntVar(value=-2)
        for e in DIGIT_STEPS:
            tk.Radiobutton(stepRow, text=f"{10.0 ** e:g}", variable=self.stepVar, value=e,
                           command=self.onStepChosen, bg=BG, fg=WHITE, selectcolor="#0f1218",
                           activebackground=BG, activeforeground=CYAN,
                           font=("Consolas", 9)).pack(side="left")

        self.stopBtn = tk.Button(left, text="STOP  (Esc)", command=self.stop, bg="#3a1414",
                                 fg=WHITE, activebackground=RED, relief="flat",
                                 font=("Segoe UI", 10, "bold"), cursor="hand2")
        self.stopBtn.pack(fill="x", pady=(0, 8), ipady=4)

        files = tk.Frame(left, bg=BG)
        files.pack(fill="x")
        self._btn(files, "Copy as C++", self.copyCode, width=12, fg=GREEN)
        self._btn(files, "Write to main.cpp", self.writeMainCpp, width=15)
        files2 = tk.Frame(left, bg=BG)
        files2.pack(fill="x", pady=(4, 0))
        self._btn(files2, "Save gains", self.saveGains, width=10)
        self._btn(files2, "Load gains", self.loadGains, width=10)
        self._btn(files2, "Save run CSV", self.saveRunCsv, width=12)

        self.hint = tk.Label(left, text="Type a value + Enter (or Set) to send it.\n"
                                        "Up/Down in a box nudges by the step.\n"
                                        "Keep the robot's surroundings clear before a test.",
                             justify="left", anchor="w", bg=BG, fg=DIM, font=("Segoe UI", 8))
        self.hint.pack(fill="x", pady=(10, 0))

        # Right: graph + leg results -------------------------------------
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(6, 10), pady=6)
        self.graph = tk.Canvas(right, width=GRAPH_W, height=GRAPH_H, bg=CARD,
                               highlightthickness=1, highlightbackground=AXIS)
        self.graph.pack(fill="both", expand=True)
        self.graph.bind("<Configure>", lambda e: self.scheduleRedraw())
        self.runStatus = tk.Label(right, text="no run yet", anchor="w", bg=BG, fg=GRAY,
                                  font=("Consolas", 10))
        self.runStatus.pack(fill="x", pady=(4, 0))
        self.results = tk.Text(right, height=5, bg="#0f1218", fg=WHITE, font=("Consolas", 10),
                               relief="flat", state="disabled", highlightthickness=1,
                               highlightbackground=AXIS)
        self.results.pack(fill="x", pady=(4, 0))
        self.results.tag_config("ok", foreground=CYAN)
        self.results.tag_config("bad", foreground=RED)
        self.results.tag_config("dim", foreground=GRAY)

    def _btn(self, parent, text, cmd, width=None, fg=WHITE):
        b = tk.Button(parent, text=text, command=cmd, width=width, bg="#22222B", fg=fg,
                      activebackground="#2d2d38", activeforeground=fg, relief="flat",
                      font=("Segoe UI", 9), cursor="hand2")
        b.pack(side="left", padx=2, pady=1)
        return b

    # -- sending --
    def send(self, line):
        self.link._send(line)

    def setGain(self, mode, gain, value):
        value = max(0.0, float(value))
        self.send(f"PID SET {mode} {gain} {value:.4f}")
        self.pendingSet[(mode, gain)] = (value, time.time())
        self.entryVars[(mode, gain)].set(fmt_gain(value))

    def setFromEntry(self, mode, gain):
        text = self.entryVars[(mode, gain)].get().strip()
        try:
            value = float(text)
        except ValueError:
            self.flashEntry(mode, gain)
            return
        self.setGain(mode, gain, value)
        self.focus_set()   # leave the box so the brain's echo can land in it

    def nudge(self, mode, gain, direction):
        try:
            current = float(self.entryVars[(mode, gain)].get())
        except ValueError:
            current = self.brainGain(mode, gain)
        step = 10.0 ** self.stepVar.get()
        self.setGain(mode, gain, round(current + direction * step, 4))
        return "break"

    def brainGain(self, mode, gain):
        if not self.brain:
            return 0.0
        return self.brain[MODES[mode]][gain]

    def onEntryFocus(self, mode, gain):
        self.editing = (mode, gain)
        # Point the controller's UP/DOWN at the same gain so the brain
        # screen highlights what's being edited here.
        self.send(f"PID MODE {mode}")
        self.send(f"PID SEL {gain}")

    def onEntryBlur(self, mode, gain):
        if self.editing == (mode, gain):
            self.editing = None
        self.entryVars[(mode, gain)].set(fmt_gain(self.brainGain(mode, gain)))

    def onStepChosen(self):
        self.send(f"PID DIGIT {self.stepVar.get()}")
        self.pendingDigit = (self.stepVar.get(), time.time())

    def flashEntry(self, mode, gain):
        ent = self.entries[(mode, gain)]
        ent.config(highlightbackground=RED, highlightcolor=RED)
        self.after(600, lambda: ent.config(highlightbackground=AXIS, highlightcolor=CYAN))

    def runTest(self, mode, idx):
        if not self.checkCanRun():
            return
        self.send(f"PID TEST {mode} {idx}")
        self.runStatus.config(text=f"{MODES[mode]} test {idx} requested...", fg=GRAY)

    def runSweep(self, mode):
        if not self.checkCanRun():
            return
        self.send(f"PID SWEEP {mode}")
        self.runStatus.config(text=f"{MODES[mode]} sweep requested...", fg=GRAY)

    def checkCanRun(self):
        if not self.onBrainScreen:
            self.runStatus.config(text="the brain isn't on the PID TUNER screen", fg=ORANGE)
            return False
        if self.busy:
            self.runStatus.config(text="a run is already in progress -- STOP it first",
                                  fg=ORANGE)
            return False
        return True

    def stop(self):
        self.send("PID STOP")
        if self.busy:
            self.runStatus.config(text="stop sent", fg=ORANGE)

    # -- state from the brain (every RUI line) --
    def applyState(self, state):
        self.onBrainScreen = state["screen"] == "PID_TUNER"
        pid = state.get("pid")
        if self.onBrainScreen and pid:
            self.brain = pid
            self.busy = pid["busy"]
            now = time.time()
            for mode in range(2):
                for gain in range(3):
                    key = (mode, gain)
                    v = pid[MODES[mode]][gain]
                    pend = self.pendingSet.get(key)
                    if pend and (abs(pend[0] - v) < 5e-4 or now - pend[1] > 1.5):
                        del self.pendingSet[key]
                        pend = None
                    # Don't type over the person: a box being edited, or one
                    # whose new value hasn't echoed back yet, keeps its text.
                    if self.editing != key and not pend:
                        self.entryVars[key].set(fmt_gain(v))
            pend = self.pendingDigit
            if pend and (pend[0] == pid["digitExp"] or now - pend[1] > 1.5):
                self.pendingDigit = pend = None
            if not pend:
                self.stepVar.set(pid["digitExp"])
            for mode in range(2):
                on = pid["mode"] == mode
                self.cards[mode].config(highlightbackground=MODE_COLORS[mode] if on else AXIS)
                self.modeTag[mode].config(text="controller editing" if on else "")
                for gain in range(3):
                    sel = on and pid["sel"] == gain
                    self.rowFrames[(mode, gain)].config(
                        highlightbackground=ORANGE if sel else CARD,
                        bg=CARD_SEL if sel else CARD)
                    for w in self.rowFrames[(mode, gain)].winfo_children():
                        if isinstance(w, tk.Label):
                            w.config(bg=CARD_SEL if sel else CARD)
            self.brainStatus.config(
                text=f"brain: PID TUNER  {MODES[pid['mode']].upper()}  "
                     f"{'RUNNING' if self.busy else 'idle'}  step {10.0 ** pid['digitExp']:g}",
                fg=GREEN if self.busy else CYAN)
            r = state.get("result")
            if r and not self.busy and self.run is None:
                self.runStatus.config(
                    text=f"last on brain: {r[0]}  overshoot {r[1]:.2f}  settle {r[2]} ms  "
                         f"final {r[3]:.2f}  ({r[4]} ms)", fg=CYAN if r[2] >= 0 else RED)
        else:
            self.busy = False
            self.brainStatus.config(text=f"brain: on {state['screen']} -- tap PID TUNING on "
                                    "the brain (or mirror) to tune from here", fg=ORANGE)
        for b in self.testButtons:
            b.config(state="normal" if (self.onBrainScreen and not self.busy) else "disabled")

    # -- the run stream (PID|... and CSV,... lines) --
    def feedLine(self, line):
        if line.startswith("CSV,"):
            f = line.split(",")
            if len(f) != 5 or self.run is None or self.run.current < 0:
                return    # header line, or a sample with no run to attach to
            try:
                t, target, actual, error = (float(x) for x in f[1:])
            except ValueError:
                return
            leg = self.run.legs[self.run.current]
            leg["target"] = target
            # target - error rather than the raw pose: on angular runs it's
            # wrap-corrected, so the trace doesn't jump across +-180.
            leg["samples"].append((t, target, target - error, error))
            self.scheduleRedraw()
            return
        parts = line.split("|")
        if len(parts) < 2 or parts[0] != "PID":
            return
        kind = parts[1]
        try:
            if kind == "RUN" and len(parts) >= 5:
                legs = []
                for chunk in parts[4].split(";"):
                    f = chunk.split(",")
                    if len(f) == 3:
                        legs.append({"label": f[0], "level": float(f[1]),
                                     "timeout": int(f[2])})
                if legs:
                    self.run = Run(parts[2] == "1", legs)
                    self.showResults()
                    self.runStatus.config(text=f"running {'angular' if self.run.angular else 'lateral'}"
                                          f" -- {' > '.join(l['label'] for l in legs)}",
                                          fg=GREEN)
            elif kind == "LEG" and len(parts) >= 7 and self.run is not None:
                i = int(parts[2])
                if 0 <= i < len(self.run.legs):
                    self.run.current = i
                    leg = self.run.legs[i]
                    leg["label"] = parts[3]
                    leg["target"] = float(parts[4])
                    leg["start"] = float(parts[5])
                    leg["timeout"] = int(parts[6])
            elif kind == "END" and len(parts) >= 8 and self.run is not None:
                i = int(parts[2])
                if 0 <= i < len(self.run.legs):
                    self.run.legs[i]["result"] = (float(parts[4]), int(parts[5]),
                                                  float(parts[6]), int(parts[7]))
                    self.showResults()
            elif kind == "DONE" and self.run is not None:
                self.run.done = True
                self.run.cancelled = len(parts) >= 3 and parts[2] == "1"
                self.run.current = -1
                self.showResults()
                self.runStatus.config(
                    text="run cancelled" if self.run.cancelled else "run finished",
                    fg=ORANGE if self.run.cancelled else CYAN)
        except ValueError:
            return
        self.scheduleRedraw()

    def showResults(self):
        self.results.config(state="normal")
        self.results.delete("1.0", "end")
        if self.run is None:
            self.results.config(state="disabled")
            return
        unit = UNITS[0 if self.run.angular else 1]
        self.results.insert("end", f"{'leg':<10}{'target':>9}{'overshoot':>11}{'settle':>9}"
                                   f"{'final':>9}{'time':>9}\n", "dim")
        for leg in self.run.legs:
            r = leg["result"]
            if r is None:
                self.results.insert("end", f"{leg['label']:<10}{leg['target']:>8.1f}{unit:<1}"
                                           f"{'...':>11}\n", "dim")
                continue
            overshoot, settle, final, dur = r
            tag = "ok" if settle >= 0 else "bad"
            self.results.insert(
                "end", f"{leg['label']:<10}{leg['target']:>8.1f}{unit:<1}{overshoot:>11.2f}"
                       f"{(str(settle) + 'ms') if settle >= 0 else 'never':>9}{final:>9.2f}"
                       f"{dur:>7}ms\n", tag)
        self.results.config(state="disabled")

    # -- graph --
    def scheduleRedraw(self):
        if not self._redrawPending:
            self._redrawPending = True
            self.after(30, self.redrawGraph)

    def redrawGraph(self):
        self._redrawPending = False
        c = self.graph
        c.delete("all")
        W = max(200, c.winfo_width())
        H = max(120, c.winfo_height())
        x0, x1 = GRAPH_PAD_L, W - GRAPH_PAD_R
        y0, y1 = GRAPH_PAD_T, H - GRAPH_PAD_B
        run = self.run

        # Value scale: everything the run touches (setpoints and trace), so
        # an overshoot past the biggest target is still on the plot.
        lo, hi = 0.0, 90.0
        if run is not None:
            vals = [leg["level"] for leg in run.legs] + [leg["target"] for leg in run.legs]
            vals += [leg["start"] for leg in run.legs if leg["start"] is not None]
            for leg in run.legs:
                for s in leg["samples"]:
                    vals.append(s[2])
            vals.append(0.0)
            lo, hi = min(vals), max(vals)
        minVal, maxVal, step = compute_scale(lo, hi)

        def toY(v):
            t = (v - minVal) / (maxVal - minVal)
            return y1 - max(0.0, min(1.0, t)) * (y1 - y0)

        # grid + labels
        v = minVal
        while v <= maxVal + step * 0.5:
            y = toY(v)
            c.create_line(x0, y, x1, y, fill=GRID)
            c.create_text(x0 - 6, y, text=f"{v:g}", anchor="e", fill=GRAY, font=("Consolas", 9))
            v += step
        c.create_line(x0, y0, x0, y1, fill=AXIS)
        c.create_line(x0, y1, x1, y1, fill=AXIS)

        if run is None:
            c.create_text((x0 + x1) / 2, (y0 + y1) / 2, fill=DIM, font=("Segoe UI", 11),
                          text="run a test or sweep -- the target (pink) and the robot's "
                               "response (green) plot here")
            return

        unit = UNITS[0 if run.angular else 1]
        c.create_text(x0 + 4, y0 - 12, anchor="w", fill=GRAY, font=("Consolas", 9),
                      text=f"{'heading (deg)' if run.angular else 'position (in)'} vs time"
                           + ("   -- CANCELLED" if run.cancelled else ""))

        # Time slots per leg, sized by timeout (like the brain's sweep graph).
        total = run.total_timeout()
        slots = []
        x = x0
        for leg in run.legs:
            w = max(1.0, (x1 - x0) * max(1, leg["timeout"]) / total)
            slots.append((x, w))
            x += w

        # Setpoint schedule (pink): the actual target once the leg has
        # started (it's relative to where the last leg really stopped),
        # the ideal level until then.
        prevY = None
        for i, leg in enumerate(run.legs):
            sx, sw = slots[i]
            y = toY(leg["target"])
            if prevY is not None:
                c.create_line(sx, prevY, sx, y, fill=TARGET_LINE, width=2)
            c.create_line(sx, y, sx + sw, y, fill=TARGET_LINE, width=2)
            prevY = y
            if i > 0:
                c.create_line(sx, y0, sx, y1, fill=AXIS, dash=(3, 3))
            c.create_text(sx + 4, y0 + 4, anchor="nw", fill=GRAY, font=("Consolas", 9),
                          text=f"{leg['label']} {leg['target']:g}{unit}")
            # settle band around the target, faint
            band = 1.5 if run.angular else 1.0
            c.create_rectangle(sx, toY(leg["target"] + band), sx + sw, toY(leg["target"] - band),
                               fill="", outline=AXIS, dash=(1, 4))

        # Actual trace (green), continuous across legs.
        pts = []
        for i, leg in enumerate(run.legs):
            sx, sw = slots[i]
            tmo = max(1, leg["timeout"])
            for (t, _target, actual, _err) in leg["samples"]:
                px = sx + min(1.0, t / tmo) * (sw - 1)
                pts.extend((px, toY(actual)))
        if len(pts) >= 4:
            c.create_line(*pts, fill=ACTUAL_LINE, width=2)
        elif len(pts) == 2:
            c.create_oval(pts[0] - 2, pts[1] - 2, pts[0] + 2, pts[1] + 2, fill=ACTUAL_LINE,
                          outline="")

        # per-leg result stamp under the graph line
        for i, leg in enumerate(run.legs):
            if leg["result"]:
                sx, sw = slots[i]
                overshoot, settle, _final, _dur = leg["result"]
                c.create_text(sx + sw - 4, y1 - 4, anchor="se",
                              fill=CYAN if settle >= 0 else RED, font=("Consolas", 9),
                              text=f"os {overshoot:.1f}  {settle if settle >= 0 else '--'}ms")

    # -- files --
    def currentGains(self):
        """What's in the boxes (i.e. the brain's values, or what's just been
        typed) as {"angular": [...], "lateral": [...]}."""
        out = {}
        for mode in range(2):
            vals = []
            for gain in range(3):
                try:
                    vals.append(float(self.entryVars[(mode, gain)].get()))
                except ValueError:
                    vals.append(self.brainGain(mode, gain))
            out[MODES[mode]] = vals
        return out

    def copyCode(self):
        code = controller_code(self.currentGains())
        self.clipboard_clear()
        self.clipboard_append(code)
        win = tk.Toplevel(self)
        win.title("Controller settings")
        txt = tk.Text(win, width=90, height=4, font=("Consolas", 10), bg=CARD, fg=GREEN)
        txt.pack(fill="both", expand=True)
        txt.insert("1.0", code)
        tk.Label(win, text="copied to the clipboard -- paste over the two lines in src/main.cpp",
                 anchor="w").pack(fill="x")

    def writeMainCpp(self):
        gains = self.currentGains()
        code = controller_code(gains)
        if not messagebox.askyesno("Write to main.cpp",
                                   f"Replace the two lemlib::ControllerSettings lines in\n"
                                   f"{os.path.normpath(MAIN_CPP)}\nwith:\n\n{code}",
                                   parent=self):
            return
        try:
            src = open(MAIN_CPP, encoding="utf-8").read()
        except OSError as e:
            messagebox.showerror("Write to main.cpp", str(e), parent=self)
            return
        new = src
        for line in code.strip().split("\n"):
            name = line.split()[1].split("(")[0]
            new, n = re.subn(r"lemlib::ControllerSettings\s+" + name + r"\s*\([^)]*\)\s*;",
                             line, new, count=1)
            if n == 0:
                messagebox.showerror("Write to main.cpp",
                                     f"couldn't find the {name} line in main.cpp", parent=self)
                return
        with open(MAIN_CPP, "w", encoding="utf-8", newline="") as f:
            f.write(new)
        self.runStatus.config(text="main.cpp updated -- rebuild and upload to make it stick",
                              fg=GREEN)

    def saveGains(self):
        os.makedirs(GAINS_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(parent=self, initialdir=GAINS_DIR,
                                            defaultextension=".json",
                                            filetypes=[("gains", "*.json")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.currentGains(), f, indent=2)

    def loadGains(self):
        os.makedirs(GAINS_DIR, exist_ok=True)
        path = filedialog.askopenfilename(parent=self, initialdir=GAINS_DIR,
                                          filetypes=[("gains", "*.json")])
        if not path:
            return
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        for mode in range(2):
            vals = data.get(MODES[mode])
            if not isinstance(vals, list) or len(vals) != 3:
                continue
            for gain in range(3):
                self.setGain(mode, gain, float(vals[gain]))

    def saveRunCsv(self):
        if self.run is None:
            self.runStatus.config(text="no run to save yet", fg=ORANGE)
            return
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".csv",
                                            filetypes=[("csv", "*.csv")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            f.write("leg,ms,target,actual,error\n")
            for leg in self.run.legs:
                for (t, target, actual, error) in leg["samples"]:
                    f.write(f"{leg['label']},{t:.0f},{target:.2f},{actual:.2f},{error:.2f}\n")


# ─── Stand-alone demo ──────────────────────────────────────────────────────
class _FakeLink:
    """Prints what would go to the robot and answers with a made-up tuner
    state, so the window can be looked at without a brain."""

    def __init__(self):
        self.gains = {"angular": [5.6, 0.001, 28.59], "lateral": [10.0, 0.0, 28.0]}
        self.mode, self.sel, self.digit = 0, 0, -2
        self.busy = False
        self.lines = []

    def _send(self, line):
        print("->", line)
        f = line.split()
        if f[:2] == ["PID", "SET"]:
            self.gains[MODES[int(f[2])]][int(f[3])] = float(f[4])
        elif f[:2] == ["PID", "MODE"]:
            self.mode = int(f[2])
        elif f[:2] == ["PID", "SEL"]:
            self.sel = int(f[2])
        elif f[:2] == ["PID", "DIGIT"]:
            self.digit = int(f[2])
        elif f[:2] == ["PID", "SWEEP"] or f[:2] == ["PID", "TEST"]:
            self.startFakeRun(int(f[2]), f[1] == "SWEEP", int(f[3]) if len(f) > 3 else 0)
        elif f[:2] == ["PID", "STOP"]:
            self.lines.append("PID|DONE|1")
            self.busy = False

    def state(self):
        return {"screen": "PID_TUNER", "breadcrumb": MODES[self.mode].upper(),
                "pid": {"mode": self.mode, "sel": self.sel, "digitExp": self.digit,
                        "angular": self.gains["angular"], "lateral": self.gains["lateral"],
                        "busy": self.busy},
                "result": ("turn90", 2.1, 640, 0.3, 1200), "selected": self.sel, "step": 0.01}

    def startFakeRun(self, mode, sweep, idx):
        angular = mode == 0
        if sweep:
            specs = ([("turn45", 45, 1200), ("turn90", 90, 1500), ("turn180", 180, 2000),
                      ("return0", 0, 1800)] if angular else
                     [("drive12", 12, 1500), ("drive24", 24, 2000), ("drive48", 48, 3000),
                      ("return0", 0, 3500)])
        else:
            specs = [([("turn90", 90, 2500), ("turn180", 180, 3000), ("turn0", 0, 2500)]
                      if angular else
                      [("drive24", 24, 3500), ("drive48", 48, 5000), ("driveBack", -24, 3500)])[idx]]
        levels, cum = [], 0.0
        for i, (label, size, tmo) in enumerate(specs):
            cum = size if (sweep and i == len(specs) - 1) or not sweep else cum + size
            levels.append(cum)
        self.lines.append("PID|RUN|%d|%d|" % (angular, len(specs)) +
                          ";".join(f"{l},{lv:.2f},{t}" for (l, _s, t), lv in zip(specs, levels)))
        # underdamped second-order response for each leg, chained
        pos = 0.0
        for i, (label, size, tmo) in enumerate(specs):
            target = levels[i]
            start = pos
            self.lines.append(f"PID|LEG|{i}|{label}|{target:.2f}|{start:.2f}|{tmo}")
            self.lines.append("CSV,ms,target,actual,error")
            wn, zeta = 6.0, 0.45
            t = 0
            while t < tmo * 0.8:
                tt = t / 1000.0
                wd = wn * math.sqrt(1 - zeta ** 2)
                resp = 1 - math.exp(-zeta * wn * tt) * (math.cos(wd * tt) +
                                                       zeta / math.sqrt(1 - zeta ** 2) * math.sin(wd * tt))
                pos = start + (target - start) * resp
                self.lines.append(f"CSV,{t},{target:.2f},{pos:.2f},{target - pos:.2f}")
                t += 20
            self.lines.append(f"PID|END|{i}|{label}|{abs(target - start) * 0.2:.2f}|{int(tmo * 0.5)}|"
                              f"{target - pos:.2f}|{int(tmo * 0.8)}")
        self.lines.append("PID|DONE|0")
        self.busy = True


def _demo():
    root = tk.Tk()
    root.withdraw()
    link = _FakeLink()
    win = PidTunerWindow(root, link)
    win.protocol("WM_DELETE_WINDOW", root.destroy)

    def tick():
        win.applyState(link.state())
        # drip the fake run out at roughly real time
        for _ in range(6):
            if link.lines:
                line = link.lines.pop(0)
                win.feedLine(line)
                if line.startswith("PID|DONE"):
                    link.busy = False
        root.after(100, tick)
    tick()
    root.mainloop()


if __name__ == "__main__":
    _demo()
