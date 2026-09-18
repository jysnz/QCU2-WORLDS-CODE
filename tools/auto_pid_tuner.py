#!/usr/bin/env python3
"""
Auto PID Tuner -- the laptop half of the brain's AUTO PID TUNE screen.

Opened automatically by tools/remote_touch.py when the brain enters that
screen (HOME -> AUTO PID TUNE), like the manual tuner's window. The robot
does the tuning itself (src/auto_tuner.cpp): kP up until it oscillates,
kD up until it's smooth, kI up until any leftover overshoot fades, one
big verification run. This window is where you watch it happen.

WHAT'S ON IT
    Left      which controller to tune (ANGULAR turns / LATERAL drive),
              START and STOP, the phase tracker (kP -> kD -> kI -> verify),
              the gains as they change run by run, the values the tuner
              has recorded so far (Pu, best kD, best kI) and its status
              line -- the same words the brain's footer shows.
    Right     the graph of one run: the target (pink) and the robot's
              heading or position chasing it (green), against time, with
              that run's gains and verdict above it. It follows the run in
              progress; click any run in the history underneath (or a bar
              in the overshoot strip) to look at an earlier one, and tick
              "follow live" to snap back. The strip plots every run's
              overshoot, coloured by phase, against the settle band, so
              the whole tune can be read at a glance.
    STOP      (or Esc) aborts the tune; the best gains found so far stay
              on the robot.

SAVING THE RESULT
    As in the manual tuner: Copy as C++ / Write to main.cpp put the gains
    into the two lemlib::ControllerSettings lines (both controllers, the
    untouched one as the brain has it), Save gains keeps them as JSON,
    Save log CSV writes every run's gains, metrics and samples.

RUN ON ITS OWN
    python tools/auto_pid_tuner.py    opens the window against a fake
    robot that plays a made-up tune through, to check the layout with no
    brain plugged in. Nothing is sent anywhere.

WIRE FORMAT (see include/auto_tuner.hpp)
    robot -> laptop  RUI|AUTO_TUNER|...|AUTO:...|MSG:...   state, a few /s
                     AT|RUN|run|angular|phase|P|I|D|label|target|start|timeoutMs
                     AT|S|ms|target|actual|error
                     AT|END|run|overshoot|settleMs|final|durMs|crossings|reversals|verdict
                     AT|DONE|phase|P|I|D
    laptop -> robot  PID AUTOSTART <mode>   PID AUTOMODE <mode>   PID STOP
"""

import json
import math
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from pid_tuner import (ACTUAL_LINE, AXIS, BG, CARD, CARD_SEL, CYAN, DIM, GAINS_DIR, GRAY,   # noqa: E402
                       GREEN, GRID, MAIN_CPP, MODES, MODE_COLORS, MODE_TITLES, ORANGE, RED,
                       TARGET_LINE, UNITS, WHITE, compute_scale, controller_code, fmt_gain,
                       write_controller_lines)

PURPLE = "#B48CFF"

# Phase numbers as auto_tuner.cpp sends them.
PHASES = ("IDLE", "P-search", "D-search", "I-search", "VERIFY", "DONE", "ABORTED", "FAILED")
PHASE_COLORS = {1: CYAN, 2: ORANGE, 3: PURPLE, 4: GREEN}
STEPS = ((1, "1  kP"), (2, "2  kD"), (3, "3  kI"), (4, "4  verify"))  # the tracker's chips
VERDICT_COLORS = {"oscillating": ORANGE, "smooth": GREEN, "bounce": "#FFD166",
                  "under": GRAY, "no_motion": GRAY, "cancelled": RED}
SETTLE_BAND = (1.5, 1.0)   # kAngularSettleBand / kLateralSettleBand, per mode


class AutoRun:
    """One of the tuner's runs as the brain streams it: gains and target
    up front (AT|RUN), samples (AT|S), then the metrics + verdict (AT|END)."""

    def __init__(self, idx, angular, phase, gains, label, target, start, timeout):
        self.idx = idx
        self.angular = angular
        self.phase = phase
        self.gains = gains               # [kP, kI, kD] this run drove with
        self.label = label
        self.target = target
        self.start = start
        self.timeout = max(1, timeout)
        self.samples = []                # (ms, target, actual, error)
        self.result = None               # dict once AT|END arrives

    def unit(self):
        return UNITS[0 if self.angular else 1]


class AutoPidTunerWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.title("Auto PID Tuner -- V5")
        self.configure(bg=BG)
        self.auto = None               # last "auto" dict from the brain
        self.onBrainScreen = False
        self.running = False
        self.mode = 0                  # controller shown / to tune (0 angular, 1 lateral)
        self.pendingMode = None        # (mode, sent at) until the brain echoes it
        self.runs = []                 # AutoRun, in order
        self.selected = None           # index into runs shown on the graph
        self.follow = tk.BooleanVar(value=True)
        self._redrawPending = False
        self.protocol("WM_DELETE_WINDOW", self.withdraw)
        self._buildUi()
        self.bind("<Escape>", lambda e: self.stop())
        self.redrawGraph()
        self.redrawStrip()
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

        # Left: controller, start/stop, phases, gains, files --------------
        left = tk.Frame(body, bg=BG, width=300)
        left.pack(side="left", fill="y", padx=(10, 6), pady=6)

        modeRow = tk.Frame(left, bg=BG)
        modeRow.pack(fill="x", pady=(0, 6))
        self.modeButtons = []
        for m in range(2):
            b = tk.Button(modeRow, text=MODE_TITLES[m], command=lambda m=m: self.chooseMode(m),
                          bg=CARD, fg=GRAY, activebackground=CARD_SEL,
                          activeforeground=MODE_COLORS[m], relief="flat",
                          font=("Segoe UI", 10, "bold"), cursor="hand2")
            b.pack(side="left", fill="x", expand=True, padx=(0, 4) if m == 0 else 0, ipady=6)
            self.modeButtons.append(b)

        self.startBtn = tk.Button(left, text="START AUTO TUNE", command=self.start,
                                  bg="#143a1e", fg=WHITE, activebackground=GREEN, relief="flat",
                                  font=("Segoe UI", 12, "bold"), cursor="hand2")
        self.startBtn.pack(fill="x", pady=(0, 4), ipady=8)
        self.stopBtn = tk.Button(left, text="STOP  (Esc)", command=self.stop, bg="#3a1414",
                                 fg=WHITE, activebackground=RED, relief="flat",
                                 font=("Segoe UI", 10, "bold"), cursor="hand2")
        self.stopBtn.pack(fill="x", pady=(0, 10), ipady=4)

        # Phase tracker: four chips, the current one lit, done ones ticked.
        tracker = tk.Frame(left, bg=CARD, highlightthickness=1, highlightbackground=AXIS)
        tracker.pack(fill="x", pady=(0, 8))
        chips = tk.Frame(tracker, bg=CARD)
        chips.pack(fill="x", padx=6, pady=(6, 2))
        self.chips = {}
        for phase, text in STEPS:
            c = tk.Label(chips, text=text, bg="#0f1218", fg=GRAY, font=("Segoe UI", 9, "bold"),
                         padx=6, pady=3)
            c.pack(side="left", fill="x", expand=True, padx=2)
            self.chips[phase] = c
        self.phaseLabel = tk.Label(tracker, text="idle", anchor="w", bg=CARD, fg=GRAY,
                                   font=("Consolas", 10))
        self.phaseLabel.pack(fill="x", padx=8, pady=(2, 6))

        # Live gains of the controller being tuned.
        card = tk.Frame(left, bg=CARD, highlightthickness=2, highlightbackground=AXIS)
        card.pack(fill="x", pady=(0, 8))
        self.gainCard = card
        self.gainTitle = tk.Label(card, text=MODE_TITLES[0], bg=CARD, fg=CYAN,
                                  font=("Segoe UI", 10, "bold"), anchor="w")
        self.gainTitle.pack(fill="x", padx=8, pady=(6, 2))
        self.gainLabels = []
        self.gainNames = []
        for name in ("kP", "kI", "kD"):
            row = tk.Frame(card, bg=CARD)
            row.pack(fill="x", padx=8, pady=1)
            n = tk.Label(row, text=name, width=3, anchor="w", bg=CARD, fg=GRAY,
                         font=("Consolas", 12))
            n.pack(side="left", padx=(4, 2))
            v = tk.Label(row, text="0.000", anchor="e", bg="#0f1218", fg=WHITE, width=11,
                         font=("Consolas", 15))
            v.pack(side="left", padx=2, ipady=2)
            self.gainNames.append(n)
            self.gainLabels.append(v)
        rec = tk.Frame(card, bg=CARD)
        rec.pack(fill="x", padx=8, pady=(4, 8))
        self.recorded = tk.Label(rec, text="Pu --    best kD --    best kI --", anchor="w",
                                 bg=CARD, fg=GRAY, font=("Consolas", 9))
        self.recorded.pack(fill="x")

        self.message = tk.Label(left, text="pick a controller, clear the area, START",
                                anchor="w", justify="left", wraplength=290, bg=BG, fg=CYAN,
                                font=("Consolas", 9))
        self.message.pack(fill="x", pady=(0, 8))

        tk.Checkbutton(left, text="follow the run in progress", variable=self.follow,
                       command=self.onFollow, bg=BG, fg=WHITE, selectcolor="#0f1218",
                       activebackground=BG, activeforeground=CYAN,
                       font=("Segoe UI", 9)).pack(anchor="w", pady=(0, 8))

        files = tk.Frame(left, bg=BG)
        files.pack(fill="x")
        self._btn(files, "Copy as C++", self.copyCode, width=12, fg=GREEN)
        self._btn(files, "Write to main.cpp", self.writeMainCpp, width=15)
        files2 = tk.Frame(left, bg=BG)
        files2.pack(fill="x", pady=(4, 0))
        self._btn(files2, "Save gains", self.saveGains, width=10)
        self._btn(files2, "Save log CSV", self.saveLogCsv, width=12)

        tk.Label(left, text="The robot turns / drives 20-40 times, alternating\n"
                            "direction. Keep ~4 ft clear all round it.\n"
                            "Gains live in the brain's RAM until written to main.cpp.",
                 justify="left", anchor="w", bg=BG, fg=DIM, font=("Segoe UI", 8)).pack(
            fill="x", pady=(10, 0))

        # Right: graph, overshoot strip, run history --------------------------
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(6, 10), pady=6)
        self.graph = tk.Canvas(right, width=660, height=300, bg=CARD, highlightthickness=1,
                               highlightbackground=AXIS)
        self.graph.pack(fill="both", expand=True)
        self.graph.bind("<Configure>", lambda e: self.scheduleRedraw())

        tk.Label(right, text="overshoot per run (click a bar to view that run)", anchor="w",
                 bg=BG, fg=GRAY, font=("Segoe UI", 8)).pack(fill="x", pady=(6, 0))
        self.strip = tk.Canvas(right, width=660, height=90, bg=CARD, highlightthickness=1,
                               highlightbackground=AXIS)
        self.strip.pack(fill="x")
        self.strip.bind("<Configure>", lambda e: self.scheduleRedraw())
        self.strip.bind("<Button-1>", self.onStripClick)

        cols = ("run", "phase", "kP", "kI", "kD", "overshoot", "cross", "settle", "final",
                "verdict")
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Auto.Treeview", background="#0f1218", fieldbackground="#0f1218",
                        foreground=WHITE, rowheight=20, font=("Consolas", 9), borderwidth=0)
        style.configure("Auto.Treeview.Heading", background=CARD, foreground=GRAY,
                        font=("Segoe UI", 9), relief="flat")
        style.map("Auto.Treeview", background=[("selected", CARD_SEL)],
                  foreground=[("selected", CYAN)])
        histFrame = tk.Frame(right, bg=BG)
        histFrame.pack(fill="x", pady=(6, 0))
        self.history = ttk.Treeview(histFrame, columns=cols, show="headings", height=7,
                                    style="Auto.Treeview", selectmode="browse")
        widths = (40, 74, 64, 70, 64, 76, 46, 62, 60, 86)
        for c, w in zip(cols, widths):
            self.history.heading(c, text=c)
            self.history.column(c, width=w, anchor="e" if c not in ("phase", "verdict") else "w",
                                stretch=(c == "verdict"))
        sb = ttk.Scrollbar(histFrame, orient="vertical", command=self.history.yview)
        self.history.configure(yscrollcommand=sb.set)
        self.history.pack(side="left", fill="x", expand=True)
        sb.pack(side="left", fill="y")
        self.history.bind("<<TreeviewSelect>>", self.onHistorySelect)
        for name, color in (("p", CYAN), ("d", ORANGE), ("i", PURPLE), ("v", GREEN)):
            self.history.tag_configure(name, foreground=color)
        self.history.tag_configure("pending", foreground=GRAY)

    def _btn(self, parent, text, cmd, width=None, fg=WHITE):
        b = tk.Button(parent, text=text, command=cmd, width=width, bg="#22222B", fg=fg,
                      activebackground="#2d2d38", activeforeground=fg, relief="flat",
                      font=("Segoe UI", 9), cursor="hand2")
        b.pack(side="left", padx=2, pady=1)
        return b

    # -- sending --
    def send(self, line):
        self.link._send(line)

    def chooseMode(self, m):
        if self.running:
            self.message.config(text="a tune is running -- STOP it before switching", fg=ORANGE)
            return
        self.mode = m
        self.pendingMode = (m, time.time())
        self.send(f"PID AUTOMODE {m}")
        self.refreshModeUi()

    def start(self):
        if not self.onBrainScreen:
            self.message.config(text="the brain isn't on the AUTO PID TUNE screen", fg=ORANGE)
            return
        if self.running:
            self.message.config(text="already running -- STOP it first", fg=ORANGE)
            return
        self.runs = []
        self.selected = None
        self.history.delete(*self.history.get_children())
        self.follow.set(True)
        self.send(f"PID AUTOSTART {self.mode}")
        self.message.config(text=f"{MODES[self.mode]} auto tune requested...", fg=GRAY)
        self.scheduleRedraw()

    def stop(self):
        self.send("PID STOP")
        if self.running:
            self.message.config(text="stop sent", fg=ORANGE)

    # -- state from the brain (every RUI line) --
    def applyState(self, state):
        self.onBrainScreen = state["screen"] == "AUTO_TUNER"
        auto = state.get("auto")
        if self.onBrainScreen and auto:
            self.auto = auto
            self.running = auto["running"]
            pend = self.pendingMode
            if pend and (pend[0] == auto["mode"] or time.time() - pend[1] > 1.5):
                self.pendingMode = pend = None
            if not pend:
                self.mode = auto["mode"]
            self.refreshModeUi()
            phase = auto["phase"]
            for p, chip in self.chips.items():
                if phase == p:
                    chip.config(bg=PHASE_COLORS[p], fg="#000000")
                elif 0 < p < phase or phase == 5:
                    chip.config(bg="#0f1218", fg=PHASE_COLORS[p])
                else:
                    chip.config(bg="#0f1218", fg=GRAY)
            name = PHASES[phase] if 0 <= phase < len(PHASES) else "?"
            self.phaseLabel.config(
                text=f"{name}   run {auto['run']}",
                fg=GREEN if self.running else RED if phase in (6, 7) else
                CYAN if phase == 5 else GRAY)
            g = auto[MODES[self.mode]]
            for i in range(3):
                self.gainLabels[i].config(text=fmt_gain(g[i]) if i != 1 else f"{g[i]:.4f}")
                lit = (phase == 1 and i == 0) or (phase == 3 and i == 1) or (phase == 2 and i == 2)
                self.gainLabels[i].config(fg=MODE_COLORS[self.mode] if lit else WHITE)
            self.recorded.config(
                text=f"Pu {auto['pu']:.3f}    best kD {auto['bestD']:.3f}    "
                     f"best kI {auto['bestI']:.4f}")
            if auto["msg"]:
                self.message.config(text=auto["msg"],
                                    fg=RED if phase in (6, 7) else GREEN if phase == 5 else CYAN)
            self.brainStatus.config(
                text=f"brain: AUTO PID TUNE  {MODES[auto['mode']].upper()}  "
                     f"{'RUNNING' if self.running else 'idle'}",
                fg=GREEN if self.running else CYAN)
        else:
            self.running = False
            self.brainStatus.config(text=f"brain: on {state['screen']} -- tap AUTO PID TUNE on "
                                    "the brain (or mirror) to tune from here", fg=ORANGE)
        can = self.onBrainScreen and not self.running
        self.startBtn.config(state="normal" if can else "disabled")
        for b in self.modeButtons:
            b.config(state="normal" if can else "disabled")

    def refreshModeUi(self):
        for m, b in enumerate(self.modeButtons):
            on = m == self.mode
            b.config(bg=CARD_SEL if on else CARD, fg=MODE_COLORS[m] if on else GRAY)
        self.gainCard.config(highlightbackground=MODE_COLORS[self.mode])
        self.gainTitle.config(text=MODE_TITLES[self.mode], fg=MODE_COLORS[self.mode])

    # -- the run stream (AT|... lines) --
    def feedLine(self, line):
        parts = line.rstrip("\r\n").split("|")
        if len(parts) < 2 or parts[0] != "AT":
            return
        kind = parts[1]
        try:
            if kind == "S" and len(parts) >= 6:
                if not self.runs or self.runs[-1].result is not None:
                    return
                t, target, actual, error = (float(x) for x in parts[2:6])
                run = self.runs[-1]
                run.target = target
                # target - error rather than the raw pose: wrap-corrected on
                # angular runs, so the trace doesn't jump across +-180.
                run.samples.append((t, target, target - error, error))
            elif kind == "RUN" and len(parts) >= 12:
                run = AutoRun(int(parts[2]), parts[3] == "1", int(parts[4]),
                              [float(parts[5]), float(parts[6]), float(parts[7])], parts[8],
                              float(parts[9]), float(parts[10]), int(parts[11]))
                self.runs.append(run)
                self.history.insert("", "end", iid=str(run.idx), tags=("pending",), values=(
                    run.idx, PHASES[run.phase] if 0 <= run.phase < len(PHASES) else "?",
                    f"{run.gains[0]:.3f}", f"{run.gains[1]:.4f}", f"{run.gains[2]:.3f}",
                    "...", "", "", "", "running"))
                self.history.see(str(run.idx))
                if self.follow.get():
                    self.selected = len(self.runs) - 1
                    self.history.selection_set(str(run.idx))
            elif kind == "END" and len(parts) >= 10 and self.runs:
                idx = int(parts[2])
                run = next((r for r in self.runs if r.idx == idx), None)
                if run is None:
                    return
                run.result = {"overshoot": float(parts[3]), "settle": int(parts[4]),
                              "final": float(parts[5]), "dur": int(parts[6]),
                              "crossings": int(parts[7]), "reversals": int(parts[8]),
                              "verdict": parts[9]}
                r = run.result
                tag = {1: "p", 2: "d", 3: "i", 4: "v"}.get(run.phase, "pending")
                self.history.item(str(idx), tags=(tag,), values=(
                    run.idx, PHASES[run.phase] if 0 <= run.phase < len(PHASES) else "?",
                    f"{run.gains[0]:.3f}", f"{run.gains[1]:.4f}", f"{run.gains[2]:.3f}",
                    f"{r['overshoot']:.2f}", r["crossings"],
                    f"{r['settle']}ms" if r["settle"] >= 0 else "never", f"{r['final']:.2f}",
                    r["verdict"]))
            elif kind == "DONE" and len(parts) >= 6:
                phase = int(parts[2])
                self.message.config(
                    text=f"{PHASES[phase] if 0 <= phase < len(PHASES) else '?'}: "
                         f"kP {float(parts[3]):.3f}  kI {float(parts[4]):.4f}  "
                         f"kD {float(parts[5]):.3f}",
                    fg=GREEN if phase == 5 else RED)
        except (ValueError, tk.TclError):
            return
        self.scheduleRedraw()

    # -- selection --
    def onHistorySelect(self, _event=None):
        sel = self.history.selection()
        if not sel:
            return
        idx = int(sel[0])
        for i, run in enumerate(self.runs):
            if run.idx == idx:
                if i != self.selected:
                    self.selected = i
                    if i != len(self.runs) - 1:
                        self.follow.set(False)
                    self.scheduleRedraw()
                return

    def onFollow(self):
        if self.follow.get() and self.runs:
            self.selected = len(self.runs) - 1
            self.history.selection_set(str(self.runs[-1].idx))
            self.scheduleRedraw()

    def onStripClick(self, event):
        if not self.runs:
            return
        W = max(1, self.strip.winfo_width())
        i = int((event.x - 8) / max(1.0, (W - 16) / len(self.runs)))
        if 0 <= i < len(self.runs):
            self.history.selection_set(str(self.runs[i].idx))
            self.history.see(str(self.runs[i].idx))

    # -- graphs --
    def scheduleRedraw(self):
        if not self._redrawPending:
            self._redrawPending = True
            self.after(30, self._redraw)

    def _redraw(self):
        self._redrawPending = False
        self.redrawGraph()
        self.redrawStrip()

    def redrawGraph(self):
        c = self.graph
        c.delete("all")
        W = max(200, c.winfo_width())
        H = max(120, c.winfo_height())
        x0, x1 = 46, W - 12
        y0, y1 = 40, H - 22
        run = self.runs[self.selected] if self.runs and self.selected is not None and \
            0 <= self.selected < len(self.runs) else None

        lo, hi = 0.0, 90.0
        if run is not None:
            vals = [run.target, run.start, 0.0] + [s[2] for s in run.samples]
            lo, hi = min(vals), max(vals)
        minVal, maxVal, step = compute_scale(lo, hi)

        def toY(v):
            t = (v - minVal) / (maxVal - minVal)
            return y1 - max(0.0, min(1.0, t)) * (y1 - y0)

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
                          text="START a tune -- each run's target (pink) and the robot's "
                               "response (green) plot here")
            return

        unit = run.unit()
        phase = PHASES[run.phase] if 0 <= run.phase < len(PHASES) else "?"
        title = (f"run {run.idx}  {phase}  {run.label}  kP {run.gains[0]:.3f}  "
                 f"kI {run.gains[1]:.4f}  kD {run.gains[2]:.3f}")
        c.create_text(x0 + 4, y0 - 28, anchor="w", fill=PHASE_COLORS.get(run.phase, GRAY),
                      font=("Consolas", 9), text=title)
        if run.result:
            r = run.result
            c.create_text(x0 + 4, y0 - 12, anchor="w", fill=VERDICT_COLORS.get(r["verdict"], GRAY),
                          font=("Consolas", 9),
                          text=f"{r['verdict'].upper()}  overshoot {r['overshoot']:.2f}{unit}  "
                               f"crossings {r['crossings']}  reversals {r['reversals']}  "
                               f"settle {str(r['settle']) + 'ms' if r['settle'] >= 0 else 'never'}"
                               f"  final {r['final']:.2f}{unit}  {r['dur']}ms")
        else:
            c.create_text(x0 + 4, y0 - 12, anchor="w", fill=GREEN, font=("Consolas", 9),
                          text="running...")

        # setpoint (pink), settle band (dotted), heading/position vs time
        y = toY(run.target)
        c.create_line(x0, y, x1, y, fill=TARGET_LINE, width=2)
        band = SETTLE_BAND[0 if run.angular else 1]
        c.create_rectangle(x0, toY(run.target + band), x1, toY(run.target - band), fill="",
                           outline=AXIS, dash=(1, 4))
        c.create_text(x0 + 4, y0 + 4, anchor="nw", fill=GRAY, font=("Consolas", 9),
                      text=f"{'heading (deg)' if run.angular else 'position (in)'} vs time  "
                           f"-- target {run.target:g}{unit}, {run.timeout} ms window")
        pts = []
        for (t, _target, actual, _err) in run.samples:
            px = x0 + min(1.0, t / run.timeout) * (x1 - x0 - 1)
            pts.extend((px, toY(actual)))
        if len(pts) >= 4:
            c.create_line(*pts, fill=ACTUAL_LINE, width=2)
        elif len(pts) == 2:
            c.create_oval(pts[0] - 2, pts[1] - 2, pts[0] + 2, pts[1] + 2, fill=ACTUAL_LINE,
                          outline="")

    def redrawStrip(self):
        c = self.strip
        c.delete("all")
        W = max(100, c.winfo_width())
        H = max(40, c.winfo_height())
        x0, x1, y0, y1 = 8, W - 8, 8, H - 14
        if not self.runs:
            c.create_text(W / 2, H / 2, fill=DIM, font=("Segoe UI", 9),
                          text="every run's overshoot lands here, coloured by phase")
            return
        band = SETTLE_BAND[0 if self.runs[0].angular else 1]
        top = max([band * 2] + [r.result["overshoot"] for r in self.runs if r.result])
        slot = (x1 - x0) / len(self.runs)

        def toY(v):
            return y1 - max(0.0, min(1.0, v / top)) * (y1 - y0)

        yb = toY(band)
        c.create_line(x0, yb, x1, yb, fill=GREEN, dash=(2, 3))
        c.create_text(x1, yb - 2, anchor="se", fill=GREEN, font=("Consolas", 8),
                      text=f"settle band {band:g}")
        for i, run in enumerate(self.runs):
            bx0 = x0 + i * slot + 1
            bx1 = bx0 + max(2.0, slot - 2)
            color = PHASE_COLORS.get(run.phase, GRAY)
            if run.result:
                c.create_rectangle(bx0, toY(run.result["overshoot"]), bx1, y1, fill=color,
                                   outline="")
            else:
                c.create_rectangle(bx0, y0, bx1, y1, fill="", outline=color, dash=(2, 2))
            if i == self.selected:
                c.create_rectangle(bx0 - 1, y0 - 2, bx1 + 1, y1 + 1, fill="", outline=WHITE)
            if slot >= 14:
                c.create_text((bx0 + bx1) / 2, y1 + 2, anchor="n", fill=GRAY,
                              font=("Consolas", 7), text=str(run.idx))

    # -- files --
    def currentGains(self):
        """Both controllers' gains as the brain last reported them."""
        if self.auto:
            return {"angular": list(self.auto["angular"]), "lateral": list(self.auto["lateral"])}
        return {"angular": [0.0, 0.0, 0.0], "lateral": [0.0, 0.0, 0.0]}

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
        code = controller_code(self.currentGains())
        if not messagebox.askyesno("Write to main.cpp",
                                   f"Replace the two lemlib::ControllerSettings lines in\n"
                                   f"{os.path.normpath(MAIN_CPP)}\nwith:\n\n{code}",
                                   parent=self):
            return
        err = write_controller_lines(code)
        if err:
            messagebox.showerror("Write to main.cpp", err, parent=self)
            return
        self.message.config(text="main.cpp updated -- rebuild and upload to make it stick",
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

    def saveLogCsv(self):
        if not self.runs:
            self.message.config(text="no runs to save yet", fg=ORANGE)
            return
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".csv",
                                            filetypes=[("csv", "*.csv")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            f.write("run,phase,kP,kI,kD,label,overshoot,settleMs,final,durMs,crossings,"
                    "reversals,verdict,ms,target,actual,error\n")
            for run in self.runs:
                r = run.result or {}
                head = (f"{run.idx},{PHASES[run.phase] if 0 <= run.phase < len(PHASES) else '?'},"
                        f"{run.gains[0]:.4f},{run.gains[1]:.4f},{run.gains[2]:.4f},{run.label},"
                        f"{r.get('overshoot', '')},{r.get('settle', '')},{r.get('final', '')},"
                        f"{r.get('dur', '')},{r.get('crossings', '')},{r.get('reversals', '')},"
                        f"{r.get('verdict', '')}")
                for (t, target, actual, error) in run.samples:
                    f.write(f"{head},{t:.0f},{target:.2f},{actual:.2f},{error:.2f}\n")


# ─── Stand-alone demo ──────────────────────────────────────────────────────
class _FakeLink:
    """Prints what would go to the robot and plays a made-up tune: kP runs
    that oscillate more and more, kD runs that calm down, a kI run, the
    verify -- so the window can be looked at without a brain."""

    def __init__(self):
        self.mode = 0
        self.phase = 0
        self.run = 0
        self.gains = {"angular": [5.6, 0.001, 28.59], "lateral": [10.0, 0.0, 28.0]}
        self.pu = self.bestD = self.bestI = 0.0
        self.running = False
        self.msg = "pick a controller, clear the area, tap START"
        self.lines = []

    def _send(self, line):
        print("->", line)
        f = line.split()
        if f[:2] == ["PID", "AUTOMODE"]:
            self.mode = int(f[2])
        elif f[:2] == ["PID", "AUTOSTART"]:
            self.mode = int(f[2])
            self.startFakeTune()
        elif f[:2] == ["PID", "STOP"] and self.running:
            self.lines = [ln for ln in self.lines if not ln.startswith("AT|S")]
            self.lines.append("AT|DONE|6|%.4f|%.4f|%.4f" % tuple(self.gains[MODES[self.mode]]))

    def state(self):
        return {"screen": "AUTO_TUNER", "breadcrumb": MODES[self.mode].upper(),
                "auto": {"mode": self.mode, "phase": self.phase, "run": self.run,
                         "angular": self.gains["angular"], "lateral": self.gains["lateral"],
                         "pu": self.pu, "bestD": self.bestD, "bestI": self.bestI,
                         "running": self.running, "msg": self.msg}}

    def startFakeTune(self):
        angular = self.mode == 0
        size, tmo = (90.0, 2500) if angular else (24.0, 3500)
        band = SETTLE_BAND[0 if angular else 1]
        g = [0.0, 0.0, 0.0]
        self.gains[MODES[self.mode]] = g
        self.running = True
        self.run = 0
        self.lines = []
        # (phase, gains, damping zeta, natural freq): what each run "drives" with
        script = []
        p = 1.0
        for k in range(6):
            script.append((1, [p, 0, 0], 1.2 - k * 0.19, 3 + k * 0.8))
            p *= 1.3
        pu = p / 1.3
        d = 0
        for k in range(4):
            d += pu * 0.5
            script.append((2, [pu, 0, d], 0.35 + k * 0.2, 6.0))
        script.append((3, [pu, 0.0005, d], 0.9, 6.0))
        script.append((4, [pu, 0.0005, d], 0.9, 6.0))
        for i, (phase, gains, zeta, wn) in enumerate(script):
            self.run += 1
            target = size if i % 2 == 0 else -size
            if phase == 4:
                target *= 2
            label = {1: "P", 2: "D", 3: "I", 4: "verify"}[phase]
            self.lines.append(("PHASE", phase, list(gains)))
            self.lines.append(f"AT|RUN|{self.run}|{int(angular)}|{phase}|{gains[0]:.4f}|"
                              f"{gains[1]:.4f}|{gains[2]:.4f}|{label}|{target:.2f}|0.00|{tmo}")
            pos, t, crossings, overshoot = 0.0, 0, 0, 0.0
            prevSign = 1 if target >= 0 else -1
            while t < tmo * 0.8:
                tt = t / 1000.0
                if zeta < 1:
                    wd = wn * math.sqrt(1 - zeta ** 2)
                    resp = 1 - math.exp(-zeta * wn * tt) * (
                        math.cos(wd * tt) + zeta / math.sqrt(1 - zeta ** 2) * math.sin(wd * tt))
                else:
                    resp = 1 - math.exp(-wn * tt / zeta) * (1 + wn * tt / zeta)
                pos = target * resp
                err = target - pos
                sign = 1 if err >= 0 else -1
                if abs(err) > band / 2 and sign != prevSign:
                    crossings += 1
                    prevSign = sign
                if crossings:
                    overshoot = max(overshoot, abs(err))
                self.lines.append(f"AT|S|{t}|{target:.2f}|{pos:.2f}|{err:.2f}")
                t += 20
            verdict = ("oscillating" if crossings >= 2 and overshoot >= 0.08 * abs(target) else
                       "bounce" if overshoot > band else "under" if crossings == 0 else "smooth")
            self.lines.append(f"AT|END|{self.run}|{overshoot:.2f}|{int(tmo * 0.5)}|"
                              f"{target - pos:.2f}|{int(tmo * 0.8)}|{crossings}|{crossings}|"
                              f"{verdict}")
            if phase == 1 and verdict == "oscillating":
                self.pu = gains[0]
            if phase == 2:
                self.bestD = gains[2]
            if phase == 3:
                self.bestI = gains[1]
        self.lines.append(("PHASE", 5, list(script[-1][1])))
        self.lines.append("AT|DONE|5|%.4f|%.4f|%.4f" % tuple(script[-1][1]))
        self.run = 0


def _demo():
    root = tk.Tk()
    root.withdraw()
    link = _FakeLink()
    win = AutoPidTunerWindow(root, link)
    win.protocol("WM_DELETE_WINDOW", root.destroy)

    def tick():
        # drip the fake tune out at roughly real time
        for _ in range(6):
            if not link.lines:
                break
            line = link.lines.pop(0)
            if isinstance(line, tuple):
                _, link.phase, gains = line
                link.gains[MODES[link.mode]][:] = gains
                link.msg = f"{PHASES[link.phase]} -- fake robot"
                if link.phase == 5:
                    link.running = False
                    link.msg = "DONE  kP %.3f  kI %.4f  kD %.3f" % tuple(gains)
                continue
            if line.startswith("AT|RUN"):
                link.run = int(line.split("|")[2])
            win.feedLine(line)
            if line.startswith("AT|DONE"):
                link.running = False
                if line.split("|")[2] == "6":
                    link.phase = 6
                    link.msg = "ABORTED -- fake robot"
        win.applyState(link.state())
        root.after(100, tick)
    tick()
    root.mainloop()


if __name__ == "__main__":
    _demo()
