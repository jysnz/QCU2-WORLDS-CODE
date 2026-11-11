#!/usr/bin/env python3
"""
Lift Calibration -- the laptop half of the brain's LIFT CAL screen.

Opened automatically by tools/remote_touch.py when the brain enters that screen
(HOME -> LIFT CAL). Every lift level has two angles, in motor degrees from the
bumper home position:

    LIFT   where the lift goes to raise to the level
    DROP   where it goes to drop the piece at that level -- once it gets there
           the claws open by themselves

The first stage (GROUND) has no drop angle: dropping there just opens the claws.

HOW TO CALIBRATE A LEVEL
    1. Jog the lift to the height you want (JOG buttons, or R1 / R2 on the
       controller), watching the gauge.
    2. Press SAVE next to the LIFT or DROP angle of that level -- or just type a
       number in its box and press Enter.
    3. GO / DROP drive the lift there to check it. DROP also opens the claws on
       arrival, exactly as it will in a match.
    A level with a dot (•) differs from what is written in the source code.

KEYS (when you are not typing in a box)
    Up / Down   select level      G   go to LIFT angle      D   drop
    S           save LIFT angle   Shift+S   save DROP angle
    [  ]        jog down / up     H   home                  Esc   stop

SAVING
    Angles live in the brain's RAM until written into the code. SAVE TO CODE
    (Ctrl+S, or SAVE CODE on the brain while this window is open) replaces the
    block between the LIFT-LEVELS markers in src/lift_calibration.cpp with the
    table shown in the code box; rebuild and upload afterwards. Copy as C++ puts
    the same table on the clipboard instead.

RUN ON ITS OWN
    python tools/lift_calibration.py   opens the window against a fake lift.

WIRE FORMAT (see include/lift_calibration.hpp)
    robot -> laptop  RUI|LIFT_CAL|...|LIFT:<sel>,<live>,<moving>,<bumper>,<upper>,<claw>,<claw2>,<dropping>,<saveReq>
                     ~<name>,<lift>,<drop>;...|MSG:...     (drop = -1: the level has none)
    laptop -> robot  LIFT SEL i | SET i deg | GO i | SAVE i | SETDROP i deg | GODROP i
                     | SAVEDROP i | JOG deg | HOME | STOP | CLAW which state | SAVED ok
"""

import os
import re
import sys
import tkinter as tk
from tkinter import messagebox

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from pid_tuner import AXIS, BG, CARD, CARD_SEL, CYAN, DIM, GRAY, GREEN, ORANGE, RED, WHITE  # noqa: E402

SOURCE = os.path.join(os.path.dirname(HERE), "src", "lift_calibration.cpp")
BEGIN, END = "// LIFT-LEVELS-BEGIN", "// LIFT-LEVELS-END"

# Must match LIFT_CLAW_OPEN_RETRACTED in include/lift_calibration.hpp: the claws
# are built extended (holding a piece), so open = retracted.
CLAW_OPEN_RETRACTED = True
JOG_PRESETS = (1, 5, 10, 25)
INPUT = "#0f1218"
BTN = "#22222B"
GO_BG = "#143a1e"
DROP_BG = "#3a2a14"
SAVE_BG = "#1c2838"
STOP_BG = "#3a1414"
YELLOW = "#FFD166"
FONT = "Segoe UI"
MONO = "Consolas"


def table_code(levels):
    """The three C++ lines between the markers, from [(name, lift, drop), ...].
    A level with no drop angle (drop < 0) is written as 0.0."""
    names = ", ".join(f'"{n}"' for n, _, _ in levels)
    angles = ", ".join(f"{d:.1f}" for _, d, _ in levels)
    drops = ", ".join(f"{max(d, 0.0):.1f}" for _, _, d in levels)
    return (f"static const char *const kLevelNames[LIFT_LEVEL_COUNT] = {{{names}}};\n"
            f"static const double kDefaultAngles[LIFT_LEVEL_COUNT] = {{{angles}}};\n"
            f"static const double kDefaultDropAngles[LIFT_LEVEL_COUNT] = {{{drops}}};\n")


def claw_is_open(extended):
    return extended != CLAW_OPEN_RETRACTED


class LiftCalWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.title("Lift Calibration -- V5")
        self.configure(bg=BG)
        self.levels = []          # [(name, lift, drop)] as the brain has them (drop < 0: none)
        self.baseline = None      # the same, as last written to source (or first seen)
        self.sel = 0
        self.live = 0.0
        self.upper = 500.0
        self.moving = False
        self.dropping = False
        self.bumper = False
        self.claw1 = self.claw2 = True        # extended?
        self.onBrainScreen = False
        self.lastSaveReq = None   # brain's SAVE CODE counter; a new value means "write the code"
        self.codeShown = None
        self.rows = []            # per level: dict of widgets
        self.step = tk.StringVar(value="10")
        self.protocol("WM_DELETE_WINDOW", self.withdraw)
        self._build()
        self._bindKeys()
        self.update_idletasks()
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())

    # ── layout ────────────────────────────────────────────────────────────
    def _chip(self, parent, text, fg=GRAY):
        return tk.Label(parent, text=text, bg=CARD, fg=fg, font=(FONT, 9, "bold"), padx=10,
                        pady=3)

    def _btn(self, parent, text, cmd, bg=BTN, fg=WHITE, size=9, width=None, pad=6):
        return tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg, activebackground="#2d2d38",
                         activeforeground=fg, disabledforeground="#555", relief="flat",
                         font=(FONT, size, "bold"), cursor="hand2", width=width, pady=pad)

    def _card(self, parent, title=None):
        f = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=AXIS)
        if title:
            tk.Label(f, text=title, bg=CARD, fg=GRAY, font=(FONT, 8, "bold")).pack(
                anchor="w", padx=10, pady=(6, 2))
        return f

    def _build(self):
        # status strip ------------------------------------------------------
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=12, pady=(10, 6))
        tk.Label(top, text="LIFT CALIBRATION", bg=BG, fg=CYAN, font=(FONT, 13, "bold")).pack(
            side="left")
        from reconnect_button import add_reconnect_button
        add_reconnect_button(self, top, side="right", padx=(6, 0))
        self.chipBrain = self._chip(top, "brain: waiting", ORANGE)
        self.chipBrain.pack(side="right", padx=(6, 0))
        self.chipState = self._chip(top, "READY", GREEN)
        self.chipState.pack(side="right", padx=(6, 0))
        self.chipBumper = self._chip(top, "bumper off")
        self.chipBumper.pack(side="right")

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=12)

        # left: the levels ----------------------------------------------------
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="both", expand=True)
        self.table = self._card(left)
        self.table.pack(fill="x")
        self.listFrame = tk.Frame(self.table, bg=CARD)
        self.listFrame.pack(fill="x", padx=8, pady=8)
        self.tipLabel = tk.Label(
            left, bg=BG, fg=DIM, font=(FONT, 8), anchor="w", justify="left",
            text="GROUND has no drop angle: DROP there just opens the claws.\n"
                 "At every other level DROP goes to the drop angle and opens the claws on "
                 "arrival.")
        self.tipLabel.pack(fill="x", pady=(6, 0))

        # what SAVE TO CODE will write -- also fills the space under the table
        codeCard = self._card(left, "CODE THAT SAVE WRITES  (src/lift_calibration.cpp)")
        codeCard.pack(fill="both", expand=True, pady=(8, 0))
        self.codeText = tk.Text(codeCard, height=6, bg=INPUT, fg=GREEN, relief="flat", wrap="word",
                                font=(MONO, 9), highlightthickness=0, padx=8, pady=6,
                                state="disabled", cursor="arrow")
        self.codeText.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        # right: gauge, jog, claws ---------------------------------------------
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="y", padx=(12, 0))

        live = self._card(right, "LIFT ANGLE")
        live.pack(fill="x")
        self.liveLabel = tk.Label(live, text="--", bg=CARD, fg=GREEN, font=(MONO, 30, "bold"))
        self.liveLabel.pack(anchor="w", padx=10)
        self.gauge = tk.Canvas(live, width=250, height=250, bg=INPUT, highlightthickness=0)
        self.gauge.pack(padx=10, pady=(2, 10))
        self.gauge.bind("<Configure>", lambda e: self.drawGauge())

        jog = self._card(right, "JOG")
        jog.pack(fill="x", pady=(8, 0))
        jr = tk.Frame(jog, bg=CARD)
        jr.pack(fill="x", padx=10, pady=(0, 4))
        self.stepBtns = {}
        for v in JOG_PRESETS:
            b = self._btn(jr, str(v), lambda v=v: self.step.set(str(v)), size=9, pad=2, width=3)
            b.pack(side="left", padx=(0, 3))
            self.stepBtns[v] = b
        tk.Entry(jr, textvariable=self.step, width=5, justify="center", bg=INPUT, fg=WHITE,
                 insertbackground=WHITE, relief="flat", font=(MONO, 11)).pack(
            side="left", padx=(4, 2), ipady=3)
        tk.Label(jr, text="deg", bg=CARD, fg=GRAY).pack(side="left")
        self.step.trace_add("write", lambda *_: self._markStep())
        jr2 = tk.Frame(jog, bg=CARD)
        jr2.pack(fill="x", padx=10, pady=(0, 8))
        self._btn(jr2, "▼  DOWN", lambda: self.jog(-1)).pack(side="left", fill="x", expand=True,
                                                          padx=(0, 3))
        self._btn(jr2, "▲  UP", lambda: self.jog(1)).pack(side="left", fill="x", expand=True)

        claws = self._card(right, "CLAWS")
        claws.pack(fill="x", pady=(8, 0))
        cr = tk.Frame(claws, bg=CARD)
        cr.pack(fill="x", padx=10, pady=(0, 8))
        self.clawBtns = [self._btn(cr, "CLAW 1", lambda: self.send("LIFT CLAW 1 2")),
                         self._btn(cr, "CLAW 2", lambda: self.send("LIFT CLAW 2 2")),
                         self._btn(cr, "BOTH", self.allClaws)]
        for b in self.clawBtns:
            b.pack(side="left", fill="x", expand=True, padx=(0, 3))

        act = tk.Frame(right, bg=BG)
        act.pack(fill="x", pady=(8, 0))
        self._btn(act, "HOME  (H)", lambda: self.send("LIFT HOME"), fg=ORANGE).pack(
            side="left", fill="x", expand=True, padx=(0, 3))
        self._btn(act, "STOP  (Esc)", lambda: self.send("LIFT STOP"), bg=STOP_BG).pack(
            side="left", fill="x", expand=True)

        # bottom: message + files ------------------------------------------------
        bottom = tk.Frame(self, bg=BG)
        bottom.pack(fill="x", padx=12, pady=(10, 10))
        self.message = tk.Label(bottom, text="", anchor="w", justify="left", wraplength=560,
                                bg=BG, fg=CYAN, font=(MONO, 9))
        self.message.pack(side="left", fill="x", expand=True)
        self.writeBtn = self._btn(bottom, "SAVE TO CODE  (Ctrl+S)", self.writeSource, size=10,
                                  pad=8)
        self.writeBtn.pack(side="right")
        self._btn(bottom, "Copy as C++", self.copyCode, fg=GREEN, size=9).pack(side="right",
                                                                              padx=(0, 6))
        self.dirtyLabel = tk.Label(bottom, text="", bg=BG, fg=YELLOW, font=(FONT, 9, "bold"))
        self.dirtyLabel.pack(side="right", padx=(0, 10))
        self._markStep()

    def _bindKeys(self):
        def key(fn):
            def handler(e):
                if isinstance(self.focus_get(), tk.Entry):
                    return
                fn()
            return handler
        self.bind("<Escape>", lambda e: self.send("LIFT STOP"))
        self.bind("<Control-s>", lambda e: self.writeSource())
        self.bind("<Up>", key(lambda: self.select(max(0, self.sel - 1))))
        self.bind("<Down>", key(lambda: self.select(min(len(self.levels) - 1, self.sel + 1))))
        self.bind("<g>", key(lambda: self.goTo()))
        self.bind("<d>", key(lambda: self.goTo(True)))
        self.bind("<s>", key(lambda: self.saveHere()))
        self.bind("<S>", key(lambda: self.saveHere(True)))
        self.bind("<h>", key(lambda: self.send("LIFT HOME")))
        self.bind("<bracketleft>", key(lambda: self.jog(-1)))
        self.bind("<bracketright>", key(lambda: self.jog(1)))
        self.bind("<Button-1>", lambda e: e.widget.focus_set() if not isinstance(
            e.widget, tk.Entry) else None, add="+")

    # ── the level table ───────────────────────────────────────────────────
    def _rebuildRows(self):
        for w in self.listFrame.winfo_children():
            w.destroy()
        self.rows = []
        hdr = tk.Frame(self.listFrame, bg=CARD)
        hdr.pack(fill="x", pady=(0, 4))
        tk.Label(hdr, text="LEVEL", bg=CARD, fg=GRAY, font=(FONT, 8, "bold"), width=11,
                 anchor="w").grid(row=0, column=0, padx=(8, 0))
        tk.Label(hdr, text="LIFT ANGLE", bg=CARD, fg=WHITE, font=(FONT, 8, "bold")).grid(
            row=0, column=1, columnspan=3, padx=6)
        tk.Label(hdr, text="DROP ANGLE  (claws open on arrival)", bg=CARD, fg=ORANGE,
                 font=(FONT, 8, "bold")).grid(row=0, column=4, columnspan=3, padx=6)

        for i, (name, lift, drop) in enumerate(self.levels):
            f = tk.Frame(self.listFrame, bg=CARD_SEL if i == self.sel else INPUT,
                         highlightthickness=2,
                         highlightbackground=CYAN if i == self.sel else AXIS)
            f.pack(fill="x", pady=2)
            row = {"frame": f}
            row["dot"] = tk.Label(f, text="", bg=f["bg"], fg=YELLOW, font=(FONT, 12, "bold"),
                                  width=1)
            row["dot"].grid(row=0, column=0, padx=(6, 0), pady=6)
            row["name"] = tk.Label(f, text=f"{i + 1}  {name}", bg=f["bg"], fg=WHITE,
                                   font=(FONT, 11, "bold"), width=10, anchor="w")
            row["name"].grid(row=0, column=1, padx=(0, 6))
            for w in (f, row["dot"], row["name"]):
                w.bind("<Button-1>", lambda e, i=i: self.select(i))

            row["liftVar"] = tk.StringVar(value=f"{lift:.1f}")
            row["liftEnt"] = self._entry(f, row["liftVar"], WHITE, i, False)
            row["liftEnt"].grid(row=0, column=2, padx=2)
            row["goBtn"] = self._btn(f, "GO", lambda i=i: self.goTo(False, i), GO_BG, size=8,
                                     pad=1, width=4)
            row["goBtn"].grid(row=0, column=3, padx=2)
            row["saveBtn"] = self._btn(f, "SAVE", lambda i=i: self.saveHere(False, i), SAVE_BG,
                                       CYAN, size=8, pad=1, width=5)
            row["saveBtn"].grid(row=0, column=4, padx=(2, 12))

            if drop < 0:        # the first stage: nothing to lower to
                tk.Label(f, text="no drop angle", bg=f["bg"], fg=GRAY,
                         font=(FONT, 9, "italic")).grid(row=0, column=5, padx=2)
                row["dropVar"] = row["dropEnt"] = None
                row["dropBtn"] = self._btn(f, "OPEN CLAWS", lambda i=i: self.goTo(True, i),
                                           DROP_BG, ORANGE, size=8, pad=1, width=11)
                row["dropBtn"].grid(row=0, column=6, columnspan=2, padx=(2, 8))
            else:
                row["dropVar"] = tk.StringVar(value=f"{drop:.1f}")
                row["dropEnt"] = self._entry(f, row["dropVar"], ORANGE, i, True)
                row["dropEnt"].grid(row=0, column=5, padx=2)
                row["dropBtn"] = self._btn(f, "DROP", lambda i=i: self.goTo(True, i), DROP_BG,
                                           ORANGE, size=8, pad=1, width=5)
                row["dropBtn"].grid(row=0, column=6, padx=2)
                row["dropSave"] = self._btn(f, "SAVE", lambda i=i: self.saveHere(True, i),
                                            SAVE_BG, ORANGE, size=8, pad=1, width=5)
                row["dropSave"].grid(row=0, column=7, padx=(2, 8))
            self.rows.append(row)

    def _entry(self, parent, var, color, i, drop):
        e = tk.Entry(parent, textvariable=var, width=7, justify="right", bg=INPUT, fg=color,
                     insertbackground=WHITE, relief="flat", font=(MONO, 12),
                     highlightthickness=1, highlightbackground=AXIS, highlightcolor=CYAN)
        e.bind("<Return>", lambda ev: (self.setAngle(i, drop), self.focus_set()))
        e.bind("<FocusIn>", lambda ev: self.select(i))
        e.bind("<FocusOut>", lambda ev: self._commitIfChanged(i, drop))
        e.bind("<Escape>", lambda ev: (self._revert(i, drop), self.focus_set(), "break")[-1])
        return e

    def _current(self, i, drop):
        return self.levels[i][2 if drop else 1]

    def _revert(self, i, drop):
        row = self.rows[i]
        var = row["dropVar"] if drop else row["liftVar"]
        var.set(f"{self._current(i, drop):.1f}")

    def _commitIfChanged(self, i, drop):
        """Leaving a box with a different number in it applies it, so typing a
        value and clicking elsewhere never silently loses it."""
        if i >= len(self.rows) or i >= len(self.levels):
            return
        var = self.rows[i]["dropVar"] if drop else self.rows[i]["liftVar"]
        if var is None:
            return
        try:
            if abs(float(var.get()) - self._current(i, drop)) > 0.049:
                self.setAngle(i, drop)
        except ValueError:
            self._revert(i, drop)

    def _modified(self, i):
        if not self.baseline or i >= len(self.baseline):
            return False
        a, b = self.levels[i], self.baseline[i]
        return abs(a[1] - b[1]) > 0.049 or abs(a[2] - b[2]) > 0.049

    def _paintRows(self):
        focus = self.focus_get()
        for i, row in enumerate(self.rows):
            sel = i == self.sel
            bg = CARD_SEL if sel else INPUT
            row["frame"].config(bg=bg, highlightbackground=CYAN if sel else AXIS)
            row["dot"].config(bg=bg, text="•" if self._modified(i) else "")
            row["name"].config(bg=bg, fg=CYAN if sel else WHITE)
            for w in row["frame"].grid_slaves():
                if isinstance(w, tk.Label) and w not in (row["dot"], row["name"]):
                    w.config(bg=bg)
            name, lift, drop = self.levels[i]
            for var, ent, val in ((row["liftVar"], row["liftEnt"], lift),
                                  (row["dropVar"], row["dropEnt"], drop)):
                if var is not None and ent is not focus:     # never overwrite what's being typed
                    var.set(f"{val:.1f}")
        changed = sum(1 for i in range(len(self.levels)) if self._modified(i))
        self.dirtyLabel.config(text=f"• {changed} level{'s' if changed != 1 else ''} not in the "
                                    "source yet" if changed else "")
        self.writeBtn.config(bg=GREEN if changed else GO_BG, fg="#000000" if changed else GREEN)
        code = table_code(self.levels) if self.levels else ""
        if code != self.codeShown:
            self.codeShown = code
            self.codeText.config(state="normal")
            self.codeText.delete("1.0", "end")
            self.codeText.insert("1.0", code)
            self.codeText.config(state="disabled")

    # ── the gauge ─────────────────────────────────────────────────────────
    def drawGauge(self):
        c = self.gauge
        c.delete("all")
        w, h = max(c.winfo_width(), 100), max(c.winfo_height(), 100)
        top, bot, x = 14, h - 14, 98
        span = (bot - top) or 1
        up = self.upper or 1

        def y(deg):
            return bot - span * min(max(deg / up, 0), 1)
        c.create_rectangle(x - 7, top, x + 7, bot, fill=AXIS, outline="")
        busy = self.moving or self.dropping
        c.create_rectangle(x - 7, y(self.live), x + 7, bot, fill=ORANGE if busy else GREEN,
                           outline="")
        for i, (name, lift, drop) in enumerate(self.levels):
            sel = i == self.sel
            c.create_line(x - 18, y(lift), x + 7, y(lift), fill=CYAN if sel else "#aaaaaa",
                          width=2 if sel else 1)
            c.create_text(x - 22, y(lift), text=f"{name} {lift:.0f}", anchor="e",
                          fill=CYAN if sel else "#aaaaaa", font=(FONT, 8, "bold" if sel else ""))
            if drop >= 0:
                c.create_line(x - 7, y(drop), x + 18, y(drop), fill=ORANGE, width=2 if sel else 1)
                c.create_text(x + 22, y(drop), text=f"{drop:.0f}", anchor="w", fill=ORANGE,
                              font=(FONT, 8, "bold" if sel else ""))
        ly = y(self.live)
        c.create_polygon(x + 9, ly, x + 19, ly - 5, x + 19, ly + 5, fill=WHITE, outline="")
        c.create_text(w - 6, 6, text="drop", anchor="ne", fill=ORANGE, font=(FONT, 7))
        c.create_text(6, 6, text="lift", anchor="nw", fill="#aaaaaa", font=(FONT, 7))

    # ── sending ───────────────────────────────────────────────────────────
    def send(self, line):
        self.link._send(line)

    def note(self, text, kind="info"):
        self.message.config(text=text, fg={"info": CYAN, "ok": GREEN, "warn": ORANGE,
                                           "err": RED}[kind])

    def select(self, i):
        if not self.levels or not 0 <= i < len(self.levels):
            return
        if i != self.sel:
            self.sel = i
            self._paintRows()
            self.drawGauge()
        self.send(f"LIFT SEL {i}")

    def setAngle(self, i, drop=False):
        var = self.rows[i]["dropVar"] if drop else self.rows[i]["liftVar"]
        if var is None:
            return
        try:
            deg = float(var.get())
        except ValueError:
            self.note("that is not a number", "err")
            self._revert(i, drop)
            return
        deg = max(0.0, min(deg, self.upper))
        var.set(f"{deg:.1f}")
        self.send(f"LIFT {'SETDROP' if drop else 'SET'} {i} {deg:.1f}")

    def goTo(self, drop=False, i=None):
        i = self.sel if i is None else i
        self.setAngle(i, drop)          # a typed-but-unapplied number is used, not ignored
        self.select(i)
        self.send(f"LIFT {'GODROP' if drop else 'GO'} {i}")

    def saveHere(self, drop=False, i=None):
        i = self.sel if i is None else i
        if drop and self.levels and self.levels[i][2] < 0:
            self.note(f"{self.levels[i][0]} has no drop angle: dropping there just opens the "
                      "claws", "warn")
            return
        self.select(i)
        self.send(f"LIFT {'SAVEDROP' if drop else 'SAVE'} {i}")

    def jog(self, sign):
        try:
            step = abs(float(self.step.get()))
        except ValueError:
            self.note("the jog step is not a number", "err")
            return
        self.send(f"LIFT JOG {sign * step:.1f}")

    def allClaws(self):
        # all open -> shut them; otherwise open them (0 = extend, 1 = retract)
        both_open = claw_is_open(self.claw1) and claw_is_open(self.claw2)
        extend = both_open == CLAW_OPEN_RETRACTED
        self.send(f"LIFT CLAW 0 {0 if extend else 1}")

    def _markStep(self):
        try:
            cur = float(self.step.get())
        except ValueError:
            cur = None
        for v, b in self.stepBtns.items():
            b.config(bg=CARD_SEL if cur == v else BTN, fg=CYAN if cur == v else WHITE)

    # ── state from the brain ──────────────────────────────────────────────
    def applyState(self, state):
        self.onBrainScreen = state["screen"] == "LIFT_CAL"
        lift = state.get("lift")
        if not (self.onBrainScreen and lift):
            self.chipBrain.config(text=f"brain: {state['screen']}", fg=ORANGE)
            self.note("Tap LIFT CAL on the brain to calibrate from here.", "warn")
            return
        self.chipBrain.config(text="brain: LIFT CAL", fg=GREEN)
        self.live, self.moving, self.bumper = lift["live"], lift["moving"], lift["bumper"]
        self.dropping = lift.get("dropping", False)
        self.upper = lift["upper"]
        self.claw1, self.claw2 = lift["claw"], lift["claw2"]
        newSel = lift["sel"]

        names = [lv[0] for lv in lift["levels"]]
        shape = [(lv[0], lv[2] >= 0) for lv in lift["levels"]]
        first = not self.levels
        oldShape = [(lv[0], lv[2] >= 0) for lv in self.levels]
        self.levels = [tuple(lv) for lv in lift["levels"]]
        self.sel = newSel
        if self.baseline is None:
            self.baseline = list(self.levels)
        if first or shape != oldShape or len(self.rows) != len(names):
            self._rebuildRows()
        self._paintRows()

        self.liveLabel.config(text=f"{self.live:.1f}", fg=ORANGE if self.moving else GREEN)
        word = "DROPPING" if self.dropping else "MOVING" if self.moving else "READY"
        self.chipState.config(text=word, fg=ORANGE if self.moving or self.dropping else GREEN)
        self.chipBumper.config(text=f"bumper {'ON' if self.bumper else 'off'}",
                               fg=GREEN if self.bumper else GRAY)
        for b, name, ext in ((self.clawBtns[0], "CLAW 1", self.claw1),
                             (self.clawBtns[1], "CLAW 2", self.claw2)):
            o = claw_is_open(ext)
            b.config(text=f"{name}  {'OPEN' if o else 'SHUT'}", bg=GO_BG if o else BTN,
                     fg=GREEN if o else GRAY)
        both = claw_is_open(self.claw1) and claw_is_open(self.claw2)
        self.clawBtns[2].config(text="SHUT BOTH" if both else "OPEN BOTH")
        self.drawGauge()
        req = lift.get("saveReq")
        if req is not None:
            if self.lastSaveReq is None:
                self.lastSaveReq = req          # first state: just remember the counter
            elif req != self.lastSaveReq:
                self.lastSaveReq = req          # SAVE CODE was tapped on the brain
                self.send(f"LIFT SAVED {1 if self.writeSource() else 0}")
                return
        if lift["msg"]:
            warn = "NOT" in lift["msg"] or "timed out" in lift["msg"] or "no drop" in lift["msg"]
            self.note(lift["msg"], "warn" if warn else "info")

    # ── saving ────────────────────────────────────────────────────────────
    def copyCode(self):
        if not self.levels:
            return
        self.clipboard_clear()
        self.clipboard_append(table_code(self.levels))
        self.note("table copied -- paste it between the LIFT-LEVELS markers in "
                  "src/lift_calibration.cpp", "ok")

    def writeSource(self):
        """Replace the table between the markers in src/lift_calibration.cpp.
        Returns True when the file was written."""
        if not self.levels:
            self.note("nothing to save yet: the brain hasn't sent the levels", "warn")
            return False
        try:
            with open(SOURCE, encoding="utf-8", newline="") as f:
                text = f.read()
            pat = re.compile(re.escape(BEGIN) + r".*?" + re.escape(END), re.S)
            if not pat.search(text):
                raise ValueError("LIFT-LEVELS markers not found")
            nl = "\r\n" if "\r\n" in text else "\n"
            block = BEGIN + nl + table_code(self.levels).replace("\n", nl) + END
            text = pat.sub(lambda m: block, text, count=1)
            with open(SOURCE, "w", encoding="utf-8", newline="") as f:
                f.write(text)
        except (OSError, ValueError) as e:
            self.note(f"could not write the code: {e}", "err")
            messagebox.showerror("Save to code", str(e), parent=self)
            return False
        self.baseline = list(self.levels)
        self._paintRows()
        self.note("saved into src/lift_calibration.cpp -- rebuild and upload to keep these angles",
                  "ok")
        return True


# ─── Stand-alone demo ──────────────────────────────────────────────────────
class _FakeLink:
    def __init__(self):
        self.levels = [["GROUND", 0.0, -1.0], ["LOW", 120.0, 100.0], ["MID", 240.0, 220.0],
                       ["HIGH", 360.0, 340.0], ["MAX", 500.0, 460.0],
                       ["PIN", 60.0, 60.0]]
        self.sel, self.live, self.msg = 0, 0.0, "fake lift"
        self.claw, self.claw2, self.dropping = True, True, False

    def _send(self, line):
        print("->", line)
        f = line.split()
        if f[1] == "SEL":
            self.sel = int(f[2])
        elif f[1] == "SET":
            self.levels[int(f[2])][1] = float(f[3])
        elif f[1] == "SETDROP" and self.levels[int(f[2])][2] >= 0:
            self.levels[int(f[2])][2] = float(f[3])
        elif f[1] == "GO":
            self.live = self.levels[int(f[2])][1]
        elif f[1] == "GODROP":
            lv = self.levels[int(f[2])]
            if lv[2] >= 0:
                self.live = lv[2]
            self.claw = self.claw2 = not CLAW_OPEN_RETRACTED      # the claws open
            self.msg = f"{lv[0]}: claws open"
        elif f[1] == "SAVE":
            self.levels[int(f[2])][1] = self.live
        elif f[1] == "SAVEDROP" and self.levels[int(f[2])][2] >= 0:
            self.levels[int(f[2])][2] = self.live
        elif f[1] == "JOG":
            self.live = max(0.0, min(500.0, self.live + float(f[2])))
        elif f[1] == "HOME":
            self.live = 0.0
        elif f[1] == "CLAW":
            which, st = int(f[2]), int(f[3])
            for i, name in ((1, "claw"), (2, "claw2")):
                if which in (0, i):
                    setattr(self, name, (not getattr(self, name)) if st == 2 else st == 0)

    def state(self):
        return {"screen": "LIFT_CAL", "lift": {
            "sel": self.sel, "live": self.live, "moving": False, "bumper": self.live == 0,
            "upper": 500.0, "claw": self.claw, "claw2": self.claw2, "dropping": self.dropping,
            "levels": [tuple(lv) for lv in self.levels], "msg": self.msg}}


def _demo():
    root = tk.Tk()
    root.withdraw()
    link = _FakeLink()
    win = LiftCalWindow(root, link)
    win.protocol("WM_DELETE_WINDOW", root.destroy)

    def tick():
        win.applyState(link.state())
        root.after(100, tick)
    tick()
    root.mainloop()


if __name__ == "__main__":
    _demo()
