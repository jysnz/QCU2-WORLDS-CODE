#!/usr/bin/env python3
"""
Distance Calibration -- the laptop half of the brain's DISTANCE screen.

Opened automatically by tools/remote_touch.py when the brain enters that screen
(HOME -> DISTANCE). Three VL53L0X sensors (S0, S1, S2 on the ESP32) are read on
smart port 20; this window shows what each side (BACK, LEFT, RIGHT) measures and
lets you choose which sensor is on which side.

HOW TO USE
    1. Pick the sensor for each side in its card (S0 / S1 / S2). Picking one that
       another side already has swaps the two, so every sensor is on exactly one side.
    2. Press SAVE. Until then the change is only a preview (orange "unsaved");
       the robot code keeps using the last saved assignment.
    3. To zero a side's reading: click its card, hold a target at a known distance,
       set the REFERENCE (mm) and press SET REF. CLEAR removes the offset.

KEYS (when you are not typing in a box)
    1 / 2 / 3   select BACK / LEFT / RIGHT      R   SET REF      C   CLEAR
    Ctrl+S      SAVE

RUN ON ITS OWN
    python tools/distance_calibration.py   opens the window against a fake brain.

WIRE FORMAT (see src/distance_diag.cpp)
    robot -> laptop  RUI|DISTANCE|...|DIST:<sel>,<ref>,<state>,<dirty>,<saved x3>,<pending x3>,<offset x3>,<raw x3>,<fps>
    laptop -> robot  DIST SEL i | DIST REF mm | DIST SETREF | DIST CLEAR | DIST SRC side sensor | DIST SAVE
"""

import os
import sys
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from pid_tuner import AXIS, BG, CARD, CYAN, GRAY, GREEN, ORANGE, RED, WHITE  # noqa: E402

SIDES = ("BACK", "LEFT", "RIGHT")
STATES = ("NO DATA", "BAD DATA", "STALE", "SENSOR NOT FOUND", "OUT OF RANGE", "OK")  # == State on the brain
STATE_COLOR = (RED, RED, ORANGE, RED, ORANGE, GREEN)
MAX_RANGE = 1200
BTN = "#22222B"
INPUT = "#0f1218"
FONT = "Segoe UI"
MONO = "Consolas"


class DistanceWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.title("Distance Calibration -- V5")
        self.configure(bg=BG)
        self.sel = 0
        self.pend = [0, 1, 2]
        self.saved = [0, 1, 2]
        self.offset = [0, 0, 0]
        self.raw = [-2, -2, -2]
        self.dirty = False
        self.onBrainScreen = False
        self.refVar = tk.StringVar(value="100")
        self.srcVars = [tk.IntVar(value=i) for i in range(3)]
        self.protocol("WM_DELETE_WINDOW", self.withdraw)
        self._build()
        self._bindKeys()
        self.update_idletasks()
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())

    # -- layout ---------------------------------------------------------------
    def _btn(self, parent, text, cmd, bg=BTN, fg=WHITE, size=10, width=None):
        return tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg, activebackground="#2d2d38",
                         activeforeground=fg, disabledforeground="#555", relief="flat",
                         font=(FONT, size, "bold"), cursor="hand2", width=width, pady=6)

    def _build(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=12, pady=(12, 6))
        tk.Label(top, text="DISTANCE", bg=BG, fg=CYAN, font=(FONT, 16, "bold")).pack(side="left")
        self.chipBrain = tk.Label(top, text="brain: --", bg=BG, fg=GRAY, font=(FONT, 9))
        self.chipBrain.pack(side="right")
        self.chipState = tk.Label(top, text="NO DATA", bg=RED, fg="#000", font=(FONT, 10, "bold"),
                                  padx=10, pady=2)
        self.chipState.pack(side="right", padx=10)

        cards = tk.Frame(self, bg=BG)
        cards.pack(fill="x", padx=12)
        self.cards = []
        for i, side in enumerate(SIDES):
            cards.columnconfigure(i, weight=1, uniform="c")
            frame = tk.Frame(cards, bg=CARD, highlightthickness=2, highlightbackground=AXIS, cursor="hand2")
            frame.grid(row=0, column=i, padx=4, sticky="nsew")
            head = tk.Frame(frame, bg=CARD)
            head.pack(fill="x", padx=10, pady=(8, 0))
            title = tk.Label(head, text=side, bg=CARD, fg=WHITE, font=(FONT, 12, "bold"))
            title.pack(side="left")
            badge = tk.Label(head, text="", bg=CARD, fg=GRAY, font=(MONO, 9))
            badge.pack(side="right")
            value = tk.Label(frame, text="--", bg=CARD, fg=GRAY, font=(MONO, 34, "bold"))
            value.pack(anchor="w", padx=10)
            unit = tk.Label(frame, text="mm", bg=CARD, fg=GRAY, font=(FONT, 9))
            unit.pack(anchor="w", padx=12)
            bar = tk.Canvas(frame, height=10, bg=AXIS, highlightthickness=0)
            bar.pack(fill="x", padx=10, pady=6)
            tag = tk.Label(frame, text="", bg=CARD, fg=GRAY, font=(FONT, 9, "bold"))
            tag.pack(anchor="w", padx=10)
            off = tk.Label(frame, text="", bg=CARD, fg=GRAY, font=(MONO, 9))
            off.pack(anchor="w", padx=10)

            pick = tk.Frame(frame, bg=CARD)
            pick.pack(fill="x", padx=10, pady=(6, 10))
            tk.Label(pick, text="SENSOR", bg=CARD, fg=GRAY, font=(FONT, 8, "bold")).pack(side="left", padx=(0, 6))
            for s in range(3):
                tk.Radiobutton(pick, text=f"S{s}", variable=self.srcVars[i], value=s, indicatoron=False,
                               command=lambda i=i, s=s: self.setSource(i, s), bg=BTN, fg=WHITE,
                               selectcolor="#14405a", activebackground="#2d2d38", activeforeground=WHITE,
                               relief="flat", font=(FONT, 9, "bold"), width=4, cursor="hand2").pack(side="left", padx=2)
            for w in (frame, head, title, value, unit, tag, off, bar):
                w.bind("<Button-1>", lambda e, i=i: self.select(i))
            self.cards.append({"frame": frame, "title": title, "badge": badge, "value": value, "bar": bar,
                               "tag": tag, "off": off})

        cal = tk.Frame(self, bg=CARD, highlightthickness=1, highlightbackground=AXIS)
        cal.pack(fill="x", padx=16, pady=10)
        self.calTitle = tk.Label(cal, text="CALIBRATE BACK", bg=CARD, fg=CYAN, font=(FONT, 9, "bold"))
        self.calTitle.pack(anchor="w", padx=10, pady=(8, 2))
        row = tk.Frame(cal, bg=CARD)
        row.pack(fill="x", padx=10, pady=(0, 10))
        tk.Label(row, text="REFERENCE", bg=CARD, fg=GRAY, font=(FONT, 8, "bold")).pack(side="left")
        self.refEntry = tk.Entry(row, textvariable=self.refVar, width=6, bg=INPUT, fg=WHITE, insertbackground=WHITE,
                                 relief="flat", font=(MONO, 12), justify="right")
        self.refEntry.pack(side="left", padx=6)
        tk.Label(row, text="mm", bg=CARD, fg=GRAY, font=(FONT, 9)).pack(side="left")
        self.refEntry.bind("<Return>", lambda e: self.sendRef())
        self.refEntry.bind("<FocusOut>", lambda e: self.sendRef())
        self._btn(row, "SET REF", self.setRef, bg="#143a1e", fg=GREEN).pack(side="left", padx=(16, 4))
        self._btn(row, "CLEAR", self.clearOffset).pack(side="left", padx=4)
        self.saveBtn = self._btn(row, "SAVE", self.save, bg=BTN, fg=GRAY, size=11, width=10)
        self.saveBtn.pack(side="right")

        self.message = tk.Label(self, text="", bg=BG, fg=CYAN, font=(FONT, 9), anchor="w")
        self.message.pack(fill="x", padx=16, pady=(0, 10))

    def _bindKeys(self):
        def guard(fn):
            return lambda e: None if isinstance(self.focus_get(), tk.Entry) else fn()
        for i in range(3):
            self.bind(str(i + 1), guard(lambda i=i: self.select(i)))
        self.bind("r", guard(self.setRef))
        self.bind("c", guard(self.clearOffset))
        self.bind("<Control-s>", lambda e: self.save())

    # -- sending --------------------------------------------------------------
    def send(self, line):
        self.link._send(line)

    def note(self, text, kind="info"):
        self.message.config(text=text, fg={"info": CYAN, "ok": GREEN, "warn": ORANGE, "err": RED}[kind])

    def select(self, i):
        self.sel = i
        self.send(f"DIST SEL {i}")
        self._paint()

    def sendRef(self):
        try:
            mm = max(20, min(1000, int(float(self.refVar.get()))))
        except ValueError:
            return False
        self.refVar.set(str(mm))
        self.send(f"DIST REF {mm}")
        return True

    def setRef(self):
        if not self.sendRef():
            self.note("Reference must be a number of millimetres.", "err")
            return
        self.send(f"DIST SEL {self.sel}")
        self.send("DIST SETREF")
        self.note(f"{SIDES[self.sel]} set to {self.refVar.get()} mm.", "ok")

    def clearOffset(self):
        self.send(f"DIST SEL {self.sel}")
        self.send("DIST CLEAR")
        self.note(f"{SIDES[self.sel]} offset cleared.", "info")

    def setSource(self, side, sensor):
        self.send(f"DIST SRC {side} {sensor}")
        # preview the swap right away; the brain's next state line confirms it
        other = next((j for j in range(3) if j != side and self.pend[j] == sensor), None)
        if other is not None:
            self.pend[other] = self.pend[side]
        self.pend[side] = sensor
        self.dirty = self.pend != self.saved
        self._paint()

    def save(self):
        if not self.onBrainScreen:
            self.note("Open DISTANCE on the brain first.", "warn")
            return
        self.send("DIST SAVE")
        self.saved = list(self.pend)
        self.dirty = False
        names = ", ".join(f"{SIDES[i]}=S{self.pend[i]}" for i in range(3))
        self.note(f"Saved: {names}", "ok")
        self._paint()

    # -- state from the brain ---------------------------------------------------
    def applyState(self, state):
        self.onBrainScreen = state["screen"] == "DISTANCE"
        d = state.get("dist")
        if not (self.onBrainScreen and d):
            self.chipBrain.config(text=f"brain: {state['screen']}", fg=ORANGE)
            self.note("Tap DISTANCE on the brain to calibrate from here.", "warn")
            return
        self.chipBrain.config(text="brain: DISTANCE", fg=GREEN)
        self.sel = d["sel"]
        self.pend, self.saved, self.offset, self.raw = d["pend"], d["saved"], d["offset"], d["raw"]
        self.dirty = d["dirty"]
        if self.focus_get() is not self.refEntry:
            self.refVar.set(str(d["ref"]))
        for i in range(3):
            self.srcVars[i].set(self.pend[i])
        self.chipState.config(text=STATES[d["state"]], bg=STATE_COLOR[d["state"]])
        self._paint()

    def _paint(self):
        live = self.onBrainScreen and self.chipState.cget("text") in ("OK", "OUT OF RANGE", "SENSOR NOT FOUND")
        for i, c in enumerate(self.cards):
            raw = self.raw[self.pend[i]]
            valid = live and raw >= 0
            shown = max(0, raw + self.offset[i]) if valid else None
            selected = i == self.sel
            c["frame"].config(highlightbackground=CYAN if selected else AXIS)
            c["title"].config(fg=CYAN if selected else WHITE)
            c["badge"].config(text=f"S{self.pend[i]}", fg=ORANGE if self.pend[i] != self.saved[i] else GRAY)
            c["value"].config(text="--" if shown is None else str(shown), fg=WHITE if valid else GRAY)
            bar = c["bar"]
            bar.delete("all")
            if valid:
                w = bar.winfo_width() * min(shown, MAX_RANGE) / MAX_RANGE
                bar.create_rectangle(0, 0, w, 12, fill=GREEN, outline="")
            if valid:
                tag, color = "OK", GREEN
            elif live and raw == -1:
                tag, color = "OUT OF RANGE", ORANGE
            elif live and raw == -2:
                tag, color = "NO SENSOR", RED
            else:
                tag, color = "NO DATA", GRAY
            c["tag"].config(text=tag, fg=color)
            c["off"].config(text=f"offset {self.offset[i]:+d} mm", fg=ORANGE if self.offset[i] else GRAY)
        self.calTitle.config(text=f"CALIBRATE {SIDES[self.sel]}: hold a target at the reference distance, then SET REF")
        if self.dirty:
            self.saveBtn.config(text="SAVE", bg=ORANGE, fg="#000")
        else:
            self.saveBtn.config(text="SAVED", bg=BTN, fg=GRAY)


# --- Stand-alone demo --------------------------------------------------------
class _FakeLink:
    def __init__(self):
        self.sel, self.ref = 0, 100
        self.pend, self.saved, self.offset = [0, 1, 2], [0, 1, 2], [0, 0, 0]
        self.raw = [250, 480, -1]

    def _send(self, line):
        print("->", line)
        f = line.split()
        if f[1] == "SEL":
            self.sel = int(f[2])
        elif f[1] == "REF":
            self.ref = int(f[2])
        elif f[1] == "SETREF":
            self.offset[self.sel] = self.ref - self.raw[self.pend[self.sel]]
        elif f[1] == "CLEAR":
            self.offset[self.sel] = 0
        elif f[1] == "SRC":
            side, src = int(f[2]), int(f[3])
            for j in range(3):
                if j != side and self.pend[j] == src:
                    self.pend[j] = self.pend[side]
            self.pend[side] = src
        elif f[1] == "SAVE":
            self.saved = list(self.pend)

    def state(self):
        return {"screen": "DISTANCE", "dist": {
            "sel": self.sel, "ref": self.ref, "state": 5, "dirty": self.pend != self.saved,
            "saved": self.saved, "pend": self.pend, "offset": self.offset, "raw": self.raw, "fps": 20.0}}


def _demo():
    root = tk.Tk()
    root.withdraw()
    link = _FakeLink()
    win = DistanceWindow(root, link)
    win.protocol("WM_DELETE_WINDOW", root.destroy)

    def tick():
        win.applyState(link.state())
        root.after(100, tick)
    tick()
    root.mainloop()


if __name__ == "__main__":
    _demo()
