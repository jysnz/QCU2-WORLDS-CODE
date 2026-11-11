#!/usr/bin/env python3
"""
Decel Tune -- the laptop half of the brain's DECEL TUNE screen.

Opened automatically by tools/remote_touch.py when the brain enters that screen
(HOME -> DECEL TUNE). It tunes the smooth deceleration of moveToPoint / Grab Pin
steps: the robot drives at SPEED until it is RANGE inches from the target, then
finishes the last stretch at END (or ramps down to END in STEPS stages).

HOW TO TUNE
    1. Put the robot with clear floor ahead of it.
    2. Set DIST / SPEED / RANGE / END / STEPS (type a number and press Enter, or
       use the - / + buttons). The graph shows the speed the drive will follow.
    3. RUN drives DIST forward from where the robot stands; RETURN drives back to
       the start with the same settings; STOP cancels. The result line says how
       far past (+) or short of (-) the target the robot stopped.
       Past the target: raise RANGE or lower END.   Too slow to arrive: the reverse.
    4. Put the final RANGE / END into the planner's decel range / decel speed
       fields (Copy values puts them on the clipboard).

KEYS (when you are not typing in a box)
    R run   B return   Esc / Space stop

RUN ON ITS OWN
    python tools/decel_tune.py   opens the window against a fake robot.

WIRE FORMAT (see include/decel_tune.hpp)
    robot -> laptop  RUI|DECEL_TUNE|...|DECEL:<dist>,<speed>,<range>,<end>,<steps>,<own>,<running>,<haveStart>|MSG:<result>
    laptop -> robot  DECEL SET i value   (i: 0 dist, 1 speed, 2 range, 3 end, 4 steps, 5 own finish)
                     | DECEL RUN | DECEL RETURN | DECEL STOP
"""

import os
import sys
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from pid_tuner import AXIS, BG, CARD, CYAN, DIM, GRAY, GREEN, ORANGE, RED, WHITE  # noqa: E402

INPUT = "#0f1218"
BTN = "#22222B"
GO_BG = "#143a1e"
STOP_BG = "#3a1414"
FONT = "Segoe UI"
MONO = "Consolas"

# (name, label, lo, hi, step, unit) -- must match params[] in src/decel_tune.cpp
PARAMS = (
    ("dist", "DIST", 12, 120, 6, "in"),
    ("speed", "SPEED", 40, 127, 10, ""),
    ("range", "RANGE", 0, 40, 2, "in"),
    ("end", "END", 15, 120, 5, ""),
    ("steps", "STEPS", 0, 8, 1, "0 = one step, 2-8 = ramp"),
    ("own", "FINISH", 0, 1, 1, "1 = own approach, 0 = LemLib's"),
)


def speed_profile(dist, speed, rng, end, steps):
    """[(inches driven, speed)] the drive follows: full speed, then the slow-down over the last
    `rng` inches (one step to `end`, or a ramp of `steps` stages)."""
    if rng <= 0 or end >= speed or dist <= rng * 1.25:
        return [(0, speed), (dist, speed)]
    start = dist - rng
    pts = [(0, speed), (start, speed)]
    if steps >= 2:
        for i in range(1, steps + 1):
            a = start + rng * (i - 1) / steps
            b = start + rng * i / steps
            v = speed - (speed - end) * i / steps
            pts += [(a, v), (b, v)]
    else:
        pts += [(start, end), (dist, end)]
    return pts


class DecelTuneWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.title("Decel Tune -- V5")
        self.configure(bg=BG)
        self.values = {p[0]: float(d) for p, d in zip(PARAMS, (48, 127, 10, 40, 0, 1))}
        self.vars = {}
        self.entries = {}
        self.running = False
        self.haveStart = False
        self.onBrainScreen = False
        self.protocol("WM_DELETE_WINDOW", self.withdraw)
        self._build()
        self.bind("<Escape>", lambda e: self.stop())
        self.bind("<space>", lambda e: self.stop())
        self.bind("r", lambda e: self._key(self.run))
        self.bind("b", lambda e: self._key(self.back))
        self.update_idletasks()
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())

    def _key(self, fn):
        if not isinstance(self.focus_get(), tk.Entry):
            fn()

    # -- layout --
    def _btn(self, parent, text, cmd, bg=BTN, fg=WHITE, size=10, width=None, pad=6):
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
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=12, pady=(10, 6))
        tk.Label(top, text="DECEL TUNE", bg=BG, fg=CYAN, font=(FONT, 13, "bold")).pack(side="left")
        try:
            from reconnect_button import add_reconnect_button
            add_reconnect_button(self, top, side="right", padx=(6, 0))
        except ImportError:
            pass
        self.chipBrain = tk.Label(top, text="brain: waiting", bg=CARD, fg=ORANGE,
                                  font=(FONT, 9, "bold"), padx=10, pady=3)
        self.chipBrain.pack(side="right", padx=(6, 0))
        self.chipState = tk.Label(top, text="READY", bg=CARD, fg=GREEN, font=(FONT, 9, "bold"),
                                  padx=10, pady=3)
        self.chipState.pack(side="right")

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=12)

        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="y")
        card = self._card(left, "SETTINGS")
        card.pack(fill="x")
        grid = tk.Frame(card, bg=CARD)
        grid.pack(padx=10, pady=(0, 10))
        for r, (name, label, lo, hi, step, unit) in enumerate(PARAMS):
            tk.Label(grid, text=label, bg=CARD, fg=GRAY, font=(FONT, 9, "bold"), width=6,
                     anchor="w").grid(row=r, column=0, pady=3)
            self._btn(grid, "-", lambda n=name: self.nudge(n, -1), size=11, width=3, pad=1).grid(
                row=r, column=1, padx=(0, 3))
            var = tk.StringVar(value=f"{self.values[name]:.0f}")
            self.vars[name] = var
            e = tk.Entry(grid, textvariable=var, width=6, justify="center", bg=INPUT, fg=WHITE,
                         insertbackground=WHITE, relief="flat", font=(MONO, 12))
            e.grid(row=r, column=2, ipady=3)
            self.entries[name] = e
            e.bind("<Return>", lambda ev, n=name: self.commit(n))
            e.bind("<FocusOut>", lambda ev, n=name: self.commit(n))
            self._btn(grid, "+", lambda n=name: self.nudge(n, +1), size=11, width=3, pad=1).grid(
                row=r, column=3, padx=(3, 8))
            tk.Label(grid, text=unit, bg=CARD, fg=DIM, font=(FONT, 8)).grid(
                row=r, column=4, sticky="w")

        acts = tk.Frame(left, bg=BG)
        acts.pack(fill="x", pady=(8, 0))
        self.runBtn = self._btn(acts, "RUN  (R)", self.run, bg=GO_BG, fg=GREEN, size=11, pad=8)
        self.runBtn.pack(fill="x")
        self.backBtn = self._btn(acts, "RETURN  (B)", self.back, size=10, pad=6)
        self.backBtn.pack(fill="x", pady=(4, 0))
        self._btn(acts, "STOP  (Esc)", self.stop, bg=STOP_BG, fg=RED, size=10, pad=6).pack(
            fill="x", pady=(4, 0))

        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(12, 0))
        gcard = self._card(right, "SPEED PROFILE  (speed vs. inches driven)")
        gcard.pack(fill="both", expand=True)
        self.graph = tk.Canvas(gcard, width=420, height=230, bg=INPUT, highlightthickness=0)
        self.graph.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.graph.bind("<Configure>", lambda e: self.drawGraph())

        rcard = self._card(right, "LAST RESULT")
        rcard.pack(fill="x", pady=(8, 0))
        self.resultLabel = tk.Label(rcard, text="no run yet", bg=CARD, fg=GRAY,
                                    font=(MONO, 11, "bold"), anchor="w")
        self.resultLabel.pack(fill="x", padx=10, pady=(0, 4))
        self.tip = tk.Label(rcard, bg=CARD, fg=DIM, font=(FONT, 8), anchor="w", justify="left",
                            text="Past the target: raise RANGE or lower END.   "
                                 "Slow to arrive: the reverse.")
        self.tip.pack(fill="x", padx=10, pady=(0, 6))

        bottom = tk.Frame(self, bg=BG)
        bottom.pack(fill="x", padx=12, pady=(8, 10))
        self.planLabel = tk.Label(bottom, bg=BG, fg=GREEN, font=(MONO, 10), anchor="w")
        self.planLabel.pack(side="left")
        self._btn(bottom, "Copy values", self.copyValues, size=9, pad=3).pack(side="right")
        self.message = tk.Label(self, text="", bg=BG, fg=CYAN, font=(FONT, 9), anchor="w")
        self.message.pack(fill="x", padx=12, pady=(0, 8))
        self.refresh()

    # -- values --
    def refresh(self):
        v = self.values
        self.planLabel.config(
            text=f"planner: decel range {v['range']:.0f}   decel speed {v['end']:.0f}"
                 + (f"   ({v['steps']:.0f}-step ramp)" if v["steps"] >= 2 else ""))
        self.drawGraph()

    def send(self, line):
        try:
            self.link._send(line)
        except Exception as e:  # noqa: BLE001
            self.note(f"send failed: {e}", RED)

    def note(self, text, color=CYAN):
        self.message.config(text=text, fg=color)

    def commit(self, name):
        i = [p[0] for p in PARAMS].index(name)
        _, _, lo, hi, _, _ = PARAMS[i]
        try:
            v = float(self.vars[name].get())
        except ValueError:
            self.vars[name].set(f"{self.values[name]:.0f}")
            return
        v = max(lo, min(hi, v))
        if name == "steps" and v == 1:
            v = 0
        self.values[name] = v
        self.vars[name].set(f"{v:.0f}")
        self.send(f"DECEL SET {i} {v:.0f}")
        self.refresh()

    def nudge(self, name, d):
        i = [p[0] for p in PARAMS].index(name)
        _, _, lo, hi, step, _ = PARAMS[i]
        v = max(lo, min(hi, self.values[name] + d * step))
        if name == "steps" and v == 1:
            v = 2 if d > 0 else 0
        self.values[name] = v
        self.vars[name].set(f"{v:.0f}")
        self.send(f"DECEL SET {i} {v:.0f}")
        self.refresh()

    def run(self):
        self.send("DECEL RUN")
        self.note("running -- keep clear of the robot", ORANGE)

    def back(self):
        self.send("DECEL RETURN")

    def stop(self):
        self.send("DECEL STOP")
        self.note("stop sent", ORANGE)

    def copyValues(self):
        v = self.values
        text = f"{v['range']:.2f}, {v['end']:.0f}"
        self.clipboard_clear()
        self.clipboard_append(text)
        self.note(f"copied decelRange, decelSpeed = {text}", GREEN)

    # -- graph --
    def drawGraph(self):
        c = self.graph
        c.delete("all")
        w, h = max(c.winfo_width(), 200), max(c.winfo_height(), 120)
        l, r, t, b = 40, w - 14, 14, h - 26
        v = self.values
        dist, speed = v["dist"], v["speed"]
        c.create_rectangle(l, t, r, b, outline=AXIS)

        def X(inches):
            return l + (r - l) * inches / max(dist, 1)

        def Y(spd):
            return b - (b - t) * spd / 127.0
        for s in (0, 40, 80, 127):
            c.create_line(l - 3, Y(s), l, Y(s), fill=AXIS)
            c.create_text(l - 6, Y(s), text=str(s), anchor="e", fill=GRAY, font=(FONT, 7))
        c.create_text(l, b + 12, text="0 in", anchor="w", fill=GRAY, font=(FONT, 7))
        c.create_text(r, b + 12, text=f"{dist:.0f} in", anchor="e", fill=GRAY, font=(FONT, 7))
        pts = speed_profile(dist, speed, v["range"], v["end"], int(v["steps"]))
        flat = []
        for x, s in pts:
            flat += [X(x), Y(s)]
        c.create_line(*flat, fill=GREEN, width=2)
        if v["range"] > 0 and v["end"] < speed and dist > v["range"] * 1.25:
            xs = X(dist - v["range"])
            c.create_line(xs, t, xs, b, fill=ORANGE, dash=(3, 3))
            c.create_text(xs + 4, t + 8, text="slow-down starts", anchor="w", fill=ORANGE,
                          font=(FONT, 7))

    # -- state from the brain --
    def applyState(self, state):
        self.onBrainScreen = state["screen"] == "DECEL_TUNE"
        d = state.get("decel")
        if not (self.onBrainScreen and d):
            self.chipBrain.config(text=f"brain: {state['screen']}", fg=ORANGE)
            self.note("Tap DECEL TUNE on the brain to tune from here.", ORANGE)
            return
        self.chipBrain.config(text="brain: DECEL TUNE", fg=GREEN)
        self.running, self.haveStart = d["running"], d["haveStart"]
        self.chipState.config(text="RUNNING" if self.running else "READY",
                              fg=ORANGE if self.running else GREEN)
        focused = self.focus_get()
        for (name, *_), val in zip(PARAMS, d["values"]):
            self.values[name] = val
            if focused is not self.entries[name]:      # don't overwrite a box being typed in
                self.vars[name].set(f"{val:.0f}")
        self.backBtn.config(state="normal" if self.haveStart and not self.running else "disabled")
        self.runBtn.config(state="disabled" if self.running else "normal")
        msg = d.get("msg", "")
        if msg:
            color = GRAY
            if "past" in msg:
                color = RED
            elif "short" in msg:
                color = ORANGE
            elif msg in ("running...", "returning..."):
                color = CYAN
            self.resultLabel.config(text=msg, fg=color)
        self.refresh()
        if not self.running:
            self.note("")


# ─── Stand-alone demo ──────────────────────────────────────────────────────
class _FakeLink:
    def __init__(self):
        self.vals = [48.0, 127.0, 10.0, 40.0, 0.0, 1.0]
        self.msg, self.have = "no run yet", False

    def _send(self, line):
        print("->", line)
        f = line.split()
        if f[1] == "SET":
            self.vals[int(f[2])] = float(f[3])
        elif f[1] == "RUN":
            self.have, self.msg = True, "1.42s   past 0.6 in   (error 0.6 in)"
        elif f[1] == "RETURN":
            self.msg = "1.38s   short 0.3 in   (error 0.3 in)"

    def state(self):
        return {"screen": "DECEL_TUNE", "decel": {
            "values": list(self.vals), "running": False, "haveStart": self.have, "msg": self.msg}}


def _demo():
    root = tk.Tk()
    root.withdraw()
    link = _FakeLink()
    win = DecelTuneWindow(root, link)
    win.protocol("WM_DELETE_WINDOW", root.destroy)

    def tick():
        win.applyState(link.state())
        root.after(200, tick)
    tick()
    root.mainloop()


if __name__ == "__main__":
    _demo()
