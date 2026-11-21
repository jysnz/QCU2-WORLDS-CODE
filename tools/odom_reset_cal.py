#!/usr/bin/env python3
"""
Odom Reset -- the laptop half of the brain's ODOM RESET screen.

Opened automatically by tools/remote_touch.py when the brain enters that screen
(HOME -> ODOM RESET). It is a bench test for odomReset(): the wall sensors work out
where the robot is, and the reset moves the odometry pose onto that.

THE MAP (the same field render as the path planner: tools/field.png, +Y up, +X right, 144 in square)
    cyan robot        where odometry thinks the robot is (the line is its heading)
    green cross       where the wall sensors say it is (needs heading within 10 deg of a wall)
    orange dot        where each sensor beam lands (a dot short of the wall = something in the way)
    pink diamond      the TRUE position you measured with a tape (type it in, or right-click the map)
    grey robot        where odometry was before the last reset
    left-click map    moves odometry there (heading kept) to simulate drift, so RESET XY has something to fix

HOW TO CHECK X/Y
    1. Put the robot on a spot you measured with a tape and type that X / Y in TRUE POSITION.
    2. Left-click the map a little way (under 4 in) from it to put odometry off, or just let it be.
    3. RESET XY. Odometry should land on the tape position: the ERROR panel shows the distance
       in inches and turns green at 0.5 in or less. Repeat from a few spots; the LOG keeps every try.
    4. Put a box in front of a sensor and reset: that reading should be rejected and nothing should move.

HOW TO CHECK THE HEADING
    Turn the robot a known 5-8 degrees and press RESET + HDG. The heading should be near the wall angle
    plus your tilt (the IMU tilt and the L+R span tilt should agree). It is not set to exactly 90.

KEYS    H reset + heading   R reset x/y only   U undo

RUN ON ITS OWN
    python tools/odom_reset_cal.py   opens the window against a fake robot.

WIRE FORMAT (see src/odom_reset_cal.cpp)
    robot -> laptop  RUI|ODOM_RESET|...|ODOMR:<25 numbers>|MSG:<status>
    laptop -> robot  ODOMR RESET 0|1  |  ODOMR UNDO  |  ODOMR POSE x y
"""

import math
import os
import sys
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from pid_tuner import AXIS, BG, CARD, CYAN, DIM, GRAY, GREEN, ORANGE, RED, WHITE  # noqa: E402
# The same field render and alignment the path planner uses (tools/field.png, drawn shifted a little
# so its pins land where they really are), so the two maps agree.
from planner_core import FIELD_IMG_SHIFT_IN, FIELD_PNG  # noqa: E402

INPUT = "#0f1218"
BTN = "#22222B"
GRID = "#1b1b24"
RAY = "#9fb4c8"
FIELD_PX = 500              # field.png is this many px square; without Pillow it is shown at this size
PINK = "#FF6B81"
FONT = "Segoe UI"
MONO = "Consolas"
FIELD_HALF = 72.0
TOLERANCE_IN = 0.5          # what counts as a good x/y fix
MAP_SIZE = 560


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a):
    return a % 360.0


def mm_text(mm):
    return f"{mm}" if mm >= 0 else ("OOR" if mm == -1 else "ERR")


class OdomResetWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.title("Odom Reset -- V5")
        self.configure(bg=BG)
        self.st = None                  # latest "odomr" dict from the brain
        self.onBrainScreen = False
        self.lastSig = None             # to notice a new reset and add a LOG row
        self.logRows = []
        self.trueVars = (tk.StringVar(), tk.StringVar())
        self.hover = None
        self._fieldSrc = None           # field.png at full size (Pillow only)
        self._fieldImg = None           # what is drawn: the scaled image
        self._fieldKey = None           # side it was scaled to
        self._fixedScale = False        # no Pillow: the image cannot be resized, so the map is FIELD_PX
        self._loadField()
        self.protocol("WM_DELETE_WINDOW", self.withdraw)
        self._build()
        self.bind("h", lambda e: self._key(lambda: self.reset(True)))
        self.bind("r", lambda e: self._key(lambda: self.reset(False)))
        self.bind("u", lambda e: self._key(self.undo))
        self.update_idletasks()
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())

    def _key(self, fn):
        if not isinstance(self.focus_get(), tk.Entry):
            fn()

    # -- layout --
    def _btn(self, parent, text, cmd, bg=BTN, fg=WHITE, size=10, pad=6):
        return tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg, activebackground="#2d2d38",
                         activeforeground=fg, disabledforeground="#555", relief="flat",
                         font=(FONT, size, "bold"), cursor="hand2", pady=pad)

    def _card(self, parent, title):
        f = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=AXIS)
        tk.Label(f, text=title, bg=CARD, fg=GRAY, font=(FONT, 8, "bold")).pack(anchor="w", padx=10, pady=(6, 2))
        return f

    def _line(self, parent, font=(MONO, 10)):
        lab = tk.Label(parent, text="", bg=CARD, fg=WHITE, font=font, anchor="w", justify="left")
        lab.pack(fill="x", padx=10, pady=1)
        return lab

    def _build(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=12, pady=(10, 6))
        tk.Label(top, text="ODOM RESET", bg=BG, fg=CYAN, font=(FONT, 13, "bold")).pack(side="left")
        try:
            from reconnect_button import add_reconnect_button
            add_reconnect_button(self, top, side="right", padx=(6, 0))
        except ImportError:
            pass
        self.chipBrain = tk.Label(top, text="brain: waiting", bg=CARD, fg=ORANGE, font=(FONT, 9, "bold"),
                                  padx=10, pady=3)
        self.chipBrain.pack(side="right", padx=(6, 0))

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=12)

        # map
        mcard = self._card(body, "FIELD  (left-click: move odometry there   right-click: set TRUE position)")
        mcard.pack(side="left", fill="both", expand=True)
        self.map = tk.Canvas(mcard, width=MAP_SIZE, height=MAP_SIZE, bg=INPUT, highlightthickness=0)
        self.map.pack(fill="both", expand=True, padx=10, pady=(0, 4))
        self.map.bind("<Configure>", lambda e: self.drawMap())
        self.map.bind("<Button-1>", self.onLeftClick)
        self.map.bind("<Button-3>", self.onRightClick)
        self.map.bind("<Motion>", self.onMotion)
        self.map.bind("<Leave>", lambda e: self.onMotion(None))
        self.hoverLabel = tk.Label(mcard, text="", bg=CARD, fg=DIM, font=(MONO, 9), anchor="w")
        self.hoverLabel.pack(fill="x", padx=10, pady=(0, 6))

        # right column
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="y", padx=(12, 0))

        acts = tk.Frame(right, bg=BG)
        acts.pack(fill="x")
        self.hdgBtn = self._btn(acts, "RESET + HDG  (H)", lambda: self.reset(True), bg="#3a2a10", fg=ORANGE,
                                size=11, pad=8)
        self.hdgBtn.pack(fill="x")
        self.xyBtn = self._btn(acts, "RESET XY  (R)", lambda: self.reset(False), bg="#10303a", fg=CYAN, size=11, pad=8)
        self.xyBtn.pack(fill="x", pady=(4, 0))
        self.undoBtn = self._btn(acts, "UNDO  (U)", self.undo, size=10, pad=6)
        self.undoBtn.pack(fill="x", pady=(4, 0))

        live = self._card(right, "LIVE")
        live.pack(fill="x", pady=(8, 0))
        self.odomLine = self._line(live)
        self.wallLine = self._line(live)
        self.offLine = self._line(live)
        self.mmLine = self._line(live)
        self.tiltLine = self._line(live)
        tk.Frame(live, bg=CARD, height=4).pack()

        tcard = self._card(right, "TRUE POSITION  (tape measure, inches)")
        tcard.pack(fill="x", pady=(8, 0))
        row = tk.Frame(tcard, bg=CARD)
        row.pack(padx=10, pady=(0, 4))
        for i, name in enumerate(("X", "Y")):
            tk.Label(row, text=name, bg=CARD, fg=GRAY, font=(FONT, 9, "bold")).grid(row=0, column=i * 2, padx=(0, 4))
            e = tk.Entry(row, textvariable=self.trueVars[i], width=8, justify="center", bg=INPUT, fg=WHITE,
                         insertbackground=WHITE, relief="flat", font=(MONO, 12))
            e.grid(row=0, column=i * 2 + 1, ipady=3, padx=(0, 10))
            e.bind("<Return>", lambda ev: self.drawMap())
            e.bind("<FocusOut>", lambda ev: self.refreshError())
        self._btn(row, "clear", self.clearTrue, size=8, pad=1).grid(row=0, column=4)
        self.errOdom = self._line(tcard)
        self.errWall = self._line(tcard)
        tk.Frame(tcard, bg=CARD, height=4).pack()

        lcard = self._card(right, "LOG  (one row per reset)")
        lcard.pack(fill="both", expand=True, pady=(8, 0))
        self.logBox = tk.Text(lcard, width=46, height=8, bg=INPUT, fg=WHITE, relief="flat", font=(MONO, 8),
                              state="disabled", wrap="none")
        self.logBox.pack(fill="both", expand=True, padx=10, pady=(0, 4))
        self._btn(lcard, "Copy log (CSV)", self.copyLog, size=8, pad=2).pack(anchor="e", padx=10, pady=(0, 6))

        self.message = tk.Label(self, text="", bg=BG, fg=CYAN, font=(FONT, 9), anchor="w")
        self.message.pack(fill="x", padx=12, pady=(8, 8))

    # -- sending --
    def send(self, line):
        try:
            self.link._send(line)
        except Exception as e:  # noqa: BLE001
            self.note(f"send failed: {e}", RED)

    def note(self, text, color=CYAN):
        self.message.config(text=text, fg=color)

    def reset(self, fixHeading):
        self.send(f"ODOMR RESET {1 if fixHeading else 0}")

    def undo(self):
        self.send("ODOMR UNDO")

    # -- field image (same approach as the path planner) --
    def _loadField(self):
        if not os.path.exists(FIELD_PNG):
            return
        try:
            from PIL import Image
            self._fieldSrc = Image.open(FIELD_PNG).convert("RGB")
        except ImportError:
            try:
                img = tk.PhotoImage(file=FIELD_PNG)
                f = max(1, round(img.width() / FIELD_PX))
                self._fieldImg = img.subsample(f, f)
                self._fixedScale = True
            except tk.TclError:
                self._fieldImg = None
        except Exception:  # noqa: BLE001
            self._fieldSrc = None

    def _fieldPhoto(self, side):
        """The field render at `side` px square, or None (then a plain grid is drawn)."""
        if self._fieldSrc is not None:
            if self._fieldKey != side:
                from PIL import Image, ImageTk
                self._fieldImg = ImageTk.PhotoImage(self._fieldSrc.resize((side, side), Image.LANCZOS))
                self._fieldKey = side
        return self._fieldImg

    # -- map geometry --
    def _geom(self):
        w, h = max(self.map.winfo_width(), 200), max(self.map.winfo_height(), 200)
        size = min(w, h)
        pad = 14
        s = (size / 2 - pad) / FIELD_HALF
        if self._fixedScale:
            s = FIELD_PX / (2 * FIELD_HALF)
        return w / 2, h / 2, s

    def toPx(self, x, y):
        cx, cy, s = self._geom()
        return cx + x * s, cy - y * s

    def toIn(self, px, py):
        cx, cy, s = self._geom()
        return (px - cx) / s, (cy - py) / s

    def onLeftClick(self, ev):
        x, y = self.toIn(ev.x, ev.y)
        x = max(-FIELD_HALF, min(FIELD_HALF, x))
        y = max(-FIELD_HALF, min(FIELD_HALF, y))
        self.send(f"ODOMR POSE {x:.2f} {y:.2f}")
        self.note(f"odometry moved to ({x:.1f}, {y:.1f}); now RESET XY", ORANGE)

    def onRightClick(self, ev):
        x, y = self.toIn(ev.x, ev.y)
        self.trueVars[0].set(f"{x:.1f}")
        self.trueVars[1].set(f"{y:.1f}")
        self.drawMap()
        self.refreshError()

    def onMotion(self, ev):
        if ev is None:
            self.hoverLabel.config(text="")
            return
        x, y = self.toIn(ev.x, ev.y)
        inside = abs(x) <= FIELD_HALF and abs(y) <= FIELD_HALF
        self.hoverLabel.config(text=f"({x:+.1f}, {y:+.1f}) in" if inside else "")

    def trueXY(self):
        try:
            return float(self.trueVars[0].get()), float(self.trueVars[1].get())
        except ValueError:
            return None

    def clearTrue(self):
        self.trueVars[0].set("")
        self.trueVars[1].set("")
        self.drawMap()
        self.refreshError()

    # -- drawing --
    def drawMap(self):
        c = self.map
        c.delete("all")
        cx, cy, s = self._geom()
        x0, y0 = self.toPx(-FIELD_HALF, FIELD_HALF)
        x1, y1 = self.toPx(FIELD_HALF, -FIELD_HALF)
        photo = self._fieldPhoto(int(round(x1 - x0)))
        if photo is not None:
            # Like the planner: the render alone on a flat ground, shifted so its pins sit where they really are.
            c.create_rectangle(x0, y0, x1, y1, outline=AXIS, fill="#12141a")
            c.create_image(x0 + FIELD_IMG_SHIFT_IN[0] * s, y0 + FIELD_IMG_SHIFT_IN[1] * s, anchor="nw", image=photo)
        else:                       # no field.png: a plain grid
            c.create_rectangle(x0, y0, x1, y1, outline=AXIS, fill="#12151b")
            for v in (-48, -24, 0, 24, 48):
                a, b = self.toPx(v, FIELD_HALF)
                c.create_line(a, y0, a, y1, fill=AXIS if v == 0 else GRID)
                a, b = self.toPx(FIELD_HALF, v)
                c.create_line(x0, b, x1, b, fill=AXIS if v == 0 else GRID)
        for v in (-72, -48, -24, 0, 24, 48, 72):
            px, _ = self.toPx(v, 0)
            c.create_text(px, y1 + 8, text=str(v), fill=GRAY, font=(FONT, 7))
            _, py = self.toPx(0, v)
            c.create_text(x0 - 10, py, text=str(v), fill=GRAY, font=(FONT, 7))

        st = self.st
        if st is None:
            c.create_text(cx, cy, text="waiting for the brain's ODOM RESET screen", fill=ORANGE, font=(FONT, 10))
            return

        if st["haveLast"]:
            lx, ly, lth = st["last"]
            self._robot(lx, ly, lth, "#555566", fill=False)

        for mm, off, mount in zip(st["mm"], (0, -90, 90), st["mount"]):
            if mm < 0:
                continue
            ang = math.radians(st["theta"] + off)
            ln = mm / 25.4 + mount
            ex, ey = st["x"] + math.sin(ang) * ln, st["y"] + math.cos(ang) * ln
            ox, oy = self.toPx(st["x"], st["y"])
            px, py = self.toPx(ex, ey)
            c.create_line(ox, oy, px, py, fill=RAY, dash=(4, 3))
            c.create_oval(px - 3, py - 3, px + 3, py + 3, fill=ORANGE, outline="")

        self._robot(st["x"], st["y"], st["theta"], CYAN, fill=True)   # under the two markers you compare

        t = self.trueXY()
        if t:
            px, py = self.toPx(*t)
            c.create_polygon(px, py - 8, px + 8, py, px, py + 8, px - 8, py, outline=PINK, fill="")
            c.create_text(px + 12, py - 10, text="TRUE", fill=PINK, font=(FONT, 8, "bold"))

        if self.haveEst():
            px, py = self.toPx(st["wx"], st["wy"])
            c.create_line(px - 9, py, px + 9, py, fill=GREEN, width=2)
            c.create_line(px, py - 9, px, py + 9, fill=GREEN, width=2)
            c.create_oval(px - 5, py - 5, px + 5, py + 5, outline=GREEN, width=2)

    def _robot(self, x, y, th, color, fill):
        """A 15 in square at (x, y) turned to heading th (compass: 0 = +Y, clockwise) with a nose line."""
        a = math.radians(th)
        fx, fy = math.sin(a), math.cos(a)      # forward
        rx, ry = math.cos(a), -math.sin(a)     # right
        pts = []
        for lx, ly in ((-7.5, -7.5), (7.5, -7.5), (7.5, 7.5), (-7.5, 7.5)):
            pts += list(self.toPx(x + lx * rx + ly * fx, y + lx * ry + ly * fy))
        # a stippled fill keeps the field markings visible under the robot
        self.map.create_polygon(*pts, outline=color, fill=color if fill else "", stipple="gray25" if fill else "",
                                width=2)
        ox, oy = self.toPx(x, y)
        nx, ny = self.toPx(x + fx * 12, y + fy * 12)
        self.map.create_line(ox, oy, nx, ny, fill=color, width=2)
        self.map.create_oval(ox - 2, oy - 2, ox + 2, oy + 2, fill=color, outline="")

    def haveEst(self):
        st = self.st
        return bool(st and st["aligned"] and (st["xOk"] or st["yOk"]))

    # -- readouts --
    def refreshLive(self):
        st = self.st
        if st is None:
            return
        snapped = round(st["theta"] / 90.0) * 90.0
        tilt = wrap180(st["theta"] - snapped)
        wall = int(round(snapped)) % 360
        self.odomLine.config(text=f"odom   X {st['x']:+7.2f}  Y {st['y']:+7.2f}  H {wrap360(st['theta']):6.1f}",
                             fg=CYAN)
        if self.haveEst():
            self.wallLine.config(
                text=f"walls  X {st['wx']:+7.2f}{'' if st['xOk'] else '*'}  Y {st['wy']:+7.2f}{'' if st['yOk'] else '*'}"
                     "   (* = no wall)", fg=GREEN)
            dx, dy, lim = st["wx"] - st["x"], st["wy"] - st["y"], st["maxCorr"]
            apply = (not st["xOk"] or abs(dx) <= lim) and (not st["yOk"] or abs(dy) <= lim)
            self.offLine.config(text=f"off    dX {dx:+6.2f}  dY {dy:+6.2f}   "
                                     + ("reset applies" if apply else f"REJECTED (> {lim:.0f} in)"),
                                fg=GREEN if apply else ORANGE)
        else:
            self.wallLine.config(text="walls  heading more than 10 deg off a wall" if not st["aligned"]
                                 else "walls  no valid reading", fg=GRAY)
            self.offLine.config(text="", fg=GRAY)
        f, l, r = st["mm"]
        self.mmLine.config(text=f"F {mm_text(f)}   L {mm_text(l)}   R {mm_text(r)}  mm", fg=WHITE)
        if st["tiltOk"]:
            self.tiltLine.config(text=f"tilt   IMU {tilt:+5.1f}   L+R span {st['tilt']:4.1f}  (wall {wall})", fg=CYAN)
        else:
            self.tiltLine.config(text=f"tilt   IMU {tilt:+5.1f}   L+R span n/a  (wall {wall})", fg=GRAY)

    def refreshError(self):
        t, st = self.trueXY(), self.st
        if not t or st is None:
            self.errOdom.config(text="odom error  -", fg=GRAY)
            self.errWall.config(text="walls error -", fg=GRAY)
            return
        e = math.hypot(st["x"] - t[0], st["y"] - t[1])
        self.errOdom.config(text=f"odom error  {e:5.2f} in   (dX {st['x'] - t[0]:+.2f}, dY {st['y'] - t[1]:+.2f})",
                            fg=GREEN if e <= TOLERANCE_IN else ORANGE)
        if self.haveEst():
            w = math.hypot(st["wx"] - t[0], st["wy"] - t[1])
            self.errWall.config(text=f"walls error {w:5.2f} in   (dX {st['wx'] - t[0]:+.2f}, dY {st['wy'] - t[1]:+.2f})",
                                fg=GREEN if w <= TOLERANCE_IN else RED)
        else:
            self.errWall.config(text="walls error -", fg=GRAY)

    # -- log --
    def addLog(self):
        st = self.st
        lx, ly, lth = st["last"]
        t = self.trueXY()
        err = math.hypot(st["x"] - t[0], st["y"] - t[1]) if t else None
        row = {"n": len(self.logRows) + 1, "bx": lx, "by": ly, "bh": lth, "ax": st["x"], "ay": st["y"],
               "ah": st["theta"], "dh": st["dH"], "err": err, "msg": st["msg"]}
        self.logRows.append(row)
        text = (f"#{row['n']:<2} ({lx:+6.1f},{ly:+6.1f}) -> ({st['x']:+6.1f},{st['y']:+6.1f})  dH {st['dH']:+5.1f}"
                + (f"  err {err:4.2f}" if err is not None else "  err  -  "))
        self.logBox.config(state="normal")
        self.logBox.insert("end", text + "\n")
        self.logBox.see("end")
        self.logBox.config(state="disabled")

    def copyLog(self):
        lines = ["n,before_x,before_y,before_h,after_x,after_y,after_h,dh,err_in"]
        for r in self.logRows:
            lines.append(f"{r['n']},{r['bx']:.2f},{r['by']:.2f},{r['bh']:.2f},{r['ax']:.2f},{r['ay']:.2f},"
                         f"{r['ah']:.2f},{r['dh']:.2f},{'' if r['err'] is None else format(r['err'], '.2f')}")
        self.clipboard_clear()
        self.clipboard_append("\n".join(lines))
        self.note(f"copied {len(self.logRows)} rows", GREEN)

    # -- state from the brain --
    def applyState(self, state):
        self.onBrainScreen = state["screen"] == "ODOM_RESET"
        d = state.get("odomr")
        if not (self.onBrainScreen and d):
            self.chipBrain.config(text=f"brain: {state['screen']}", fg=ORANGE)
            self.note("Tap ODOM RESET on the brain to test from here.", ORANGE)
            return
        self.chipBrain.config(text="brain: ODOM RESET", fg=GREEN)
        self.st = d
        self.undoBtn.config(state="normal" if d["haveUndo"] else "disabled")
        sig = (d["haveLast"], d["last"])
        if self.lastSig is not None and d["haveLast"] and sig != self.lastSig:
            self.addLog()
        self.lastSig = sig
        self.refreshLive()
        self.refreshError()
        self.drawMap()
        self.note(d.get("msg", ""), CYAN)


# ─── Stand-alone demo ──────────────────────────────────────────────────────
class _FakeLink:
    """A pretend robot standing at a true spot, with odometry that has drifted off it."""

    def __init__(self):
        self.true = (-20.0, 30.0)
        self.odom = [-17.5, 28.0]
        self.theta = 4.0
        self.mount = (7.5, 7.5, 7.5)
        self.haveLast, self.last, self.haveUndo, self.undoPose = False, (0, 0, 0), False, None
        self.dh = self.dx = self.dy = 0.0
        self.msg = "demo robot"

    def _send(self, line):
        print("->", line)
        f = line.split()
        if f[1] == "POSE":
            self.undoPose, self.haveUndo = list(self.odom), True
            self.odom = [float(f[2]), float(f[3])]
        elif f[1] == "RESET":
            b = tuple(self.odom) + (self.theta,)
            wx, wy, ok = self.walls()
            if ok and abs(wx - self.odom[0]) <= 4 and abs(wy - self.odom[1]) <= 4:
                self.undoPose, self.haveUndo = list(self.odom), True
                self.dx, self.dy, self.dh = wx - self.odom[0], wy - self.odom[1], 0.0
                self.odom = [wx, wy]
                self.msg = f"OK (x/y only): x {self.dx:+.2f} in, y {self.dy:+.2f} in"
            else:
                self.msg = "NO CHANGE: no reading used (obstacle / >4 in off)"
            self.haveLast, self.last = True, b
        elif f[1] == "UNDO" and self.haveUndo:
            self.odom, self.haveUndo, self.haveLast = self.undoPose, False, False
            self.msg = "pose restored to before the last change"

    def walls(self):
        tx, ty = self.true
        c = math.cos(math.radians(self.theta))
        # what the sensors measure: perpendicular distance / cos(tilt), minus the mount, in mm
        mm = lambda d: int((d / c - self.mount[0]) * 25.4)  # noqa: E731
        self.mm = [mm(72 - ty), mm(72 + tx), mm(72 - tx)]
        return tx, ty, True

    def state(self):
        wx, wy, _ = self.walls()
        d = {"x": self.odom[0], "y": self.odom[1], "theta": self.theta, "mm": list(self.mm), "aligned": True,
             "xOk": True, "yOk": True, "wx": wx, "wy": wy, "tiltOk": False, "tilt": 0.0, "haveLast": self.haveLast,
             "last": tuple(self.last), "dH": self.dh, "dX": self.dx, "dY": self.dy, "haveUndo": self.haveUndo,
             "mount": self.mount, "maxCorr": 4.0, "msg": self.msg}
        return {"screen": "ODOM_RESET", "odomr": d}


def _demo():
    root = tk.Tk()
    root.withdraw()
    link = _FakeLink()
    win = OdomResetWindow(root, link)
    win.protocol("WM_DELETE_WINDOW", root.destroy)
    win.trueVars[0].set("-20")
    win.trueVars[1].set("30")

    def tick():
        win.applyState(link.state())
        root.after(200, tick)
    tick()
    root.mainloop()


if __name__ == "__main__":
    _demo()
