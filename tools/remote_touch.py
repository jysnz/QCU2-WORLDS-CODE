#!/usr/bin/env python3
"""
Remote Touch -- the V5 brain's DRIVER MENU, on your laptop.

    python tools/remote_touch.py            connect to the brain over the USB cable
    python tools/remote_touch.py --demo     no robot: a pretend brain, to try it out
    python tools/remote_touch.py --debug    also print every line to / from the robot
    python tools/remote_touch.py --port COM5   use this serial port instead of looking for it

Needs pyserial (pip install pyserial). Close `pros terminal` and the lift tuner first -- only one
program can hold the brain's USB port.

WHAT YOU SEE
    A copy of whatever the driver menu shows on the brain (home, motor temps, the motor test),
    drawn at twice the size. Click a card or button and the brain reacts as if you had tapped its
    screen. The full-screen tools (PID tuning, path planner, lift tuner, intake tuner) cannot be
    mirrored; the window says which one is open and offers "Leave tool". For the lift and intake
    tuners it also has "Open lift tuner window", which hands the USB port over to tools/lift_tuner.py.

OPENING THE MENU ON THE ROBOT
    Hold DPAD-LEFT on the controller as driver control starts (or set START_IN_DRIVER_MENU in
    src/main.cpp). This window shows "waiting" until the menu is open.

HOW IT WORKS (robot side: the "Laptop link" section of src/driver_menu.cpp)
    robot -> laptop   RUI|HOME|label~sub;..           the cards
                      RUI|TEMPS|title:name,temp,conn,over;..#title:..
                      RUI|TEST_CONFIRM|   RUI|TEST_RUN|done|aborted|label,port,rev,state;..
                      RUI|TOOL|PID|       RUI|DRIVE|
    laptop -> robot   TOUCH x y   a tap on the brain screen (480 x 240)
                      LEAVE       leave the open tool, same as holding the screen's top-right corner
                      RATE ms     how often the robot sends (default 150)
    The drawing code below is a twin of the draw*() functions in src/driver_menu.cpp (same
    coordinates and colours). If you change one, change the other.
"""

import argparse
import math
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import lift_tuner as lt  # noqa: E402  (palette, widgets and the USB helpers)
from vexcom_link import VexcomLink, find_vexcom, is_controller_port  # noqa: E402  (laptop plugged into the controller)

C = lt.C

# -- the brain's screen: palette and layout (twin of src/driver_menu.cpp) ------------------------
W, H = 480, 240
BG, CARD, CYAN, ORANGE, GREEN = 0x0A0A0F, 0x161B22, 0x00F0FF, 0xFF8C00, 0x2ECC71
RED, GRAY, AXIS, FOOTER, WHITE = 0xFF4D4D, 0x777777, 0x2D2D38, 0x18181F, 0xFFFFFF
WARM_C, HOT_C = 45, 55   # dashboard::WARM_C / HOT_C
GRID_X0, GRID_X1, GRID_Y0, ROW_H, GAP, COLS = 15, 465, 58, 48, 4, 3
BACK = (396, 14, 468, 40)
# The DISTANCE page (src/distance_diag.cpp): same rectangles as on the brain, so a click is the same tap.
D_CLAW = (345, 57, 466, 81)
D_MINUS, D_PLUS = (12, 181, 44, 209), (128, 181, 160, 209)
D_SET, D_DEFAULT, D_RESET = (170, 181, 252, 209), (260, 181, 330, 209), (338, 181, 466, 209)
PLAN_STATES = ["IDLE", "RUNNING", "DONE", "CANCELLED"]   # the brain's order (src/laptop_plan.cpp)
D_STATES = ["NO DATA", "BAD DATA", "STALE", "SENSOR NOT FOUND", "OUT OF RANGE", "OK"]  # the brain's State order
START = (150, 160, 330, 205)
TOOL_NAMES = {"PID": "PID TUNING", "PLANNER": "PATH PLANNER", "LIFT": "LIFT TUNER", "INTAKE": "INTAKE TUNER",
              "DISTANCE": "DISTANCE SENSOR", "ODOM": "ODOM SETUP"}
HOME_ITEMS = [("DRIVE", "joystick driving"), ("PID TUNING", "angular + lateral"), ("PATH PLANNER", "drive to a point"),
              ("TEST MOTORS", "drivetrain, 2 s each"), ("LIFT TUNER", "levels, combos, angles"),
              ("INTAKE TUNER", "combos on L1 / L2"), ("MOTOR TEMPS", "every motor, live"),
              ("DISTANCE", "sensor check + calibrate")]


def hx(c):
    return f"#{c:06X}"


def in_rect(r, x, y):
    return r[0] <= x <= r[2] and r[1] <= y <= r[3]


def item_rect(i):
    col_w = (GRID_X1 - GRID_X0 - (COLS - 1) * GAP) // COLS
    x0 = GRID_X0 + (i % COLS) * (col_w + GAP)
    y0 = GRID_Y0 + (i // COLS) * (ROW_H + GAP)
    return (x0, y0, x0 + col_w, y0 + ROW_H)


# -- parsing the robot's RUI lines ----------------------------------------------------------------
def parse_rui(line):
    """'RUI|TEMPS|...' -> (screen, data) with the data already split into Python structures."""
    parts = line.rstrip("\r\n").split("|", 2)
    if len(parts) < 2 or parts[0] != "RUI":
        return None
    screen, raw = parts[1], parts[2] if len(parts) > 2 else ""
    try:
        if screen == "HOME":
            items = [tuple(x.split("~", 1)) for x in raw.split(";") if x]
            return screen, [(a, b) for a, b in items]
        if screen == "TEMPS":
            sections = []
            for sec in filter(None, raw.split("#")):
                title, motors = sec.split(":", 1)
                rows = []
                for mo in filter(None, motors.split(";")):
                    name, temp, conn, over = mo.split(",")
                    rows.append((name, float(temp), conn == "1", over == "1"))
                sections.append((title, rows))
            return screen, sections
        if screen == "TEST_RUN":
            done, aborted, rows = (raw.split("|", 2) + ["", ""])[:3]
            tests = []
            for t in filter(None, rows.split(";")):
                label, port, rev, state, rpm, ms = t.split(",")
                tests.append((label, int(port), rev == "1", int(state), int(rpm), int(ms)))
            return screen, (done == "1", aborted == "1", tests)
        if screen == "TOOL":
            return screen, raw.split("|")[0]
        if screen == "PLANNER":
            f = raw.split("|", 6)
            if len(f) < 7:
                return None
            status, block, count = int(f[0]), int(f[1]), int(f[2])
            x, y, th = float(f[3]), float(f[4]), float(f[5])
            pts = []
            for chunk in filter(None, f[6].split(";")):
                px, py = chunk.split(",")
                pts.append((float(px), float(py)))
            return screen, {"status": status, "block": block, "count": count, "x": x, "y": y, "theta": th, "path": pts}
        if screen == "PID_TUNER":   # <mode>,<sel>,<digitExp>,<aP,aI,aD>,<lP,lI,lD>,<busy>,<cal>|<label>,<over>,<settle>,<err>,<ms>
            gains, _, res = raw.partition("|")
            v = [float(x) for x in gains.split(",")]
            if len(v) != 11:
                return None
            pid = {"mode": int(v[0]), "sel": int(v[1]), "digitExp": int(v[2]), "angular": v[3:6], "lateral": v[6:9],
                   "busy": v[9] == 1, "calibrating": v[10] == 1}
            r = res.split(",")
            result = (r[0], float(r[1]), int(r[2]), float(r[3]), int(r[4])) if len(r) == 5 else None
            return screen, {"screen": "PID_TUNER", "pid": pid, "result": result}
        if screen == "DISTANCE":
            f = raw.split("|", 13)  # 13 numbers, then the footer text (which is free text)
            if len(f) < 14:
                return None
            keys = ("state", "card", "fps", "frames", "bad", "shown", "raw", "offset", "default", "ref", "claw",
                    "can_set", "is_note")
            d = {k: int(v) for k, v in zip(keys, f[:13])}
            d["footer"] = f[13]
            return screen, d
        return screen, raw
    except ValueError:
        return None


# -- the mirror ---------------------------------------------------------------------------------------
class Mirror(tk.Canvas):
    """The brain's 480x240 screen, redrawn at k times the size. Clicks are reported in brain pixels."""

    def __init__(self, parent, k, on_tap):
        super().__init__(parent, width=W * k, height=H * k, bg=hx(BG), highlightthickness=1,
                         highlightbackground=C.field_edge, cursor="hand2")
        self.k, self.on_tap = k, on_tap
        # The brain's own font is a condensed one; use a condensed face here too when there is one.
        fams = set(tkfont.families(self))
        self.family = next((f for f in ("Bahnschrift SemiCondensed", "Bahnschrift Condensed", "Arial Narrow") if f in fams),
                           lt.UI_FONT)
        self._fonts = {}
        self.bind("<Button-1>", self._click)

    def _click(self, e):
        x, y = max(0, min(W - 1, int(e.x / self.k))), max(0, min(H - 1, int(e.y / self.k)))
        self.on_tap(x, y)
        r = lt.S(14)
        ring = self.create_oval(e.x - r, e.y - r, e.x + r, e.y + r, outline=hx(CYAN), width=2)
        self.after(260, lambda: self.delete(ring))

    # primitives, all in brain pixels
    def rrect(self, x0, y0, x1, y1, r, fill, outline=""):
        k = self.k
        r = min(r, (x1 - x0) / 2, (y1 - y0) / 2)
        pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1, x1 - r, y1, x0 + r, y1, x0, y1,
               x0, y1 - r, x0, y0 + r, x0, y0]
        self.create_polygon([p * k for p in pts], smooth=True, fill=hx(fill), outline=hx(outline) if outline else "")

    def rect(self, x0, y0, x1, y1, fill):
        self.create_rectangle(x0 * self.k, y0 * self.k, x1 * self.k, y1 * self.k, fill=hx(fill), outline="")

    def dot(self, cx, cy, r, fill):
        k = self.k
        self.create_oval((cx - r) * k, (cy - r) * k, (cx + r) * k, (cy + r) * k, fill=hx(fill), outline="")

    def _font(self, px):
        if px not in self._fonts:
            self._fonts[px] = tkfont.Font(family=self.family, size=-px, weight="bold")
        return self._fonts[px]

    def text(self, x, y, s, color, size=12, anchor="nw", max_w=None):
        """Text at (x, y) in brain pixels. It shrinks until it fits max_w (default: the screen's right edge)."""
        px = max(1, int(size * self.k))
        if max_w is None and anchor == "nw":
            max_w = W - x - 6
        if max_w:
            while px > 6 * self.k and self._font(px).measure(s) > max_w * self.k:
                px -= 1
        self.create_text(x * self.k, y * self.k, text=s, fill=hx(color), anchor=anchor, font=self._font(px))

    def header(self, title, crumb=None, back=False):
        self.delete("all")
        self.rect(0, 0, W, H, BG)
        self.rrect(5, 5, 475, 50, 8, CYAN)
        self.text(20, 9 if crumb else 14, title, 0x000000, 19)
        if crumb:
            self.text(20, 31, crumb, 0x1A1A1A, 12)
        if back:
            self.rrect(BACK[0], BACK[1], BACK[2], BACK[3], 6, BG)
            self.text(BACK[0] + 12, BACK[1] + 6, "< BACK", 0xCCCCCC, 12)

    def footer(self, s, color):
        self.rect(0, 218, W, H, FOOTER)
        self.text(10, 222, s, color, 12)

    # screens
    def show(self, screen, data):
        getattr(self, "draw_" + screen.lower(), self.draw_other)(data)

    def draw_wait(self, data):
        self.header("DRIVER MENU", "waiting for the brain")
        self.text(20, 80, "No driver menu yet.", WHITE, 19)
        self.text(20, 112, "On the controller, hold DPAD-LEFT as driver control starts.", GRAY, 12)
        self.text(20, 130, "(Or set START_IN_DRIVER_MENU in src/main.cpp.)", GRAY, 12)

    def draw_home(self, items):
        self.header("DRIVER MENU")
        for i, (label, sub) in enumerate(items):
            x0, y0, x1, y1 = item_rect(i)
            self.rrect(x0, y0, x1, y1, 8, CARD, AXIS)
            self.rrect(x0, y0, x0 + 5, y1, 3, CYAN)
            self.text(x0 + 14, y0 + 5, label, WHITE, 18, max_w=x1 - 24 - (x0 + 14))   # clear of the ">"
            self.text(x0 + 14, y1 - 20, sub, GRAY, 12, max_w=x1 - x0 - 24)
            self.text(x1 - 16, y0 + 6, ">", CYAN, 12)
        self.footer("To leave a tool, hold the top-right corner of this screen for 1 second.", GRAY)

    @staticmethod
    def temp_color(conn, over, temp):
        if not conn:
            return GRAY
        if over or temp >= HOT_C:
            return RED
        return ORANGE if temp >= WARM_C else CYAN

    def draw_temps(self, sections):
        self.header("MOTOR TEMPS", None, True)
        left, right, gap, per_row, tile_h, pitch = 15, 465, 6, 5, 38, 53
        tile_w = (right - left - (per_row - 1) * gap) // per_row
        hottest, missing = 0.0, False
        for s, (title, rows) in enumerate(sections):
            label_y = 55 + s * pitch
            self.text(left, label_y, title, ORANGE if title.startswith("RIGHT") else CYAN, 12)
            for i, (name, temp, conn, over) in enumerate(rows[:per_row]):
                x, y = left + i * (tile_w + gap), label_y + 15
                col = self.temp_color(conn, over, temp)
                self.rrect(x, y, x + tile_w, y + tile_h, 5, CARD, AXIS)
                self.dot(x + tile_w - 9, y + 9, 4, col)
                self.text(x + 7, y + 3, name, GRAY, 12)
                self.text(x + 7, y + 17, f"{temp:.1f}°C" if conn else "no motor", col, 12)
                hottest = max(hottest, temp) if conn else hottest
                missing = missing or not conn
                frac = max(0.0, min(1.0, (temp - 20.0) / 50.0)) if conn else 0.0
                bx, by, bw = x + 7, y + tile_h - 7, tile_w - 14
                self.rrect(bx, by, bx + bw, by + 4, 2, AXIS)
                if bw * frac >= 4:
                    self.rrect(bx, by, bx + bw * frac, by + 4, 2, col)
        self.footer(f"hottest {hottest:.1f}°C" + ("   (a motor is not answering)" if missing else ""),
                    RED if hottest >= HOT_C else (ORANGE if hottest >= WARM_C else CYAN))

    def draw_test_confirm(self, data):
        self.header("TEST MOTORS", "drivetrain only", True)
        self.text(20, 68, "Put the robot on blocks first.", WHITE, 19)
        self.text(20, 98, "The drive motors turn one at a time, 2 seconds each.", GRAY, 12)
        self.text(20, 116, "Every wheel should roll forward. If one rolls backward, flip", GRAY, 12)
        self.text(20, 134, "the sign of its port in main.cpp. BACK stops the test.", GRAY, 12)
        self.rrect(*START, 8, CARD, CYAN)
        self.text(240, 172, "START", GREEN, 19, "n")

    def draw_test_run(self, data):
        done, aborted, tests = data
        self.header("TEST MOTORS", "done" if done else "running...", True)
        left, gap, top, tile_h, pitch = 15, 8, 71, 25, 29
        tile_w = (465 - left - gap) // 2
        per_side = sum(1 for t in tests if t[0].startswith("L"))   # column 1: left drive, column 2: right
        self.text(left, 54, "LEFT DRIVE", CYAN, 12)
        self.text(left + tile_w + gap, 54, "RIGHT DRIVE", ORANGE, 12)
        answered = failed = 0
        for i, (label, port, rev, state, rpm, ms) in enumerate(tests):
            col = 0 if i < per_side else 1
            x, y = left + col * (tile_w + gap), top + (i - col * per_side) * pitch
            edge = CYAN if state == 1 else (RED if state == 3 else AXIS)
            self.rrect(x, y, x + tile_w, y + tile_h, 5, CARD, edge)
            self.text(x + 8, y + 5, f"{label}  P{port}{' rev' if rev else ''}", WHITE, 12)
            status = {1: f"{max(0, 2000 - ms) / 1000:.1f} s  {rpm} rpm", 2: f"OK  {rpm} rpm",
                      3: "NO ANSWER"}.get(state, "")
            self.text(x + tile_w - 8, y + 5, status, RED if state == 3 else (GREEN if state == 2 else CYAN), 12, "ne")
            if state == 1:
                self.rect(x + 8, y + tile_h - 4, x + 8 + (tile_w - 16) * min(ms, 2000) / 2000, y + tile_h - 2, CYAN)
            answered += state == 2
            failed += state == 3
        if aborted:
            msg = f"stopped. {answered} answered, {failed} did not."
        elif done:
            msg = f"{answered} of {len(tests)} motors answered." + ("  Check the red ones." if failed else "")
        else:
            msg = f"testing {answered + failed + 1} of {len(tests)}...  wheel should roll forward"
        self.footer(msg, RED if failed else (GREEN if done else CYAN))

    def draw_tool(self, name):
        self.header(TOOL_NAMES.get(name, name), "open on the brain")
        self.rrect(20, 66, 460, 190, 8, CARD, AXIS)
        self.text(36, 84, f"{TOOL_NAMES.get(name, name)} is open on the brain.", WHITE, 19)
        self.text(36, 116, "Its screen cannot be mirrored here. Use the controller on the robot.", GRAY, 12, max_w=416)
        self.text(36, 134, "Leave it with the button below, or hold the top-right corner of the brain screen.", GRAY, 12, max_w=416)

    def draw_distance(self, d):
        """The brain's DISTANCE page, drawn from the 'RUI|DISTANCE|...' line. Every button is where it is on the brain."""
        st, cs = d["state"], d["card"]
        state_col = GREEN if st == 5 else (ORANGE if st in (2, 4) else RED)
        self.header("DISTANCE", "VL53L0X on smart port 2", True)
        # status pill, link numbers, claw
        self.rrect(15, 57, 190, 81, 8, state_col)
        self.text(102, 61, D_STATES[st] if 0 <= st < len(D_STATES) else "?", 0x000000, 14, "n")
        self.text(200, 62, f"{d['fps']}/s  {d['frames']} frames  {d['bad']} bad", GRAY, 12, max_w=140)
        claw = d["claw"] == 1
        self.rrect(*D_CLAW, 8, GREEN if claw else AXIS)
        self.text((D_CLAW[0] + D_CLAW[2]) // 2, 61, "CLAW ON" if claw else "CLAW OFF", 0x000000 if claw else WHITE, 14, "n")
        # the sensor card
        valid = cs == 5
        self.rrect(15, 88, 465, 170, 8, CARD, CYAN)
        self.text(27, 96, "VL53L0X", CYAN, 12)
        tag = "OK" if valid else {4: "OUT OF RANGE", 3: "NO SENSOR"}.get(cs, "NO DATA")
        self.text(453, 96, tag, GREEN if valid else (RED if cs == 3 else (ORANGE if cs == 4 else GRAY)), 12, "ne")
        big = f"{d['shown']}" if valid else "--"
        self.text(27, 110, big, WHITE if valid else GRAY, 34, max_w=120)
        self.text(27 + 18 * len(big) + 14, 126, "mm", GRAY, 12)
        self.text(215, 114, f"raw {d['raw']}" if d["raw"] >= 0 else "raw --", GRAY, 12)
        self.text(325, 114, f"offset {d['offset']:+d} mm", GRAY if d["default"] else ORANGE, 12)
        self.rect(27, 154, 453, 160, AXIS)
        if valid:
            w = max(2, int(426 * min(d["shown"], 1200) / 1200))
            self.rect(27, 154, 27 + w, 160, GREEN)
        # calibration panel
        self.rrect(5, 176, 475, 214, 8, CARD, AXIS)
        for r, label in ((D_MINUS, "-"), (D_PLUS, "+")):
            self.rrect(*r, 6, AXIS)
            self.text((r[0] + r[2]) // 2, r[1] + 7, label, WHITE, 14, "n")
        self.text(86, 189, f"REF {d['ref']} mm", WHITE, 12, "n")
        self.rrect(*D_SET, 6, GREEN if d["can_set"] else AXIS)
        self.text((D_SET[0] + D_SET[2]) // 2, D_SET[1] + 8, "SET REF", 0x000000 if d["can_set"] else GRAY, 12, "n")
        self.rrect(*D_DEFAULT, 6, AXIS)
        self.text((D_DEFAULT[0] + D_DEFAULT[2]) // 2, D_DEFAULT[1] + 8, "DEFAULT", GRAY if d["default"] else ORANGE, 12, "n")
        self.rrect(*D_RESET, 6, AXIS)
        self.text((D_RESET[0] + D_RESET[2]) // 2, D_RESET[1] + 8, "RESET COUNTERS", 0xDDDDDD, 12, "n")
        self.footer(d["footer"], GREEN if d["is_note"] else (GRAY if st == 5 else state_col))

    def draw_planner(self, d):
        self.header("PATH PLANNER", "map planner on the laptop", True)
        self.rrect(15, 58, 465, 160, 8, CARD, AXIS)
        self.text(27, 66, "Plan on the robot", GRAY, 12)
        st = PLAN_STATES[d["status"]] if 0 <= d["status"] < len(PLAN_STATES) else "?"
        self.text(27, 84, st, GREEN if st == "RUNNING" else (RED if st == "CANCELLED" else WHITE), 28)
        self.text(300, 68, f"{d['count']} block{'' if d['count'] == 1 else 's'}", WHITE, 12)
        if st == "RUNNING" and d["block"] >= 0:
            self.text(300, 88, f"now: block {d['block'] + 1} of {d['count']}", GREEN, 12)
        self.text(27, 128, f"x {d['x']:.1f}   y {d['y']:.1f}   heading {d['theta']:.0f}", CYAN, 12)
        self.rrect(15, 168, 205, 206, 6, AXIS)
        self.text(110, 180, "BRAIN EDITOR", WHITE, 12, "n")
        self.footer("The planner window on the laptop has the map. Closed it? Tap PATH PLANNER again.", GRAY)

    def draw_drive(self, data):
        self.header("DRIVER MENU", "closed")
        self.text(20, 80, "The robot is driving.", WHITE, 19)
        self.text(20, 112, "To open the menu again, hold DPAD-LEFT as driver control starts.", GRAY, 12)

    def draw_pid_tuner(self, data):
        self.header("PID TUNER", "open on the brain", True)
        self.text(20, 84, "The PID Tuner window is open on the laptop.", WHITE, 14)
        self.text(20, 110, "Type gains, run tests and watch the graph there.", GRAY, 12)
        pid = data["pid"]
        if pid["calibrating"]:
            self.footer("CALIBRATING -- keep the robot still", ORANGE)
        else:
            self.footer("running -- graph in the PID Tuner window" if pid["busy"] else "idle", GRAY)

    def draw_other(self, data):
        self.header("DRIVER MENU", "unknown screen")


# -- links ----------------------------------------------------------------------------------------------
class Link(lt.SerialLink):
    """The USB link: like the lift tuner's, but it keeps the menu's RUI lines."""

    def __init__(self, port, debug=False):
        self.debug = debug
        self.pid_lines = queue.Queue()
        super().__init__(port)

    def _read(self):
        stream = lt.PacketStream()
        while not self.stop:
            try:
                raw = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                self.dead = True
                return
            for line in stream.feed(raw) if raw else []:
                if line.startswith("RUI|"):
                    if self.debug:
                        print("<", line)
                    self.lines.put(line)
                elif line.startswith(("PID|", "CSV,")):   # the PID tuner's run stream
                    self.pid_lines.put(line.rstrip("\r\n"))

    def send(self, lines):
        if self.debug:
            for line in lines:
                print(">", line)
        super().send(lines)


class FakeRobot:
    """A pretend brain with the same menu, for --demo and the tests."""
    name = "demo"
    port = "demo"

    def __init__(self):
        self.lines = queue.Queue()
        self.dead = False
        self.screen, self.tool = "HOME", ""
        self.plan, self.plan_status, self.plan_block = [], 0, -1
        self.pose, self.trail, self.run = [0.0, 0.0, 0.0], [], None
        self.dist = dict(ref=100, offset=-60, claw=0, frames=412, bad=0, raw=73, note="", note_until=0.0)
        self.sent = []
        self.t_state = 0.0
        self.t_test = None
        self.tests = [(f"{p}{i + 1}", port, rev, 0) for p, ports in
                      (("L", [-8, 9, 10, -11, 12]), ("R", [-3, 4, -5, 6, -7]))
                      for i, port, rev in ((i, abs(x), x < 0) for i, x in enumerate(ports))]

    def send(self, lines):
        for line in lines:
            self.sent.append(line)
            parts = line.split()
            if parts and parts[0] == "TOUCH" and len(parts) == 3:
                self._tap(int(parts[1]), int(parts[2]))
            elif parts and parts[0] == "LEAVE" and self.screen in ("TOOL", "DISTANCE", "PLANNER"):
                self.screen = "HOME"
            elif self.screen == "PLANNER":
                self._planner_line(line)

    def _tap(self, x, y):
        if self.screen == "HOME":
            for i in range(len(HOME_ITEMS)):
                if in_rect(item_rect(i), x, y):
                    label = HOME_ITEMS[i][0]
                    if label == "DRIVE":
                        self.screen = "DRIVE"
                    elif label == "MOTOR TEMPS":
                        self.screen = "TEMPS"
                    elif label == "TEST MOTORS":
                        self.screen = "TEST_CONFIRM"
                    elif label == "DISTANCE":
                        self.screen = "DISTANCE"
                    elif label == "PATH PLANNER":
                        self.screen = "PLANNER"
                    else:
                        self.screen, self.tool = "TOOL", {"PID TUNING": "PID", "PATH PLANNER": "PLANNER",
                                                          "LIFT TUNER": "LIFT", "INTAKE TUNER": "INTAKE",
                                                          "DISTANCE": "DISTANCE"}[label]
        elif self.screen == "DISTANCE":
            self._tap_distance(x, y)
        elif self.screen == "PLANNER":
            if in_rect(BACK, x, y):
                self.screen = "HOME"
        elif self.screen in ("TEMPS", "TEST_RUN", "TEST_CONFIRM") and in_rect(BACK, x, y):
            self.screen, self.t_test = "HOME", None
        elif self.screen == "TEST_CONFIRM" and in_rect(START, x, y):
            self.screen, self.t_test = "TEST_RUN", time.time()
            self.tests = [(a, b, c, 0) for a, b, c, _ in self.tests]

    def _tap_distance(self, x, y):
        d, now = self.dist, time.time()

        def note(text):
            d["note"], d["note_until"] = text, now + 3
        if in_rect(BACK, x, y):
            self.screen = "HOME"
        elif in_rect(D_CLAW, x, y):
            d["claw"] ^= 1
            note("Claw ON (by hand)" if d["claw"] else "Claw OFF (by hand)")
        elif in_rect(D_MINUS, x, y):
            d["ref"] = max(20, d["ref"] - 10)
        elif in_rect(D_PLUS, x, y):
            d["ref"] = min(1200, d["ref"] + 10)
        elif in_rect(D_SET, x, y):
            d["offset"] = d["ref"] - d["raw"]
            note(f"Offset set to {d['offset']:+d} mm (saved)")
        elif in_rect(D_DEFAULT, x, y):
            d["offset"] = -60
            note("Offset back to the default -60 mm (saved)")
        elif in_rect(D_RESET, x, y):
            d["frames"] = d["bad"] = 0
            note("Counters reset")

    def _distance_line(self):
        d, now = self.dist, time.time()
        raw = max(0, int(d["raw"] + 6 * math.sin(now * 2)))      # a target that wobbles a little
        shown = max(0, raw + d["offset"])
        d["frames"] += 3
        if now > d["note_until"]:
            d["note"] = ""
        if d["note"] == "" and shown <= 10 and not d["claw"]:    # the auto-claw, like the brain's page
            d["claw"], d["note"], d["note_until"] = 1, "In the claw (0-10 mm): claw ON", now + 3
        footer = d["note"] or "Working. Offset -60 by default: a reading of 60 mm shows as 0. SET REF re-calibrates."
        return (f"RUI|DISTANCE|5|5|20|{d['frames']}|{d['bad']}|{shown}|{raw}|{d['offset']}|{1 if d['offset'] == -60 else 0}|"
                f"{d['ref']}|{d['claw']}|1|{1 if d['note'] else 0}|{footer}")

    def _planner_line(self, line):
        """What the real brain does with the planner's commands (src/laptop_plan.cpp), roughly, at demo speed."""
        p = line.split()
        if p[:2] == ["PLAN", "CLEAR"] and self.plan_status != 1:
            self.plan, self.plan_status, self.plan_block = [], 0, -1
        elif p[:2] == ["PLAN", "ADD"] and len(p) == 15:
            self.plan.append([float(v) for v in p[2:]])
        elif p[:2] == ["PLAN", "RUN"] and self.plan:
            self.run = {"i": 0, "t0": time.time(), "from": list(self.pose)}
            self.plan_status, self.trail = 1, []
        elif p[:2] == ["PLAN", "STOP"] and self.plan_status == 1:
            self.plan_status, self.run = 3, None
        elif p[0] == "POSE" and len(p) == 4 and self.plan_status != 1:
            self.pose, self.trail = [float(p[1]), float(p[2]), float(p[3])], []

    def _planner_tick(self):
        r = self.run
        if not r or self.plan_status != 1:
            return
        if r["i"] >= len(self.plan):
            self.plan_status, self.run = 2, None
            return
        self.plan_block = r["i"]
        k, x, y, th, ms = (int(self.plan[r["i"]][0]), *self.plan[r["i"]][1:4], self.plan[r["i"]][4])
        dist = math.hypot(x - r["from"][0], y - r["from"][1]) if k in (4, 5) else 0.0
        dur = max(0.4, ms / 1000.0) if k == 6 else (max(1.0, dist / 20.0) if k <= 5 else 0.05)   # ~20 in / s
        u = min(1.0, (time.time() - r["t0"]) / dur)
        if k in (4, 5):
            self.pose[0] = r["from"][0] + (x - r["from"][0]) * u
            self.pose[1] = r["from"][1] + (y - r["from"][1]) * u
            self.trail.append((self.pose[0], self.pose[1]))
        elif k == 7:
            self.pose = [x, y, th]
        if u >= 1.0:
            r["i"] += 1
            r["t0"], r["from"] = time.time(), list(self.pose)

    def _planner_state(self):
        pts = ";".join(f"{a:.1f},{b:.1f}" for a, b in self.trail[-120:])
        return (f"RUI|PLANNER|{self.plan_status}|{self.plan_block}|{len(self.plan)}|"
                f"{self.pose[0]:.2f}|{self.pose[1]:.2f}|{self.pose[2]:.2f}|{pts}")

    def _line(self):
        s = self.screen
        if s == "PLANNER":
            self._planner_tick()
            return self._planner_state()
        if s == "DISTANCE":
            return self._distance_line()
        if s == "HOME":
            return "RUI|HOME|" + ";".join(f"{a}~{b}" for a, b in HOME_ITEMS)
        if s == "TEMPS":
            t = time.time()
            sec = lambda title, prefix, ports: title + ":" + ";".join(
                f"{prefix} P{p},{35 + 12 * abs(math.sin(t / 9 + p)):.1f},{0 if (prefix == 'R' and p == 6) else 1},0"
                for p in ports)
            return "RUI|TEMPS|" + "#".join([sec("LEFT DRIVETRAIN", "L", [8, 9, 10, 11, 12]),
                                            sec("RIGHT DRIVETRAIN", "R", [3, 4, 5, 6, 7]),
                                            sec("LIFT, INTAKE, CLAW ARM", "M", [14, 18, 17, 16, 19])])
        if s == "TEST_CONFIRM":
            return "RUI|TEST_CONFIRM|"
        if s == "TEST_RUN":
            elapsed = time.time() - self.t_test
            step = int(elapsed / 2.3)   # 2 s spinning + a short pause, like the brain
            ms = min(2000, int((elapsed - step * 2.3) * 1000))
            done = step >= len(self.tests)
            rows = []
            for i, (label, port, rev, _) in enumerate(self.tests):
                state = 0 if i > step else (1 if i == step and not done else (3 if label == "R3" else 2))
                rpm = 140 if state in (1, 2) else 0
                rows.append(f"{label},{port},{1 if rev else 0},{state},{rpm},{ms if state == 1 else 0}")
            return f"RUI|TEST_RUN|{1 if done else 0}|0|" + ";".join(rows)
        if s == "TOOL":
            return f"RUI|TOOL|{self.tool}|"
        return "RUI|DRIVE|"

    def tick(self):
        now = time.time()
        if now - self.t_state >= 0.15:
            self.t_state = now
            self.lines.put(self._line())

    def close(self):
        pass


class PlannerLink:
    """What the planner window sends commands through (it calls ._send(text))."""

    def __init__(self, app):
        self.app = app

    def _send(self, cmd, *_a, **_k):
        if self.app.link:
            self.app.link.send([cmd])


# -- the window ---------------------------------------------------------------------------------------------
class App(lt.tk.Tk):
    def __init__(self, demo=False, debug=False, port=None):
        super().__init__()
        lt.init_look(self)
        self.demo, self.debug, self.port = demo, debug, port
        self.title("Remote Touch — Driver Menu")
        self.configure(bg=C.base)
        self.link, self.tuner_proc = None, None
        self.screen, self.data, self.last_line, self.last_rui = "WAIT", None, None, 0.0
        self.planner = None
        self.pid_tuner = None
        k = max(1, round(2 * lt.SCALE))
        self.columnconfigure(0, weight=1)

        bar = tk.Frame(self, bg=C.panel)
        bar.grid(row=0, column=0, sticky="ew")
        inner = tk.Frame(bar, bg=C.panel)
        inner.pack(fill="x", padx=lt.S(20), pady=lt.S(10))
        tk.Label(inner, text="Remote touch", bg=C.panel, fg=C.text, font=lt.fnt(lt.T["title"], "bold")).pack(side="left")
        conn = tk.Frame(inner, bg=C.panel)
        conn.pack(side="left", padx=(lt.S(24), 0))
        self.dotConn = lt.dot(conn, C.panel)
        self.dotConn.pack(side="left", padx=(0, lt.S(8)))
        self.lblConn = tk.Label(conn, text="Connecting…", bg=C.panel, fg=C.warning, font=lt.fnt(lt.T["body"], "bold"))
        self.lblConn.pack(side="left")
        lt.paint_dot(self.dotConn, C.warning)
        lt.Btn(inner, "Reconnect", self.connect, icon="refresh", variant="quiet", tip="Reconnect to the brain.").pack(side="right")
        tk.Frame(bar, bg=C.line, height=1).pack(fill="x")

        body = tk.Frame(self, bg=C.base)
        body.grid(row=1, column=0, padx=lt.S(20), pady=lt.S(16))
        self.mirror = Mirror(body, k, self.tap)
        self.mirror.pack()
        self.actions = tk.Frame(body, bg=C.base)
        self.actions.pack(fill="x", pady=(lt.S(12), 0))
        self.lblScreen = tk.Label(self.actions, text="", bg=C.base, fg=C.muted, font=lt.fnt())
        self.lblScreen.pack(side="left")
        self.btnTuner = lt.Btn(self.actions, "Open lift tuner window", self.open_tuner, icon="go",
                               tip="Hands the USB port to tools/lift_tuner.py. Close that window to come back here.")
        self.btnLeave = lt.Btn(self.actions, "Leave tool", lambda: self.send("LEAVE"), icon="stop", variant="quiet",
                               tip="Back to the driver menu (same as holding the brain screen's top-right corner).")

        bar2 = tk.Frame(self, bg=C.panel)
        bar2.grid(row=2, column=0, sticky="ew")
        tk.Frame(bar2, bg=C.line, height=1).pack(fill="x")
        self.lblNote = tk.Label(bar2, text="Click the picture like the brain's touchscreen.", bg=C.panel, fg=C.muted,
                                font=lt.fnt(), anchor="w")
        self.lblNote.pack(fill="x", padx=lt.S(20), pady=lt.S(7))

        self.mirror.show("WAIT", None)
        self.connect()
        self.after(40, self.poll)

    # -- link
    def set_conn(self, text, kind):
        col = {"ok": C.success, "wait": C.warning, "bad": C.danger}[kind]
        self.lblConn.configure(text=text, fg=col)
        lt.paint_dot(self.dotConn, col)

    def connect(self):
        if self.link:
            self.link.close()
            self.link = None
        if self.tuner_proc and self.tuner_proc.poll() is None:
            self.set_conn("Lift tuner window has the USB port", "wait")
            return
        if self.demo:
            self.link = FakeRobot()
            self.set_conn("Demo robot", "ok")
            return
        try:
            import serial  # noqa: F401
        except ImportError:
            self.set_conn("pyserial missing", "bad")
            self.lblNote.configure(text="Install it with:  pip install pyserial", fg=C.danger)
            return
        if self.port:
            self.cands = [self.port]
        else:
            self.cands = lt.find_brain_ports()
        self.cand_idx = getattr(self, "cand_idx", 0) % max(1, len(self.cands))
        port = self.cands[self.cand_idx] if self.cands else None
        if not port:
            self.set_conn("No brain found", "bad")
            self.lblNote.configure(text="Plug in the brain over USB and close pros terminal / the lift tuner. Retrying…", fg=C.danger)
            self.after(2000, self._retry)
            return
        try:
            vexcom = find_vexcom() if is_controller_port(port) else None
            if is_controller_port(port) and not vexcom:
                self.set_conn("Controller found, vexcom missing", "bad")
                self.lblNote.configure(text="The controller's radio needs vexcom.exe (installed with the PROS VS Code extension).", fg=C.danger)
                self.after(3000, self._retry)
                return
            self.link = VexcomLink(port, vexcom, self.debug) if vexcom else Link(port, self.debug)
        except Exception as e:
            self.set_conn(f"Can't open {port}", "bad")
            self.lblNote.configure(text=f"{e}  (is another program using the port?)", fg=C.danger)
            self.after(2000, self._retry)
            return
        self.last_rui = 0.0
        self.got_line, self.connected_at = False, time.time()
        self.set_conn(f"Connected to {port}", "ok")

    def _retry(self):
        if self.link is None and not (self.tuner_proc and self.tuner_proc.poll() is None):
            self.connect()

    def send(self, line):
        if not self.link:
            self.set_conn("Not connected", "bad")
            return
        self.link.send([line])

    def tap(self, x, y):
        self.send(f"TOUCH {x} {y}")

    def open_tuner(self):
        if self.link:
            self.link.close()
            self.link = None
        args = [sys.executable, os.path.join(HERE, "lift_tuner.py")] + (["--demo"] if self.demo else [])
        self.tuner_proc = subprocess.Popen(args)
        self.set_conn("Lift tuner window has the USB port", "wait")
        self.lblNote.configure(text="Close the lift tuner window to get this one back.", fg=C.muted)

    # -- incoming
    def poll(self):
        if self.tuner_proc and self.tuner_proc.poll() is not None:
            self.tuner_proc = None
            self.connect()
        if self.link:
            if getattr(self.link, "tick", None):
                self.link.tick()
            if self.link.dead:
                err = getattr(self.link, "error", None)
                self.link = None
                self.set_conn("Connection lost", "bad")
                if err:
                    self.lblNote.configure(text=err, fg=C.danger)
                self.after(2000, self._retry)
            else:
                try:
                    while True:
                        self.on_line(self.link.lines.get_nowait())
                except queue.Empty:
                    pass
                self._pump_pid_lines()
                if (not self.demo and self.link and not getattr(self, "got_line", True)
                        and time.time() - getattr(self, "connected_at", time.time()) > 3):
                    # The port opened, but not one line has come back: the brain is not talking on it.
                    if len(getattr(self, "cands", [])) > 1:
                        # The brain has two USB ports and only one carries the program's text: try the other.
                        self.cand_idx = (self.cand_idx + 1) % len(self.cands)
                        self.link.close()
                        self.link = None
                        self.connect()
                        self.lblNote.configure(text=f"Nothing came from the previous port, so this one ({self.cands[self.cand_idx]}) is being tried. "
                                                    "If every port stays silent: is the program running, and is the new code uploaded?", fg=C.danger)
                        return self.after(40, self.poll)
                    self.set_conn("Port open, but the brain is silent", "wait")
                    self.lblNote.configure(text="Is the brain running this program (not just powered on)? Is the new code uploaded? "
                                                "Is VEXNET field control on? Retrying every 3 s.", fg=C.danger)
                    self.connected_at = time.time()
                if not self.demo and self.link and self.last_rui and time.time() - self.last_rui > 2.5:
                    self.set_conn("Connected, but the menu is not open", "wait")
                    self.lblNote.configure(text="Hold DPAD-LEFT on the controller as driver control starts.", fg=C.muted)
                    self.last_rui = 0.0
        self.after(40, self.poll)

    def on_line(self, line):
        parsed = parse_rui(line)
        if not parsed:
            return
        self.last_rui = time.time()
        self.got_line = True
        self._to_planner(parsed)
        self._to_pid_tuner(parsed)
        if self.link:
            self.set_conn(f"Connected to {getattr(self.link, 'port', 'brain')}" if not self.demo else "Demo robot", "ok")
        if line == self.last_line:
            return
        self.last_line = line
        self.screen, self.data = parsed
        self.mirror.show(self.screen, self.data)
        self.update_actions()

    # -- the PID tuner window (tools/pid_tuner.py, debi's) --
    def _to_pid_tuner(self, parsed):
        screen, data = parsed
        if screen == "PID_TUNER" and self.pid_tuner is None:
            try:
                from pid_tuner import PidTunerWindow
            except Exception as e:  # noqa: BLE001
                self.lblNote.configure(text=f"pid_tuner.py failed to load: {e}", fg=C.danger)
                return
            self.pid_tuner = PidTunerWindow(self, PlannerLink(self))
            self.pid_tuner.onReconnect = self.connect
            self.pid_tuner.bind("<Destroy>", lambda e: setattr(self, "pid_tuner", None) if e.widget is self.pid_tuner else None)
        if self.pid_tuner is None:
            return
        if screen == "PID_TUNER":
            if self.pid_tuner.state() == "withdrawn":
                self.pid_tuner.deiconify()
            state = data
        else:   # the brain is somewhere else: the window says so and keeps its graph
            state = {"screen": screen, "pid": None, "result": None}
        try:
            self.pid_tuner.applyState(state)
        except Exception as e:  # noqa: BLE001
            self.lblNote.configure(text=f"PID tuner window: {e}", fg=C.danger)

    def _pump_pid_lines(self):
        lines = getattr(self.link, "pid_lines", None)
        if lines is None:
            return
        try:
            while True:
                line = lines.get_nowait()
                if self.pid_tuner is not None:
                    self.pid_tuner.feedLine(line)
        except queue.Empty:
            pass

    # -- the map planner (tools/path_planner.py, debi's) --
    def _to_planner(self, parsed):
        screen, data = parsed
        if screen == "PLANNER" and self.planner is None:
            try:
                from path_planner import PlannerWindow
            except Exception as e:  # noqa: BLE001  (a missing PIL / syntax error should say so, not crash the window)
                self.lblNote.configure(text=f"path_planner.py failed to load: {e}", fg=C.danger)
                return
            self.planner = PlannerWindow(self, PlannerLink(self))
            self.planner.onReconnect = self.connect
            self.planner.bind("<Destroy>", lambda e: setattr(self, "planner", None) if e.widget is self.planner else None)
        if self.planner is None:
            return
        if screen == "PLANNER":
            if self.planner.state() == "withdrawn":
                self.planner.deiconify()
            state = {"screen": "PLANNER",
                     "plot": {"poseX": data["x"], "poseY": data["y"], "poseTheta": data["theta"]},
                     "path": data["path"], "plan": (PLAN_STATES[data["status"]], data["block"], data["count"])}
        else:   # the brain is somewhere else: the planner says so and keeps what it has
            state = {"screen": screen, "plot": None, "path": [], "plan": None}
        try:
            self.planner.applyState(state)
        except Exception as e:  # noqa: BLE001
            self.lblNote.configure(text=f"planner window: {e}", fg=C.danger)

    def update_actions(self):
        labels = {"HOME": "Driver menu", "TEMPS": "Motor temps", "TEST_CONFIRM": "Motor test", "TEST_RUN": "Motor test",
                  "TOOL": TOOL_NAMES.get(self.data, "") + " is open on the brain" if self.screen == "TOOL" else "",
                  "DISTANCE": "Distance sensor", "PID_TUNER": "PID tuner (the window opens on the laptop)", "PLANNER": "Map path planner (the window opens on the laptop)",
                  "DRIVE": "The robot is driving"}
        self.lblScreen.configure(text=labels.get(self.screen, ""))
        self.btnLeave.pack_forget()
        self.btnTuner.pack_forget()
        if self.screen in ("TOOL", "DISTANCE", "PLANNER", "PID_TUNER"):
            self.btnLeave.pack(side="right")
            if self.data in ("LIFT", "INTAKE"):
                self.btnTuner.pack(side="right", padx=(0, lt.S(8)))
        self.lblNote.configure(text="Click the picture like the brain's touchscreen.", fg=C.muted)


def main():
    ap = argparse.ArgumentParser(description="The V5 brain's driver menu on the laptop.")
    ap.add_argument("--demo", action="store_true", help="a pretend brain, no robot needed")
    ap.add_argument("--debug", action="store_true", help="print every line to / from the robot")
    ap.add_argument("--port", help="serial port of the brain (default: look for it)")
    args = ap.parse_args()
    lt.enable_hidpi()
    App(args.demo, args.debug, args.port).mainloop()


if __name__ == "__main__":
    main()
