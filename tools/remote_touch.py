#!/usr/bin/env python3
"""
Remote Touch -- laptop-side mirror + remote control for the V5 brain's
driver menu (see src/driver_menu.cpp, the "Remote touch bridge" section).

WHAT THIS IS
    A window that mirrors whatever screen the driver menu is currently
    showing on the brain (HOME, PATH PLANNER, TEST MOTORS, ...) and lets
    you click its buttons from your laptop instead of reaching for the
    brain's touchscreen. It rides on the same USB cable you'd normally
    plug in for `pros terminal` -- no extra hardware.

    The drawing code here is a function-for-function port of the brain's
    own draw*() routines in src/driver_menu.cpp (same palette, same
    coordinates, same primitives: fill_rect / fill_circle / draw_rect /
    draw_line / print), fed with the same inputs the brain reads -- so
    the window lays out exactly like the brain screen, just scaled up.
    If you change a layout constant or a draw function on the brain,
    change its twin here (they're named the same).

HOW IT WORKS
    The brain prints one line a few times a second describing everything
    the current screen shows -- not just its buttons, but the field
    list, footer status, movement code preview, and motor-test results
    too, whichever of those the current screen actually has:
        RUI|<screen>|<breadcrumb>|BTN:...|FIELDS:...|FOOTER:...|CODE:...|MOTORS:...
    This script reads that over a serial port, draws it as a window, and
    when you click a button, sends "TOUCH <x> <y>\n" back over the same
    port -- which the brain's driverMenuControl() loop treats exactly
    like a real tap on its screen. Keyboard input goes back the same way:
        KEY <name>            one controller button press (LEFT RIGHT UP
                              DOWN A B X L1 L2), handled like a real one
        SEL <index>           select EDIT field #index
        SET <index> <value>   type a value straight into EDIT field #index

PATH PLANNER
    Tapping PATH PLANNER (on the brain or here) opens a second, larger
    window -- see tools/path_planner.py -- with the field map, stackable
    movement blocks, run buttons and code export. The mirror keeps
    showing what the brain screen shows meanwhile.

KEYBOARD (EDIT screen)
    Left / Right        previous / next field         (controller LEFT/RIGHT)
    Up / Down           nudge value by the digit step (controller UP/DOWN)
    PageUp / PageDown   bigger / smaller digit step   (controller L1/L2)
    A                   run the motion                (controller A)
    X                   cancel a running motion       (controller X)
    B                   reset pose to (0, 0, 0)       (controller B)
    Esc                 tap BACK
    Click a field row   select it
    Type a value, Enter set the selected field to it -- numbers for
                        numeric fields; for choice fields the label
                        (FWD, CW, LEFT, ...) or its index

BEFORE YOU RUN THIS
    - Install pyserial once:  pip install pyserial
    - Close `pros terminal` and any other serial monitor (VS Code's PROS
      extension terminal included) -- only one program can hold the port
      open at a time.
    - Plug the USB cable into either the BRAIN (fast, ~7 updates/s) or
      the CONTROLLER (wireless, ~2 updates/s -- fine for tapping and
      editing, the path trail just draws more coarsely). Both are
      auto-detected:
        brain       -> two COM ports; we talk to the user port directly.
        controller  -> one COM port; program I/O only travels over the
                       radio through VEX's own `vexcom --user`, so we run
                       that as a helper and talk through it. vexcom ships
                       with the PROS VS Code extension and is found
                       automatically (or pass --vexcom PATH).
    - Make sure the brain is running code built from this repo (i.e.
      driverMenuControl() is active / the driver menu is on screen).

USAGE
    python tools/remote_touch.py                 # auto-picks a VEX port
    python tools/remote_touch.py COM5             # or name it yourself
    python tools/remote_touch.py COM5 --baud 9600 # if the default doesn't work
    python tools/remote_touch.py --debug          # print every decoded line
    python tools/remote_touch.py --via vexcom     # force the controller/radio path
    python tools/remote_touch.py --via serial     # force the direct-cable path

    Windows: check Device Manager -> Ports (COM & LPT) for something
    like "VEX Robotics ... (COMx)". If the brain exposes more than one
    port (a "User"/"Communications" one and a "System" one), the user
    one is the one running your program's printf/stdin -- try it first,
    then the other one if you see no RUI| lines.
"""

import argparse
import glob
import math
import os
import shutil
import subprocess
import queue
import sys
import threading
import time
import tkinter as tk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("This tool needs pyserial. Install it with:\n    pip install pyserial")
    sys.exit(1)

BRAIN_W, BRAIN_H = 480, 240   # native brain screen resolution
SCALE = 2                     # window is drawn at this multiple, for easier clicking
DEFAULT_BAUD = 115200


VEX_VID = 0x2888
VEX_PID_BRAIN = 0x0501
VEX_PID_CONTROLLER = 0x0503


def vex_ports():
    return [p for p in serial.tools.list_ports.comports()
            if p.vid == VEX_VID or "vex" in ((p.description or "") + (p.manufacturer or "")).lower()]


def is_controller_port(device):
    """True if `device` is a V5 controller (radio link) rather than a brain."""
    for p in vex_ports():
        if p.device == device:
            return p.pid == VEX_PID_CONTROLLER
    return False


def find_vexcom():
    """vexcom.exe as installed by the PROS VS Code extension, or on PATH."""
    found = shutil.which("vexcom")
    if found:
        return found
    roots = [os.path.expandvars(r"%APPDATA%\Code\User\globalStorage\sigbots.pros\install"),
             os.path.expanduser("~/.config/Code/User/globalStorage/sigbots.pros/install")]
    for root in roots:
        hits = glob.glob(os.path.join(root, "vex-vexcom-*", "vexcom*"))
        hits = [h for h in hits if os.path.isfile(h) and not h.endswith(".txt")]
        if hits:
            return hits[0]
    return None


def guess_port():
    """Best-effort pick of the brain's *user* serial port; None if unsure.

    A directly-connected brain shows up as two ports (system + user);
    the user one carries the program's stdout/stdin. A controller shows
    up as just one port, and its radio link doesn't carry program output
    at all -- so with exactly one VEX port we still return it, but the
    "nothing received" hint in the GUI explains what's going on."""
    candidates = list(serial.tools.list_ports.comports())
    vex = vex_ports()
    for p in vex:
        if "user" in (p.description or "").lower():
            return p.device
    if len(vex) == 2:
        # Windows often labels both as "USB Serial Device"; the user port
        # is the second interface (x.2) of the composite device.
        for p in vex:
            if "x.2" in (p.location or ""):
                return p.device
        return vex[1].device
    if len(vex) == 1:
        return vex[0].device
    return candidates[0].device if len(candidates) == 1 else None


def cobs_decode(packet):
    """Decodes one COBS-encoded packet (without its 0x00 terminator).
    Returns None if the packet is malformed."""
    out = bytearray()
    i = 0
    n = len(packet)
    while i < n:
        code = packet[i]
        if code == 0 or i + code > n + 1:
            return None
        out += packet[i + 1:i + code]
        i += code
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


class PacketStream:
    """Turns the raw byte stream from the brain's user port into text
    lines.

    PROS wraps every stdout write as  <4-byte stream id, e.g. b"sout">
    <payload>, COBS-encodes it and terminates with 0x00. COBS inserts an
    extra code byte every 254 bytes, so long RUI lines (the EDIT screen)
    would be corrupted if we just searched the raw bytes -- decode
    properly instead. Text that isn't framed that way (older kernels,
    or anything else on the port) is passed through as-is."""

    STREAM_IDS = (b"sout", b"serr", b"kdbg")

    def __init__(self):
        self._buf = bytearray()
        self._text = ""

    def feed(self, raw):
        """Feeds raw bytes; returns a list of complete text lines."""
        self._buf += raw
        while True:
            zero = self._buf.find(b"\x00")
            if zero == -1:
                break
            packet = bytes(self._buf[:zero])
            del self._buf[:zero + 1]
            if not packet:
                continue
            decoded = cobs_decode(packet)
            if decoded is not None and decoded[:4] in self.STREAM_IDS:
                self._text += decoded[4:].decode("utf-8", errors="ignore")
            else:
                self._text += packet.decode("utf-8", errors="ignore")
        # Unframed fallback: a complete line of plain printable text with
        # no 0x00 framing in sight is passed through as-is, so a non-COBS
        # source still works. (A half-received COBS packet is almost never
        # all-printable, so this rarely misfires; if it does, the next
        # 0x00 resyncs us.)
        if self._buf and b"\n" in self._buf and all(9 <= b < 127 for b in self._buf):
            self._text += bytes(self._buf).decode("utf-8", errors="ignore")
            self._buf.clear()
        lines = self._text.split("\n")
        self._text = lines.pop()
        return lines


KNOWN_SCREENS = ("HOME", "PATH_TYPE", "ANGULAR_LIST", "LATERAL_LIST", "EDIT", "MOTOR_TEST",
                 "PLANNER", "HANDOFF")
PLAN_STATUS = ("IDLE", "RUNNING", "DONE", "CANCELLED")   # == PlanStatus on the brain


def _ints(text, n):
    """'1,2,3' -> [1,2,3] if it has exactly n ints, else None."""
    try:
        v = [int(x) for x in text.split(",")]
    except ValueError:
        return None
    return v if len(v) == n else None


def parse_rui_line(line):
    """Parses one
        RUI|<screen>|<breadcrumb>|BTN:...|FIELDS:...|FOOTER:...|CODE:...
           |MOTORS:...|SCROLL:...|STEP:...|FOOTC:...|PLOT:...|TRAIL:...
           |SEL:...|ACTIVE:...
    line (see sendRemoteUiState() in driver_menu.cpp) into a dict, or
    None if the line isn't one of ours -- the port also carries all of
    the program's normal printf output (telemetry, CSV, ...) which we
    ignore."""
    idx = line.find("RUI|")
    if idx == -1:
        return None
    line = line[idx:]
    parts = line.rstrip("\n").split("|", 3)
    if len(parts) < 3:
        return None
    screen, breadcrumb = parts[1], parts[2]
    rest = parts[3] if len(parts) > 3 else ""
    # The controller's radio link drops bytes now and then; a mangled
    # screen tag is the cheapest tell, so throw those lines away rather
    # than drawing garbage.
    if screen not in KNOWN_SCREENS:
        return None

    # None of our segment values ever contain a literal '|', so splitting
    # the rest of the line on it separates the tagged segments cleanly
    # regardless of which ones the current screen actually populated.
    segments = {k: "" for k in ("BTN", "FIELDS", "FOOTER", "CODE", "MOTORS", "SCROLL",
                                "STEP", "FOOTC", "PLOT", "TRAIL", "SEL", "ACTIVE", "PLAN",
                                "PATH")}
    for chunk in rest.split("|"):
        tag, sep, value = chunk.partition(":")
        if sep and tag in segments:
            segments[tag] = value

    elements = []
    for chunk in segments["BTN"].split(";"):
        f = chunk.split(",", 4)
        if len(f) != 5:
            continue
        box = _ints(",".join(f[:4]), 4)
        if box:
            elements.append((*box, f[4]))

    fields = []
    for chunk in segments["FIELDS"].split(";"):
        f = chunk.split(",")
        if len(f) == 3:
            fields.append((f[0], f[1], f[2] == "1"))

    code_lines = segments["CODE"].split("~") if segments["CODE"] else []

    motors = {"L": [], "R": []}
    for chunk in segments["MOTORS"].split(";"):
        f = chunk.split(",", 2)
        if len(f) == 3 and f[0] in motors:
            motors[f[0]].append((f[1], f[2]))

    scroll = _ints(segments["SCROLL"], 2) or [0, 0]

    try:
        step = float(segments["STEP"])
    except ValueError:
        step = 1.0

    plot = None
    if segments["PLOT"]:
        try:
            v = [float(x) for x in segments["PLOT"].split(",")]
        except ValueError:
            v = []
        if len(v) == 9:
            plot = {"mode": int(v[0]), "hasPt": v[1] == 1, "hasHdg": v[2] == 1,
                    "x": v[3], "y": v[4], "theta": v[5],
                    "poseX": v[6], "poseY": v[7], "poseTheta": v[8]}

    trail = []
    for chunk in segments["TRAIL"].split(";"):
        pt = _ints(chunk, 2)
        if pt:
            trail.append(tuple(pt))

    active = _ints(segments["ACTIVE"], 2) or [-1, -1]

    plan = None
    v = _ints(segments["PLAN"], 3)
    if v and 0 <= v[0] < len(PLAN_STATUS):
        plan = (PLAN_STATUS[v[0]], v[1], v[2])

    path = []
    for chunk in segments["PATH"].split(";"):
        f = chunk.split(",")
        if len(f) == 2:
            try:
                path.append((float(f[0]), float(f[1])))
            except ValueError:
                pass

    return {
        "screen": screen,
        "breadcrumb": breadcrumb,
        "elements": elements,
        "fields": fields,
        "footer": segments["FOOTER"],
        "footerCancelled": segments["FOOTC"] == "1",
        "code": code_lines,
        "motors": motors,
        "canUp": scroll[0] == 1,
        "canDown": scroll[1] == 1,
        "step": step,
        "plot": plot,
        "trail": trail,
        "selected": int(segments["SEL"]) if segments["SEL"].lstrip("-").isdigit() else 0,
        "activeSide": active[0],
        "activeIdx": active[1],
        "plan": plan,
        "path": path,
    }


class SerialLink:
    """Owns the actual port. Reading happens on a background thread (serial
    reads block) and hands parsed states to the GUI thread via a queue;
    writes (touch commands) are called directly -- pyserial writes don't
    block meaningfully for a few bytes."""

    def __init__(self, port, baud, debug=False):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.states = queue.Queue()
        self.debug = debug
        self.bytesReceived = 0   # anything at all from the port
        self.linesReceived = 0   # decoded text lines (RUI or not)
        self._stop = False
        self._thread = threading.Thread(target=self._readLoop, daemon=True)
        self._thread.start()

    def _readLoop(self):
        stream = PacketStream()
        while not self._stop:
            try:
                raw = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                break
            if not raw:
                continue
            self.bytesReceived += len(raw)
            for line in stream.feed(raw):
                self.linesReceived += 1
                if self.debug:
                    print(repr(line))
                state = parse_rui_line(line)
                if state:
                    self.states.put(state)

    def sendTouch(self, x, y):
        self._send(f"TOUCH {x} {y}")

    def sendKey(self, name):
        self._send(f"KEY {name}")

    def sendSelect(self, index):
        self._send(f"SEL {index}")

    def sendSet(self, index, value):
        # One token only: the brain parses it with %s (see the listener).
        self._send(f"SET {index} {value.strip().split()[0] if value.strip() else ''}")

    def sendRate(self, ms):
        self._send(f"RATE {ms}")

    def _send(self, line):
        try:
            self.ser.write((line + "\n").encode("ascii", errors="ignore"))
        except serial.SerialException:
            pass

    def close(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass


class VexcomLink:
    """Same interface as SerialLink, but the bytes go through a child
    `vexcom --user <port>` process instead of a serial handle. That's the
    only thing that carries a program's stdout/stdin over the controller's
    radio (plain serial and even `pros terminal` see nothing there).
    vexcom relays the raw PROS-framed stream, so it's decoded with the
    same PacketStream as the direct cable."""

    def __init__(self, port, vexcom, debug=False):
        self.vexcom = vexcom
        self.port = port
        # The brain only relays program I/O over the radio on the
        # "download" channel (1); it sits on the pit channel (0) the rest
        # of the time -- same switch `pros upload` makes for a wireless
        # upload. Put it back on close().
        self._setChannel(1)
        self.proc = subprocess.Popen([vexcom, "--user", port], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        self.states = queue.Queue()
        self.debug = debug
        self.bytesReceived = 0
        self.linesReceived = 0
        self.error = None        # set if vexcom quits on us (port busy, etc.)
        self._lastLines = []
        self._stop = False
        self._thread = threading.Thread(target=self._readLoop, daemon=True)
        self._thread.start()

    def _readLoop(self):
        stream = PacketStream()
        while not self._stop:
            raw = self.proc.stdout.read1(4096) if hasattr(self.proc.stdout, "read1") \
                else self.proc.stdout.read(1)
            if not raw:
                break
            self.bytesReceived += len(raw)
            for line in stream.feed(raw):
                self.linesReceived += 1
                self._lastLines = (self._lastLines + [line.strip()])[-3:]
                if self.debug:
                    print(repr(line))
                state = parse_rui_line(line)
                if state:
                    self.states.put(state)
        if not self._stop:
            # vexcom exited by itself. When the port is already open
            # elsewhere it just says "COMx: No error" and quits with a
            # nonzero code -- translate that into something actionable.
            code = self.proc.wait()
            tail = " / ".join(l for l in self._lastLines if l and "vexcom utility" not in l)
            self.error = (f"vexcom quit (exit {code}: {tail or 'no message'}). Is the PROS "
                          "terminal in VS Code (or another vexcom / remote_touch) still using "
                          f"{self.port}? Close it and restart this tool.")

    def sendTouch(self, x, y):
        self._send(f"TOUCH {x} {y}")

    def sendKey(self, name):
        self._send(f"KEY {name}")

    def sendSelect(self, index):
        self._send(f"SEL {index}")

    def sendSet(self, index, value):
        self._send(f"SET {index} {value.strip().split()[0] if value.strip() else ''}")

    def sendRate(self, ms):
        self._send(f"RATE {ms}")

    def _send(self, line):
        try:
            self.proc.stdin.write((line + "\n").encode("ascii", errors="ignore"))
            self.proc.stdin.flush()
        except (OSError, ValueError):
            pass

    def _setChannel(self, chan):
        try:
            subprocess.run([self.vexcom, "--chan", str(chan), self.port], capture_output=True,
                           timeout=15, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except (OSError, subprocess.TimeoutExpired):
            pass

    def close(self):
        self._stop = True
        try:
            self.proc.kill()
            self.proc.wait(timeout=5)
        except Exception:
            pass
        self._setChannel(0)


# ─── Port of the brain's drawing code ───────────────────────────────────────
# Everything from here to RemoteTouchApp mirrors src/driver_menu.cpp:
# the ui:: palette, the layout constants, and each draw*() function, on
# top of pros::screen-shaped primitives. Keep the two in step.

# ui:: palette
BG = "#0A0A0F"
CARD = "#161B22"
CYAN = "#00F0FF"
ORANGE = "#FF8C00"
GREEN = "#2ECC71"
GRAY = "#777777"
GRID = "#121217"
AXIS = "#2d2d38"
SEL_BG = "#1c2838"
FOOTER_BG = "#18181F"
TARGET_LINE = "#FF6B81"
PATH_LINE = "#2EFF8C"
WHITE = "#FFFFFF"
BLACK = "#000000"
HEADER_SUB = "#1a1a1a"
BACK_FG = "#CCCCCC"

# Field-plot rect + code strip (ui::FX.. / LIST_Y1 / CODE_Y0..)
FX, FY, FW, FH = 160, 58, 310, 110
FIELD_HALF = 72.0
LIST_Y1 = FY + FH
CODE_Y0, CODE_Y1 = LIST_Y1 + 4, LIST_Y1 + 4 + 40

# Grid (GRID_*, kScrollUp/Down, kBack)
GRID_X0, GRID_X1 = 15, 428
GRID_Y0, GRID_Y1 = 58, 210
GRID_COLS, GRID_ROW_H, GRID_GAP = 3, 70, 8
K_SCROLL_UP = (435, 60, 475, 130)
K_SCROLL_DOWN = (435, 140, 475, 210)
K_BACK = (396, 14, 468, 40)
K_MOTOR_LEFT = (10, 58, 233, 210)
K_MOTOR_RIGHT = (247, 58, 470, 210)
K_BRAIN_EDITOR = (10, 140, 150, 166)

# Which header title/colour each screen tag draws with (drawHome() etc.).
# EDIT's title is the motion name, which arrives as the breadcrumb.
SCREEN_HEADER = {
    "HOME": ("DRIVER MENU", None, CYAN),
    "PATH_TYPE": ("PATH PLANNER", None, ORANGE),
    "ANGULAR_LIST": ("PATH PLANNER", "ANGULAR", CYAN),
    "LATERAL_LIST": ("PATH PLANNER", "LATERAL", ORANGE),
    "MOTOR_TEST": ("TEST MOTORS", "DRIVETRAIN", CYAN),
}
ANGULAR_MOTIONS = ("TURN TO HEADING", "TURN TO POINT", "SWING TO HEADING", "SWING TO POINT")

# pros::E_TEXT_SMALL / E_TEXT_MEDIUM -- the brain's fonts are a proportional
# sans at roughly these pixel heights (12px line pitch for SMALL, per the
# 12px row spacing the brain uses for stacked SMALL text).
FONT_SMALL = ("Segoe UI", -11 * SCALE)
FONT_MEDIUM = ("Segoe UI", -16 * SCALE)


class BrainScreen:
    """pros::screen-shaped primitives on a Tk canvas (inclusive pixel
    coordinates, like the brain's), plus the ported draw functions."""

    def __init__(self, canvas):
        self.c = canvas
        self.pen = WHITE

    # -- primitives (pros::screen::*) --
    def set_pen(self, color):
        self.pen = color

    def fill_rect(self, x0, y0, x1, y1):
        self.c.create_rectangle(x0 * SCALE, y0 * SCALE, (x1 + 1) * SCALE, (y1 + 1) * SCALE,
                                fill=self.pen, outline="")

    def draw_rect(self, x0, y0, x1, y1):
        self.c.create_rectangle(x0 * SCALE, y0 * SCALE, (x1 + 1) * SCALE - 1,
                                (y1 + 1) * SCALE - 1, outline=self.pen, width=SCALE)

    def fill_circle(self, x, y, r):
        self.c.create_oval((x - r) * SCALE, (y - r) * SCALE, (x + r + 1) * SCALE,
                           (y + r + 1) * SCALE, fill=self.pen, outline="")

    def draw_circle(self, x, y, r):
        self.c.create_oval((x - r) * SCALE, (y - r) * SCALE, (x + r + 1) * SCALE,
                           (y + r + 1) * SCALE, outline=self.pen, width=SCALE)

    def draw_line(self, x0, y0, x1, y1):
        self.c.create_line(x0 * SCALE, y0 * SCALE, x1 * SCALE, y1 * SCALE, fill=self.pen,
                           width=SCALE)

    def print(self, font, x, y, text):
        self.c.create_text(x * SCALE, y * SCALE, anchor="nw", text=text, fill=self.pen, font=font)

    # -- helpers (fillRoundedRect, clearScreen, drawHeader, drawBackButton) --
    def fillRoundedRect(self, x0, y0, x1, y1, r, color):
        self.set_pen(color)
        self.fill_rect(x0 + r, y0, x1 - r, y1)
        self.fill_rect(x0, y0 + r, x1, y1 - r)
        self.fill_circle(x0 + r, y0 + r, r)
        self.fill_circle(x1 - r, y0 + r, r)
        self.fill_circle(x0 + r, y1 - r, r)
        self.fill_circle(x1 - r, y1 - r, r)

    def clearScreen(self):
        self.set_pen(BG)
        self.fill_rect(0, 0, 480, 240)

    def drawHeader(self, title, breadcrumb, color):
        self.fillRoundedRect(5, 5, 475, 50, 8, color)
        self.set_pen(BLACK)
        self.print(FONT_MEDIUM, 20, 12 if breadcrumb else 18, title)
        if breadcrumb:
            self.set_pen(HEADER_SUB)
            self.print(FONT_SMALL, 20, 30, breadcrumb)

    def drawBackButton(self):
        x0, y0, x1, y1 = K_BACK
        self.fillRoundedRect(x0, y0, x1, y1, 6, BG)
        self.set_pen(BACK_FG)
        self.print(FONT_SMALL, x0 + 12, y0 + 8, "< BACK")

    # -- field plot (toPixel, drawFieldFrame, drawCrosshair, drawHeadingIndicator) --
    def toPixel(self, x, y):
        ppi = (FH / 2.0) / FIELD_HALF
        dx = max(-(FW // 2 - 4), min(FW // 2 - 4, x * ppi))
        dy = max(-(FH // 2 - 4), min(FH // 2 - 4, y * ppi))
        return FX + FW // 2 + int(dx), FY + FH // 2 - int(dy)

    def drawFieldFrame(self):
        self.set_pen(CARD)
        self.fill_rect(FX, FY, FX + FW, FY + FH)
        v = -FIELD_HALF
        while v <= FIELD_HALF + 0.5:
            px, _ = self.toPixel(v, 0)
            self.set_pen(AXIS if abs(v) < 0.5 else GRID)
            self.draw_line(px, FY, px, FY + FH)
            _, py = self.toPixel(0, v)
            self.set_pen(AXIS if abs(v) < 0.5 else GRID)
            self.draw_line(FX, py, FX + FW, py)
            v += 24.0

    def drawCrosshair(self, x, y):
        px, py = self.toPixel(x, y)
        self.set_pen(TARGET_LINE)
        self.draw_line(px - 6, py, px + 6, py)
        self.draw_line(px, py - 6, px, py + 6)
        self.draw_circle(px, py, 3)

    def drawHeadingIndicator(self, ox_in, oy_in, headingDeg):
        kLen = 24.0
        rad = math.radians(headingDeg)
        ox, oy = self.toPixel(ox_in, oy_in)
        tx, ty = self.toPixel(ox_in + math.sin(rad) * kLen, oy_in + math.cos(rad) * kLen)
        self.set_pen(TARGET_LINE)
        self.draw_line(ox, oy, tx, ty)
        self.draw_circle(ox, oy, 3)
        self.draw_circle(tx, ty, 3)

    def drawTargetIndicator(self, plot):
        if plot["hasPt"]:
            self.drawCrosshair(plot["x"], plot["y"])
            if plot["hasHdg"]:
                self.drawHeadingIndicator(plot["x"], plot["y"], plot["theta"])
        elif plot["hasHdg"]:
            self.drawHeadingIndicator(plot["poseX"], plot["poseY"], plot["theta"])

    # -- grid (drawGridButton, drawGrid) --
    def drawGridButton(self, r, label):
        x0, y0, x1, y1 = r
        self.fillRoundedRect(x0, y0, x1, y1, 8, CARD)
        self.fillRoundedRect(x0, y0, x0 + 5, y1, 3, CYAN)
        self.set_pen(AXIS)
        self.draw_rect(x0, y0, x1, y1)
        self.set_pen(WHITE)
        self.print(FONT_SMALL, x0 + 12, y0 + (y1 - y0) // 2 - 6, label)
        self.set_pen(CYAN)
        self.print(FONT_SMALL, x1 - 16, y0 + (y1 - y0) // 2 - 6, ">")

    def drawGrid(self, elements, canUp, canDown):
        # The brain sends each visible button's rect already laid out by
        # gridItemRect(); "^"/"v" entries are the (tappable) scroll arrows.
        for (x0, y0, x1, y1, label) in elements:
            if label in ("BACK", "^", "v"):
                continue
            self.drawGridButton((x0, y0, x1, y1), label)
        for r, can, glyph in ((K_SCROLL_UP, canUp, "^"), (K_SCROLL_DOWN, canDown, "v")):
            self.fillRoundedRect(*r, 8, CARD)
            self.set_pen(CYAN if can else AXIS)
            self.print(FONT_MEDIUM, r[0] + 28, r[1] + 28, glyph)

    # -- EDIT (drawEditUI, drawCodePreview, drawFooter) --
    def drawFooter(self, text, cancelled):
        self.set_pen(FOOTER_BG)
        self.fill_rect(0, 215, 480, 240)
        self.set_pen(TARGET_LINE if cancelled else GRAY)
        self.print(FONT_SMALL, 10, 222, text)

    def drawCodePreview(self, lines):
        self.set_pen(CARD)
        self.fill_rect(10, CODE_Y0, 470, CODE_Y1)
        self.set_pen(AXIS)
        self.draw_rect(10, CODE_Y0, 470, CODE_Y1)
        self.set_pen(GREEN)
        for i, line in enumerate(lines[:3]):
            if line:
                self.print(FONT_SMALL, 16, CODE_Y0 + 3 + i * 12, line)

    def drawEditUI(self, st):
        name = st["breadcrumb"]
        angular = name in ANGULAR_MOTIONS
        self.clearScreen()
        self.drawHeader(name, "ANGULAR" if angular else "LATERAL", CYAN if angular else ORANGE)
        self.drawBackButton()

        fields = st["fields"]
        selected = st["selected"]
        listX0, listX1, listY0, listY1 = 10, 150, FY, LIST_Y1
        rowH = 22
        visibleRows = (listY1 - listY0) // rowH
        maxScroll = max(0, len(fields) - visibleRows)
        scrollIdx = max(0, min(maxScroll, selected - visibleRows // 2))
        for row in range(visibleRows):
            i = scrollIdx + row
            if i >= len(fields):
                break
            y = listY0 + row * rowH
            label, value, _ = fields[i]
            sel = (i == selected)
            self.set_pen(SEL_BG if sel else CARD)
            self.fill_rect(listX0, y, listX1, y + rowH - 3)
            if sel:
                self.set_pen(ORANGE)
                self.fill_rect(listX0, y, listX0 + 4, y + rowH - 3)
            self.set_pen(WHITE if sel else GRAY)
            self.print(FONT_SMALL, listX0 + 8, y + 3, label)
            self.set_pen(CYAN if sel else GRAY)
            self.print(FONT_SMALL, listX0 + 62, y + 3, value)

        # The plot: what drawEditUI() drew, plus whatever runMotion() has
        # painted over it since (trail / live heading) -- see remotePlotMode.
        self.drawFieldFrame()
        plot = st["plot"]
        if plot:
            if plot["mode"] == 2:
                self.drawHeadingIndicator(plot["poseX"], plot["poseY"], plot["poseTheta"])
            else:
                self.drawTargetIndicator(plot)
                if plot["mode"] == 1 and len(st["trail"]) > 1:
                    self.set_pen(PATH_LINE)
                    prev = st["trail"][0]
                    for pt in st["trail"][1:]:
                        self.draw_line(prev[0], prev[1], pt[0], pt[1])
                        prev = pt

        self.set_pen(GRAY)
        self.print(FONT_SMALL, FX + FW - 160, FY + 2, "</>:field ^/v:adj")
        self.print(FONT_SMALL, FX + FW - 160, FY + 14, f"L1/L2:digit  {st['step']:.2f}")
        self.print(FONT_SMALL, FX + FW - 160, FY + 26, "A:go X:cancel B:reset")

        self.drawCodePreview(st["code"])
        self.drawFooter(st["footer"], st["footerCancelled"])

    # -- MOTOR_TEST (drawMotorTestPanel, drawMotorTestScreen) --
    def drawMotorTestPanel(self, r, title, accent, results, activeIdx):
        x0, y0, x1, y1 = r
        self.fillRoundedRect(x0, y0, x1, y1, 8, CARD)
        self.fillRoundedRect(x0, y0, x0 + 5, y1, 3, accent)
        self.set_pen(AXIS)
        self.draw_rect(x0, y0, x1, y1)
        self.set_pen(WHITE)
        self.print(FONT_MEDIUM, x0 + 14, y0 + 8, title)
        rowY = y0 + 34
        for i, (port, state) in enumerate(results):
            if state != "pending":
                self.set_pen({"No spin": ORANGE, "Negative": TARGET_LINE}.get(state, GREEN))
                line = f"Port {port} - {state}"
            elif i == activeIdx:
                self.set_pen(CYAN)
                line = f"Port {port} - testing..."
            else:
                self.set_pen(GRAY)
                line = f"Port {port} - pending"
            self.print(FONT_SMALL, x0 + 14, rowY, line)
            rowY += 22

    def drawMotorTestScreen(self, st):
        self.clearScreen()
        self.drawHeader(*SCREEN_HEADER["MOTOR_TEST"])
        self.drawBackButton()
        side, idx = st["activeSide"], st["activeIdx"]
        self.drawMotorTestPanel(K_MOTOR_LEFT, "LEFT DRIVETRAIN", CYAN, st["motors"]["L"],
                                idx if side == 0 else -1)
        self.drawMotorTestPanel(K_MOTOR_RIGHT, "RIGHT DRIVETRAIN", ORANGE, st["motors"]["R"],
                                idx if side == 1 else -1)
        self.drawFooter(st["footer"], False)

    # -- PLANNER (drawPlannerScreen) --
    def drawPlannerScreen(self, st):
        self.clearScreen()
        self.drawHeader("PATH PLANNER", "LAPTOP", ORANGE)
        self.drawBackButton()
        status, idx, count = st["plan"] or ("IDLE", -1, 0)
        self.set_pen(WHITE)
        self.print(FONT_SMALL, 10, FY + 2, "Plan on the laptop:")
        self.set_pen(GRAY)
        self.print(FONT_SMALL, 10, FY + 14, "tools/remote_touch.py")
        self.set_pen(WHITE)
        self.print(FONT_SMALL, 10, FY + 34, f"{count} step{'' if count == 1 else 's'}")
        self.set_pen({"CANCELLED": TARGET_LINE, "RUNNING": CYAN}.get(status, GRAY))
        self.print(FONT_SMALL, 10, FY + 46, f"{status} {idx + 1}/{count}" if idx >= 0 else status)
        x0, y0, x1, y1 = K_BRAIN_EDITOR
        self.fillRoundedRect(x0, y0, x1, y1, 6, CARD)
        self.set_pen(CYAN)
        self.print(FONT_SMALL, x0 + 10, y0 + 7, "BRAIN EDITOR >")

        self.drawFieldFrame()
        if len(st["trail"]) > 1:
            self.set_pen(PATH_LINE)
            prev = st["trail"][0]
            for pt in st["trail"][1:]:
                self.draw_line(prev[0], prev[1], pt[0], pt[1])
                prev = pt
        plot = st["plot"]
        if plot:
            self.drawHeadingIndicator(plot["poseX"], plot["poseY"], plot["poseTheta"])

        self.set_pen(CARD)
        self.fill_rect(10, CODE_Y0, 470, CODE_Y1)
        self.set_pen(AXIS)
        self.draw_rect(10, CODE_Y0, 470, CODE_Y1)
        self.set_pen(GREEN)
        for i, line in enumerate(st["code"][:3]):
            if line:
                self.print(FONT_SMALL, 16, CODE_Y0 + 3 + i * 12, line)
        self.drawFooter(st["footer"], False)

    # -- handoff notice (the brain screen belongs to something we don't mirror) --
    def drawHandoff(self, what):
        self.clearScreen()
        self.drawHeader("NOT MIRRORED", what, GRAY)
        self.set_pen(GRAY)
        if what == "DRIVING":
            msg = "Driver menu closed -- robot is in normal driving."
        else:
            msg = f"{what} owns the brain screen. Tap its BACK on the brain to return."
        self.print(FONT_SMALL, 20, 110, msg)

    # -- top level: which screen --
    def draw(self, st):
        tag = st["screen"]
        if tag == "EDIT":
            self.drawEditUI(st)
        elif tag == "MOTOR_TEST":
            self.drawMotorTestScreen(st)
        elif tag == "PLANNER":
            self.drawPlannerScreen(st)
        elif tag == "HANDOFF":
            self.drawHandoff(st["breadcrumb"])
        elif tag in SCREEN_HEADER:
            self.clearScreen()
            self.drawHeader(*SCREEN_HEADER[tag])
            if tag != "HOME":
                self.drawBackButton()
            self.drawGrid(st["elements"], st["canUp"], st["canDown"])
        else:
            self.clearScreen()
            self.set_pen(GRAY)
            self.print(FONT_SMALL, 20, 110, f"unknown screen '{tag}'")


class RemoteTouchApp:
    def __init__(self, root, link):
        self.root = root
        self.link = link
        self.elements = []  # last-drawn (x0,y0,x1,y1,label) in brain coords

        root.title("Remote Touch -- V5 Brain")
        self.status = tk.Label(root, text="waiting for the brain...", anchor="w",
                                font=("Segoe UI", 9))
        self.status.pack(fill="x", padx=6, pady=(6, 0))

        self.canvas = tk.Canvas(root, width=BRAIN_W * SCALE, height=BRAIN_H * SCALE,
                                 bg="#0A0A0F", highlightthickness=0)
        self.canvas.pack(padx=6, pady=6)
        self.canvas.bind("<Button-1>", self.onClick)
        self.canvas.bind("<MouseWheel>", self.onWheel)

        # Value box: type a number (or choice label) and press Enter to
        # set the selected EDIT field. Digits typed while the mirror has
        # focus jump here automatically.
        bar = tk.Frame(root)
        bar.pack(fill="x", padx=6)
        self.valueLabel = tk.Label(bar, text="value:", font=("Segoe UI", 9), width=18, anchor="w")
        self.valueLabel.pack(side="left")
        self.valueVar = tk.StringVar()
        self.entry = tk.Entry(bar, textvariable=self.valueVar, font=("Consolas", 10), width=16)
        self.entry.pack(side="left", padx=(0, 6))
        self.entry.bind("<Return>", self.onSetValue)
        self.entry.bind("<Escape>", lambda e: self.canvas.focus_set())
        tk.Button(bar, text="Set", command=self.onSetValue).pack(side="left")
        for name, key in (("Run (A)", "A"), ("Cancel (X)", "X"), ("Reset pose (B)", "B")):
            tk.Button(bar, text=name, command=lambda k=key: self.sendKey(k)).pack(side="left",
                                                                                   padx=(6, 0))

        self.hint = tk.Label(
            root,
            text="Click a button to tap it on the brain.  EDIT screen: arrows = field/nudge, "
                 "PgUp/PgDn = digit step, A/X/B = run/cancel/reset, Esc = BACK, "
                 "click a field row to select it, type a value + Enter to set it.",
            anchor="w", justify="left", wraplength=BRAIN_W * SCALE, font=("Segoe UI", 8),
            fg="#666666")
        self.hint.pack(fill="x", padx=6, pady=(4, 6))

        # Keyboard -> controller buttons. Bound on the canvas (not the
        # root) so the value box keeps its own arrow keys / letters.
        self.lastState = None
        for keysym, name in (("Left", "LEFT"), ("Right", "RIGHT"), ("Up", "UP"),
                             ("Down", "DOWN"), ("Prior", "L1"), ("Next", "L2"),
                             ("a", "A"), ("A", "A"), ("x", "X"), ("X", "X"), ("b", "B"),
                             ("B", "B")):
            self.canvas.bind(f"<KeyPress-{keysym}>", lambda e, n=name: self.sendKey(n))
        self.canvas.bind("<KeyPress-Escape>", self.onEscape)
        self.canvas.bind("<KeyPress>", self.onCanvasKey)
        self.canvas.focus_set()

        self.startedAt = time.time()
        self.gotState = False
        self.planner = None
        self.pollQueue()

    def pollQueue(self):
        state = None
        try:
            while True:  # drain to the latest state; older ones are stale
                state = self.link.states.get_nowait()
        except queue.Empty:
            pass
        if state:
            self.gotState = True
            self.lastState = state
            self.render(state)
            self.updateValueLabel(state)
            self.updatePlanner(state)
        elif not self.gotState and time.time() - self.startedAt > 5:
            self.showWaitingHint()
        self.root.after(50, self.pollQueue)

    def showWaitingHint(self):
        """Nothing usable has arrived yet -- say which of the usual
        suspects it looks like, based on what the port is doing."""
        if getattr(self.link, "error", None):
            msg = self.link.error
        elif self.link.bytesReceived == 0:
            msg = ("nothing received. Is the program running on the brain? Is this the "
                   "right COM port (a brain shows two; pick the user one)? "
                   "If plugged into the controller: is it linked to the brain?")
        elif self.link.linesReceived == 0:
            msg = "receiving bytes but no text lines -- wrong COM port or baud?"
        else:
            msg = (f"receiving output ({self.link.linesReceived} lines) but no RUI| lines -- "
                   "is the driver menu open on the brain? (run with --debug to see them)")
        self.status.config(text=msg)

    def render(self, state):
        self.elements = state["elements"]
        self.status.config(
            text=f"{state['screen']}" +
                 (f"  /  {state['breadcrumb']}" if state["breadcrumb"] not in ("", "-") else ""))
        self.canvas.delete("all")
        BrainScreen(self.canvas).draw(state)

    # -- laptop path planner window --
    def updatePlanner(self, state):
        """The brain's PATH PLANNER screen is planned from a separate, big
        window (tools/path_planner.py): open it the first time the brain
        gets there, and keep it fed with every state after that."""
        if state["screen"] == "PLANNER" and self.planner is None:
            try:
                from path_planner import PlannerWindow
            except ImportError as e:
                self.status.config(text=f"path_planner.py failed to load: {e}")
                return
            self.planner = PlannerWindow(self.root, self.link)
        if self.planner is not None:
            if state["screen"] == "PLANNER" and self.planner.state() == "withdrawn":
                self.planner.deiconify()
            self.planner.applyState(state)

    # -- keyboard / value entry --
    def sendKey(self, name):
        self.link.sendKey(name)

    def onEscape(self, _event=None):
        for (x0, y0, x1, y1, label) in self.elements:
            if label == "BACK":
                self.link.sendTouch((x0 + x1) // 2, (y0 + y1) // 2)
                self.flash(x0, y0, x1, y1)
                return

    def onCanvasKey(self, event):
        # A digit / sign / dot typed on the mirror starts a value entry.
        if event.char and (event.char.isdigit() or event.char in "-."):
            self.valueVar.set(event.char)
            self.entry.focus_set()
            self.entry.icursor("end")

    def onWheel(self, event):
        self.sendKey("UP" if event.delta > 0 else "DOWN")

    def updateValueLabel(self, state):
        if state["screen"] == "EDIT" and state["fields"]:
            i = max(0, min(len(state["fields"]) - 1, state["selected"]))
            label, value, _ = state["fields"][i]
            self.valueLabel.config(text=f"{label} = {value}")
            self.entry.config(state="normal")
        else:
            self.valueLabel.config(text="value: (EDIT screen only)")
            self.entry.config(state="disabled")

    def onSetValue(self, _event=None):
        st = self.lastState
        text = self.valueVar.get().strip()
        if not st or st["screen"] != "EDIT" or not st["fields"] or not text:
            return
        i = max(0, min(len(st["fields"]) - 1, st["selected"]))
        self.link.sendSet(i, text)
        self.valueVar.set("")
        self.canvas.focus_set()

    # Same row layout as drawEditUI() -- so a click on a field row picks
    # the field the brain drew there, scroll and all.
    def fieldRowAt(self, x, y):
        st = self.lastState
        if not st or st["screen"] != "EDIT" or not st["fields"]:
            return None
        listX0, listX1, listY0, listY1 = 10, 150, FY, LIST_Y1
        rowH = 22
        if not (listX0 <= x <= listX1 and listY0 <= y < listY1):
            return None
        visibleRows = (listY1 - listY0) // rowH
        maxScroll = max(0, len(st["fields"]) - visibleRows)
        scrollIdx = max(0, min(maxScroll, st["selected"] - visibleRows // 2))
        i = scrollIdx + (y - listY0) // rowH
        return i if i < len(st["fields"]) else None

    def onClick(self, event):
        self.canvas.focus_set()
        x, y = event.x // SCALE, event.y // SCALE
        row = self.fieldRowAt(x, y)
        if row is not None:
            self.link.sendSelect(row)  # the brain redraws with the new highlight
            return
        for (x0, y0, x1, y1, label) in self.elements:
            if x0 <= x <= x1 and y0 <= y <= y1:
                # Send the button's own center -- more reliable than the
                # raw click point, and matches how a fingertip tap tends
                # to land anyway.
                cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
                self.link.sendTouch(cx, cy)
                self.flash(x0, y0, x1, y1)
                return

    def flash(self, x0, y0, x1, y1):
        rect = self.canvas.create_rectangle(x0 * SCALE, y0 * SCALE, x1 * SCALE, y1 * SCALE,
                                             outline="#2EFF8C", width=3)
        self.root.after(150, lambda: self.canvas.delete(rect))


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # for path_planner
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="Serial port, e.g. COM5 (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--debug", action="store_true",
                    help="print every line received from the brain to the console")
    ap.add_argument("--via", choices=("auto", "serial", "vexcom"), default="auto",
                    help="serial = cable to the brain; vexcom = through the controller's "
                         "radio via vexcom --user (default: auto-detect from the USB device)")
    ap.add_argument("--vexcom", help="path to vexcom.exe (default: PROS extension's copy)")
    ap.add_argument("--rate", type=int, default=None,
                    help="minimum ms between screen updates from the brain (default 150 on "
                         "cable, 300 over the controller; unchanged screens only send a 1 s "
                         "heartbeat anyway)")
    args = ap.parse_args()

    port = args.port or guess_port()
    if not port:
        print("Couldn't auto-detect a serial port. Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  -  {p.description}")
        print("\nRe-run with the right one, e.g.: python tools/remote_touch.py COM5")
        sys.exit(1)

    via = args.via
    if via == "auto":
        via = "vexcom" if is_controller_port(port) else "serial"

    if via == "vexcom":
        vexcom = args.vexcom or find_vexcom()
        if not vexcom:
            print("This port is a V5 controller, so the brain can only be reached through "
                  "VEX's vexcom tool -- and I couldn't find vexcom.exe.")
            print("Install the PROS VS Code extension, or pass --vexcom PATH\\TO\\vexcom.exe")
            sys.exit(1)
        print(f"Connecting to {port} through the controller's radio (vexcom --user)...")
        print("(close the PROS terminal / any other vexcom first -- only one can use the port)")
        link = VexcomLink(port, vexcom, debug=args.debug)
        rate = args.rate or 300
    else:
        print(f"Connecting to {port} @ {args.baud}...")
        try:
            link = SerialLink(port, args.baud, debug=args.debug)
        except serial.SerialException as e:
            print(f"Couldn't open {port}: {e}")
            print("Is `pros terminal` (or another serial monitor) still open on this port?")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}  -  {p.description}  ({p.location or ''})")
            sys.exit(1)
        rate = args.rate or 150
    link.sendRate(rate)

    root = tk.Tk()
    app = RemoteTouchApp(root, link)
    try:
        root.mainloop()
    finally:
        link.close()


if __name__ == "__main__":
    main()
