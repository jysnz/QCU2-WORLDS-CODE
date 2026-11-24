#!/usr/bin/env python3
"""
Lift Tuner -- the laptop half of the robot's LIFT TUNER (hold DPAD-LEFT when
driver control starts to open the driver menu, then tap LIFT TUNER).

    python tools/lift_tuner.py           connect to the brain over the USB cable
    python tools/lift_tuner.py --demo    no robot: a fake lift, to try the layout

Needs pyserial (pip install pyserial). Close `pros terminal` first -- only one
program can hold the brain's serial port.

WHAT YOU CAN SET (four tabs)
    Start & limits      START POSITION: the angle the lift is assumed to be at when
                        driver control starts. FORWARD LIMIT: how far X may drive it.
    Combos              Your library of named combos -- add as many as you like and
                        edit any of them any time. A combo is a list of steps on any
                        motor or piston (left/right drive, intake, lift, claw arm,
                        claw piston, lift-arm orientation). Each
                        combo is saved into the code as a function, e.g. combo_Score().
    Scoring levels      Named lift heights; each one plays a combo of your choice once
                        the lift gets there (UP / LEFT on the controller step through
                        the levels).
    Angle functions     "When the lift passes ANGLE going up / down / either, play this
                        COMBO (or one step)". Optionally tied to a controller button:
                        HOLD = only while it is held, TAP = tap it to arm, then it fires
                        once when the lift gets there. A combo that moves the lift (e.g.
                        lift BACKWARD) wins over a held R1.

THE WORKFLOW
    1. Move the lift (jog buttons, click the gauge, or X / B in the on-robot tuner) and
       press "Use live" next to a number to capture the lift's real angle.
    2. Build combos, pick them for levels. Test buttons run them on the robot.
    3. APPLY TO ROBOT  (Ctrl+Enter) sends everything to the brain to try it.
    4. SAVE  (Ctrl+S) applies it, stores it on the brain's SD card AND writes it into
       the code (src/lift_tuning.cpp + include/lift_tuning.hpp: defaults plus one
       function per combo). Rebuild and upload afterwards.
    STOP (Esc) halts the lift and every running step. Ctrl+1..4 switch tabs.

Wire format: see the top of src/lift_tuning.cpp.
"""

import argparse
import copy
import json
import math
import os
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import font as tkfont, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(os.path.dirname(HERE), "src", "lift_tuning.cpp")
BACKUP = os.path.join(HERE, "lift_tuning.json")
HEADER_H = os.path.join(os.path.dirname(HERE), "include", "lift_tuning.hpp")
BEGIN, END = "// LIFT-TUNING-BEGIN", "// LIFT-TUNING-END"
FUNCS_BEGIN, FUNCS_END = "// LIFT-COMBO-FUNCS-BEGIN", "// LIFT-COMBO-FUNCS-END"
DECLS_BEGIN, DECLS_END = "// LIFT-COMBO-DECLS-BEGIN", "// LIFT-COMBO-DECLS-END"

# -- design system --------------------------------------------------------------
# Raw values live in this block and nowhere else. Widgets use the semantic roles on
# C, the spacing helper S() and the type helpers fnt()/mono() -- never a hex literal.
class C:
    # surfaces, darkest to lightest
    sunken = "#0a0e14"          # input wells
    base = "#0e131b"            # window and tab pages
    panel = "#141b26"           # header, status bar, gauge column, tables
    raised = "#1b2431"          # secondary buttons
    hover = "#243040"
    pressed = "#2d3b4f"
    line = "#222d3c"            # hairlines
    line_strong = "#35455c"     # button and table borders
    field_edge = "#64748c"      # input borders: 3:1 against the surface so a field is findable
    # text
    text = "#e6edf3"
    muted = "#a0acbe"
    subtle = "#6f7d92"          # disabled text only
    on_fill = "#ffffff"
    # accent: primary action, selection, focus
    accent = "#5aa7ff"
    accent_fill = "#2a63c7"
    accent_hover = "#2f6ed6"
    accent_press = "#2152a8"
    accent_subtle = "#17345a"   # selected row
    focus = "#8cc2ff"
    # state
    success = "#46c15c"
    warning = "#f0b03a"
    danger = "#ff6b63"
    danger_fill = "#b8302b"
    danger_hover = "#c9392f"
    danger_press = "#9a2823"
    danger_subtle = "#3a171a"
    # gauge
    marker = "#f2c230"          # start position
    fill_idle = "#4f6384"


T = {"caption": 9, "body": 10, "heading": 11, "title": 13, "readout": 26}   # pt, ratio ~1.15
SP = {"xs": 4, "sm": 8, "md": 12, "lg": 16, "xl": 24}                       # px at 96 dpi

SCALE = 1.0
UI_FONT, MONO_FONT, ICON_FONT = "Segoe UI", "Consolas", None
# Segoe Fluent Icons (Win 11) / Segoe MDL2 Assets (Win 10): the platform's own icon font.
ICONS = {"add": "", "delete": "", "copy": "", "play": "", "stop": "",
         "save": "", "refresh": "", "up": "", "down": "",
         "download": "", "upload": "", "go": ""}
MAX_COMBOS, MAX_LEVELS, MAX_COMBO_STEPS, MAX_ACTIONS = 8, 8, 6, 8


def S(n):
    """Scale a pixel measurement designed at 96 dpi to this display."""
    return int(round(n * SCALE)) if n else 0


def fnt(size=T["body"], weight="normal"):
    return (UI_FONT, size, weight)


def mono(size=11, weight="normal"):
    return (MONO_FONT, size, weight)

DEVICES = ["Left drive", "Right drive", "Intake", "Lift", "Claw arm",
           "Claw piston", "Lift-arm orientation"]   # index == LiftDevice
FIRST_PISTON = 5
MOTOR_CMDS = [("Forward", 0), ("Backward", 1), ("Stop", 2)]
PISTON_CMDS = [("Extend", 3), ("Retract", 4), ("Toggle", 5)]
WHENS = ["Going up", "Going down", "Either way"]
BUTTONS = ["Any (no button)", "R1", "R2", "L1", "L2", "Up", "Down", "Left", "Right", "X", "B", "Y", "A"]  # index == LiftButton
BTN_MODES = ["Hold", "Tap"]
SINGLE_STEP = "A single step (set below)"
JOGS = (-25, -5, -1, 1, 5, 25)


def is_piston(dev):
    return dev >= FIRST_PISTON


def cmd_name(dev, cmd):
    for name, v in (PISTON_CMDS if is_piston(dev) else MOTOR_CMDS):
        if v == cmd:
            return name
    return "?"


def new_step(**kw):
    s = {"delay": 0, "dev": 4, "cmd": 0, "speed": 100, "dur": 500}
    s.update(kw)
    return s


def new_action(angle=0.0, when=0, step=None, combo=-1, button=0, mode=0):
    """An angle function: at ANGLE play COMBO (or the single STEP when combo is -1), optionally only
    while BUTTON is held (mode 0) or after it was tapped (mode 1)."""
    return {"angle": angle, "when": when, "step": step or new_step(), "combo": combo, "button": button, "mode": mode}


def normalize_model(m):
    """Fills in the fields that settings saved by an older version do not have."""
    for a in m.get("actions", []):
        a.setdefault("combo", -1)
        a.setdefault("button", 0)
        a.setdefault("mode", 0)
    return m


def default_model():
    return {"startEnabled": False, "start": 0.0, "limit": 743.2, "combos": [],
            "levels": [{"name": n, "angle": a, "combo": -1}
                       for n, a in (("Low", 150.0), ("Mid", 400.0), ("High", 700.0))],
            "actions": []}


def sample_model():
    """What --demo opens with: a filled-in example so every tab has something to look at."""
    return {"startEnabled": True, "start": 42.0, "limit": 743.2,
            "combos": [
                {"name": "Release", "steps": [new_step(dev=5, cmd=4)]},
                {"name": "ScoreLow", "steps": [
                    new_step(dev=4, cmd=0, speed=100, dur=400),
                    new_step(dev=5, cmd=4, delay=400)]},
                {"name": "ScoreMid", "steps": [
                    new_step(dev=6, cmd=3),
                    new_step(dev=4, cmd=0, speed=120, dur=600, delay=300),
                    new_step(dev=5, cmd=4, delay=600),
                    new_step(dev=2, cmd=1, speed=200, dur=800)]},
                {"name": "LiftDrop", "steps": [
                    new_step(dev=3, cmd=1, speed=200, dur=600),
                    new_step(dev=2, cmd=1, speed=200, dur=800)]}],
            "levels": [
                {"name": "Ground", "angle": 0.0, "combo": 0},
                {"name": "Low", "angle": 180.0, "combo": 1},
                {"name": "Mid", "angle": 420.0, "combo": 2},
                {"name": "High", "angle": 700.0, "combo": -1}],
            "actions": [
                new_action(300.0, 0, new_step(dev=6, cmd=3)),
                new_action(200.0, 1, new_step(dev=6, cmd=4)),
                new_action(650.0, 0, combo=3, button=1, mode=0)]}


def step_summary(s, with_delay=True):
    what = f"{DEVICES[s['dev']]}: {cmd_name(s['dev'], s['cmd'])}"
    if not is_piston(s["dev"]) and s["cmd"] != 2:
        what += f" {s['speed']} rpm for {s['dur']} ms"
    if with_delay:
        what = (f"+{s['delay']} ms  " if s["delay"] else "now  ") + what
    return what


def action_summary(a, combo_names):
    """'Then do' text of an angle function: its combo, or its single step."""
    c = a.get("combo", -1)
    return f"Combo {combo_names[c]}" if 0 <= c < len(combo_names) else step_summary(a["step"], False)


def button_summary(a):
    b = a.get("button", 0)
    return BUTTONS[b].split(" ")[0] if not b else f"{BUTTONS[b]} {BTN_MODES[a.get('mode', 0)].lower()}"


# -- wire format --------------------------------------------------------------
def clean_name(name):
    return re.sub(r"[^A-Za-z0-9_-]", "_", name.strip())[:11] or "Level"


def ident(name):
    t = re.sub(r"[^A-Za-z0-9_]", "_", name.strip()) or "Combo"
    return "c" + t if t[0].isdigit() else t


def combo_idents(m):
    """Function name per combo: combo_<Name>, de-duplicated with _2, _3 ..."""
    seen, out = {}, []
    for c in m["combos"]:
        base = ident(c["name"])
        seen[base] = seen.get(base, 0) + 1
        out.append(f"combo_{base}" if seen[base] == 1 else f"combo_{base}_{seen[base]}")
    return out


def model_to_commands(m):
    out = ["LT BEGIN", "LT CLEAR", f"LT START {1 if m['startEnabled'] else 0} {m['start']:.2f}",
           f"LT LIMIT {m['limit']:.2f}"]
    for c in m["combos"]:
        out.append(f"LT COMBO {clean_name(c['name'])}")
        for s in c["steps"]:
            out.append(f"LT STEP {s['delay']} {s['dev']} {s['cmd']} {s['speed']} {s['dur']}")
    for lv in m["levels"]:
        out.append(f"LT LEVEL {lv['angle']:.2f} {clean_name(lv['name'])} {lv['combo']}")
    for a in m["actions"]:
        s = a["step"]
        out.append(f"LT ACT {a['angle']:.2f} {a['when']} {s['dev']} {s['cmd']} {s['speed']} {s['dur']} "
                   f"{a.get('combo', -1)} {a.get('button', 0)} {a.get('mode', 0)}")
    out.append("LT APPLY")
    return out


def parse_ltc(line):
    """'LTC|en,start,limit|combos|levels|actions' -> model, or None."""
    try:
        parts = line.rstrip("\r\n").split("|")
        en, start, limit = parts[1].split(",")
        m = {"startEnabled": en == "1", "start": float(start), "limit": float(limit),
             "combos": [], "levels": [], "actions": []}
        for cb in filter(None, parts[2].split(";")):
            name, steps = cb.split(":", 1)
            sl = []
            for st in filter(None, steps.split("/")):
                d, dev, cmd, sp, du = (int(float(v)) for v in st.split(","))
                sl.append({"delay": d, "dev": dev, "cmd": cmd, "speed": sp, "dur": du})
            m["combos"].append({"name": name, "steps": sl})
        for lv in filter(None, parts[3].split(";")):
            name, angle, combo = lv.split(":")
            m["levels"].append({"name": name, "angle": float(angle), "combo": int(combo)})
        for ac in filter(None, parts[4].split(";")):
            v = ac.split(",")
            ang, when, d, dev, cmd, sp, du = v[:7]
            combo, button, mode = (int(x) for x in v[7:10]) if len(v) >= 10 else (-1, 0, 0)
            m["actions"].append({"angle": float(ang), "when": int(when),
                                 "step": {"delay": int(d), "dev": int(dev), "cmd": int(cmd),
                                          "speed": int(sp), "dur": int(du)},
                                 "combo": combo, "button": button, "mode": mode})
        return m
    except (ValueError, IndexError):
        return None


def parse_ltl(line):
    try:
        v = line.rstrip("\r\n").split("|")[1].split(",")
        return {"live": float(v[0]), "moving": v[1] == "1", "sel": int(v[2]), "level": int(v[3]),
                "combo": v[4] == "1", "saved": int(v[5])}
    except (ValueError, IndexError):
        return None


def code_block(m):
    """The default settings: statements between the LIFT-TUNING markers."""
    L = [f"    t.startEnabled = {'true' if m['startEnabled'] else 'false'};",
         f"    t.startPosition = {m['start']:.1f};", f"    t.forwardLimit = {m['limit']:.1f};",
         f"    t.comboCount = {len(m['combos'])};"]
    for i, c in enumerate(m["combos"]):
        steps = ", ".join(f"{{{s['delay']}, {s['dev']}, {s['cmd']}, {s['speed']}, {s['dur']}}}" for s in c["steps"])
        L.append(f"    t.combos[{i}] = {{\"{clean_name(c['name'])}\", {len(c['steps'])}, {{{steps}}}}};")
    L.append(f"    t.levelCount = {len(m['levels'])};")
    for i, lv in enumerate(m["levels"]):
        L.append(f"    t.levels[{i}] = {{\"{clean_name(lv['name'])}\", {lv['angle']:.1f}, {lv['combo']}}};")
    L.append(f"    t.actionCount = {len(m['actions'])};")
    for i, a in enumerate(m["actions"]):
        s = a["step"]
        L.append(f"    t.actions[{i}] = {{{a['angle']:.1f}, {a['when']}, "
                 f"{{0, {s['dev']}, {s['cmd']}, {s['speed']}, {s['dur']}}}, "
                 f"{a.get('combo', -1)}, {a.get('button', 0)}, {a.get('mode', 0)}}};")
    return "\n".join(L) + "\n"


def combo_functions(m):
    """One blocking C++ function per combo, e.g. combo_Score() -- callable from autonomous()."""
    L = []
    for c, fn in zip(m["combos"], combo_idents(m)):
        L.append(f"void {fn}() {{")
        if c["steps"]:
            L.append("    static const LiftStep steps[] = {")
            for s in c["steps"]:
                L.append(f"        {{{s['delay']}, {s['dev']}, {s['cmd']}, {s['speed']}, {s['dur']}}},  // {step_summary(s)}")
            L.append("    };")
            L.append(f"    liftPlayCombo(steps, {len(c['steps'])});")
        L.append("}")
        L.append("")
    return "\n".join(L) + ("\n" if L else "")


def combo_decls(m):
    return "".join(f"void {fn}();  // {c['name']}\n" for c, fn in zip(m["combos"], combo_idents(m)))


def replace_region(text, begin, end, body, nl, indent=""):
    a, b = text.find(begin), text.find(end)
    if a < 0 or b < a:
        raise ValueError(f"{begin} / {end} not found")
    return text[:a + len(begin)] + nl + body.replace("\n", nl) + indent + text[b:]


# -- links --------------------------------------------------------------------
def cobs_decode(packet):
    out = bytearray()
    i, n = 0, len(packet)
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
    """Raw bytes from the brain's user port -> text lines (PROS COBS-frames stdout)."""
    IDS = (b"sout", b"serr", b"kdbg")

    def __init__(self):
        self.buf = bytearray()
        self.text = ""

    def feed(self, raw):
        self.buf += raw
        while True:
            z = self.buf.find(b"\x00")
            if z == -1:
                break
            pkt = bytes(self.buf[:z])
            del self.buf[:z + 1]
            if not pkt:
                continue
            dec = cobs_decode(pkt)
            if dec is not None and dec[:4] in self.IDS:
                self.text += dec[4:].decode("utf-8", errors="ignore")
            else:
                self.text += pkt.decode("utf-8", errors="ignore")
        if self.buf and b"\n" in self.buf and all(9 <= b < 127 for b in self.buf):
            self.text += bytes(self.buf).decode("utf-8", errors="ignore")
            self.buf.clear()
        lines = self.text.split("\n")
        self.text = lines.pop()
        return lines


def find_brain_port():
    import serial.tools.list_ports
    vex = [p for p in serial.tools.list_ports.comports()
           if p.vid == 0x2888 or "vex" in ((p.description or "") + (p.manufacturer or "")).lower()]
    for p in vex:
        if "user" in (p.description or "").lower():
            return p.device
    if len(vex) >= 2:
        for p in vex:
            if "x.2" in (p.location or ""):
                return p.device
        return vex[1].device
    return vex[0].device if vex else None


class SerialLink:
    name = "brain"

    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.port = port
        self.lines = queue.Queue()
        self.out = queue.Queue()
        self.dead = False
        self.stop = False
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._write, daemon=True).start()

    def _read(self):
        stream = PacketStream()
        while not self.stop:
            try:
                raw = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                self.dead = True
                return
            for line in stream.feed(raw) if raw else []:
                if line.startswith(("LTL|", "LTC|")):
                    self.lines.put(line)

    def _write(self):
        while not self.stop:
            try:
                line = self.out.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.ser.write((line + "\n").encode("ascii", errors="ignore"))
            except Exception:
                self.dead = True
                return
            time.sleep(0.012)   # the brain reads its input a line at a time

    def send(self, lines):
        for l in lines:
            self.out.put(l)

    def close(self):
        self.stop = True
        try:
            self.ser.close()
        except Exception:
            pass


class FakeLink:
    """A pretend robot: a lift that chases GO targets, and a config it echoes back."""
    name = "demo"

    def __init__(self, model):
        self.model = copy.deepcopy(model)
        self.pending = None
        self.live, self.goal = 0.0, None
        self.lines = queue.Queue()
        self.dead = False
        self.t_state = self.t_cfg = 0.0
        self.saved = 0

    def send(self, lines):
        for l in lines:
            m = re.match(r"LT (\w+)\s*(.*)", l)
            if not m:
                continue
            cmd, rest = m.groups()
            pd = self.pending
            if cmd == "BEGIN":
                self.pending = {"startEnabled": False, "start": 0.0, "limit": 743.2,
                                "combos": [], "levels": [], "actions": []}
            elif cmd == "START" and pd is not None:
                en, deg = rest.split()
                pd["startEnabled"], pd["start"] = en == "1", float(deg)
            elif cmd == "LIMIT" and pd is not None:
                pd["limit"] = float(rest)
            elif cmd == "COMBO" and pd is not None:
                pd["combos"].append({"name": rest.split()[0], "steps": []})
            elif cmd == "STEP" and pd is not None and pd["combos"]:
                d, dev, c, sp, du = map(int, rest.split())
                pd["combos"][-1]["steps"].append({"delay": d, "dev": dev, "cmd": c, "speed": sp, "dur": du})
            elif cmd == "LEVEL" and pd is not None:
                a, n, *c = rest.split()
                pd["levels"].append({"name": n, "angle": float(a), "combo": int(c[0]) if c else -1})
            elif cmd == "ACT" and pd is not None:
                v = rest.split()
                a, w, dev, c, sp, du = v[:6]
                ex = [int(x) for x in v[6:9]] if len(v) >= 9 else [-1, 0, 0]
                pd["actions"].append({"angle": float(a), "when": int(w), "step": {
                    "delay": 0, "dev": int(dev), "cmd": int(c), "speed": int(sp), "dur": int(du)},
                    "combo": ex[0], "button": ex[1], "mode": ex[2]})
            elif cmd == "APPLY" and pd is not None:
                self.model, self.pending = pd, None
                self.t_cfg = 0
            elif cmd == "GO":
                self.goal = float(rest)
            elif cmd == "JOG":
                self.goal = self.live + float(rest)
            elif cmd == "STOP":
                self.goal = None
            elif cmd == "SAVE":
                self.saved = 1

    def tick(self):
        now = time.time()
        if self.goal is not None:
            d = self.goal - self.live
            self.live += max(-6.0, min(6.0, d))
            if abs(self.goal - self.live) < 0.5:
                self.goal = None
        if now - self.t_state > 0.15:
            self.t_state = now
            self.lines.put(f"LTL|{self.live:.2f},{1 if self.goal is not None else 0},0,-1,0,{self.saved}")
        if now - self.t_cfg > 1.0:
            self.t_cfg = now
            m = self.model
            cb = ";".join(f"{c['name']}:" + "/".join(
                f"{s['delay']},{s['dev']},{s['cmd']},{s['speed']},{s['dur']}" for s in c["steps"]) for c in m["combos"])
            lv = ";".join(f"{l['name']}:{l['angle']:.2f}:{l['combo']}" for l in m["levels"])
            ac = ";".join(f"{a['angle']:.2f},{a['when']},0,{a['step']['dev']},{a['step']['cmd']},"
                          f"{a['step']['speed']},{a['step']['dur']},{a.get('combo', -1)},{a.get('button', 0)},"
                          f"{a.get('mode', 0)}" for a in m["actions"])
            self.lines.put(f"LTC|{1 if m['startEnabled'] else 0},{m['start']:.2f},{m['limit']:.2f}|{cb}|{lv}|{ac}|")

    def close(self):
        pass


# -- UI -------------------------------------------------------------------------
def enable_hidpi():
    """Crisp text on scaled Windows displays. Call before the first Tk()."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass


def init_look(root):
    """Pick fonts that exist on this machine, set the display scale, theme ttk."""
    global SCALE, UI_FONT, MONO_FONT, ICON_FONT
    SCALE = max(1.0, root.winfo_fpixels("1i") / 96.0)
    fams = set(tkfont.families(root))
    UI_FONT = next((f for f in ("Segoe UI", "Helvetica Neue", "Arial") if f in fams), "TkDefaultFont")
    MONO_FONT = next((f for f in ("Cascadia Mono", "Consolas", "Courier New") if f in fams), "TkFixedFont")
    ICON_FONT = next((f for f in ("Segoe Fluent Icons", "Segoe MDL2 Assets") if f in fams), None)
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
        tkfont.nametofont(name).configure(family=UI_FONT, size=T["body"])
    st = ttk.Style(root)
    st.theme_use("clam")
    st.configure(".", background=C.base, foreground=C.text, fieldbackground=C.sunken, font=fnt())

    # tables
    st.layout("App.Treeview", [("Treeview.treearea", {"sticky": "nswe"})])
    st.configure("App.Treeview", background=C.panel, fieldbackground=C.panel, foreground=C.text,
                 rowheight=S(28), borderwidth=0, relief="flat", font=fnt())
    st.map("App.Treeview", background=[("selected", C.accent_subtle)], foreground=[("selected", C.text)])
    st.configure("App.Treeview.Heading", background=C.raised, foreground=C.muted, relief="flat", borderwidth=0,
                 font=fnt(T["caption"], "bold"), padding=(S(8), S(5)))
    st.map("App.Treeview.Heading", background=[("active", C.raised)])

    # thin scrollbar, no arrows
    st.layout("App.Vertical.TScrollbar", [("Vertical.Scrollbar.trough", {
        "sticky": "ns", "children": [("Vertical.Scrollbar.thumb", {"expand": "1", "sticky": "nswe"})]})])
    st.configure("App.Vertical.TScrollbar", background=C.hover, troughcolor=C.panel, bordercolor=C.panel,
                 lightcolor=C.hover, darkcolor=C.hover, relief="flat", width=S(10))
    st.map("App.Vertical.TScrollbar", background=[("pressed", C.line_strong), ("active", C.pressed)],
           lightcolor=[("pressed", C.line_strong), ("active", C.pressed)],
           darkcolor=[("pressed", C.line_strong), ("active", C.pressed)])

    # combobox
    st.configure("App.TCombobox", fieldbackground=C.sunken, background=C.raised, foreground=C.text,
                 arrowcolor=C.muted, bordercolor=C.field_edge, lightcolor=C.field_edge,
                 darkcolor=C.field_edge, arrowsize=S(14), padding=(S(8), S(6)),
                 selectbackground=C.sunken, selectforeground=C.text)
    st.map("App.TCombobox",
           fieldbackground=[("disabled", C.base), ("readonly", C.sunken)],
           foreground=[("disabled", C.subtle)],
           arrowcolor=[("disabled", C.subtle), ("hover", C.text)],
           bordercolor=[("focus", C.accent), ("hover", C.focus)],
           lightcolor=[("focus", C.accent), ("hover", C.focus)],
           darkcolor=[("focus", C.accent), ("hover", C.focus)],
           selectbackground=[("readonly", "focus", C.accent_subtle)],
           selectforeground=[("readonly", "focus", C.text)])
    root.option_add("*TCombobox*Listbox.background", C.raised)
    root.option_add("*TCombobox*Listbox.foreground", C.text)
    root.option_add("*TCombobox*Listbox.selectBackground", C.accent_fill)
    root.option_add("*TCombobox*Listbox.selectForeground", C.on_fill)
    root.option_add("*TCombobox*Listbox.font", fnt())
    root.option_add("*Entry.selectBackground", C.accent_fill)
    root.option_add("*Entry.selectForeground", C.on_fill)

    # checkbox
    st.configure("App.TCheckbutton", background=C.base, foreground=C.text, font=fnt(), padding=(0, S(2)),
                 indicatorbackground=C.sunken, indicatorforeground=C.on_fill, indicatormargin=(0, 0, S(8), 0),
                 upperbordercolor=C.line_strong, lowerbordercolor=C.line_strong, focuscolor=C.base)
    st.map("App.TCheckbutton", background=[("active", C.base)],
           indicatorbackground=[("selected", C.accent_fill), ("pressed", C.pressed)],
           upperbordercolor=[("focus", C.focus), ("selected", C.accent_fill)],
           lowerbordercolor=[("focus", C.focus), ("selected", C.accent_fill)])


class Tooltip:
    """Small hover hint. Appears after a pause, never steals focus."""

    def __init__(self, widget, text, auto=True):
        self.widget, self.text, self.win, self.job = widget, text, None, None
        if auto:
            widget.bind("<Enter>", self.schedule, add="+")
            widget.bind("<Leave>", self.hide, add="+")
            widget.bind("<ButtonPress>", self.hide, add="+")

    def schedule(self, _e=None):
        self.hide()
        self.job = self.widget.after(500, self.show)

    def show(self):
        self.job = None
        if self.win or not self.text:
            return
        x = self.widget.winfo_rootx() + S(8)
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + S(6)
        self.win = tk.Toplevel(self.widget)
        self.win.wm_overrideredirect(True)
        self.win.wm_geometry(f"+{x}+{y}")
        tk.Label(self.win, text=self.text, bg=C.sunken, fg=C.text, font=fnt(T["caption"]), justify="left",
                 wraplength=S(320), padx=S(10), pady=S(7), highlightthickness=1,
                 highlightbackground=C.line_strong).pack()

    def hide(self, _e=None):
        if self.job:
            self.widget.after_cancel(self.job)
            self.job = None
        if self.win:
            self.win.destroy()
            self.win = None


_VARIANTS = {   # bg None = the parent's background (a quiet button has no fill until hovered)
    "primary": dict(bg=C.accent_fill, hover=C.accent_hover, press=C.accent_press, fg=C.on_fill, border=None),
    "secondary": dict(bg=C.raised, hover=C.hover, press=C.pressed, fg=C.text, border=C.line_strong),
    "quiet": dict(bg=None, hover=C.raised, press=C.hover, fg=C.text, border=None),
    "danger": dict(bg=C.danger_fill, hover=C.danger_hover, press=C.danger_press, fg=C.on_fill, border=None,
                   kbd="#ffd9d6"),
    "dangerq": dict(bg=None, hover=C.danger_subtle, press=C.danger_subtle, fg=C.danger, border=None),
}


class Btn(tk.Frame):
    """One button for the whole app: variants, icon, hover / pressed / disabled, and a
    focus ring for keyboard use (Tab to reach it, Space or Enter to press it)."""

    def __init__(self, parent, text="", command=None, icon=None, variant="secondary", compact=False,
                 tip=None, kbd=None, pad_x=None):
        self._pbg = parent.cget("bg")
        super().__init__(parent, bg=self._pbg, highlightthickness=S(2), highlightcolor=C.focus,
                         highlightbackground=self._pbg, takefocus=True, cursor="hand2")
        self.command, self.variant, self._state = command, variant, "normal"
        self._hover = self._down = False
        self.tip = Tooltip(self, tip, auto=False) if tip else None
        px, py = (S(8), S(3)) if compact else (S(12), S(6))
        if pad_x is not None:
            px = pad_x
        self.box = tk.Frame(self, bg=self._pbg)
        self.box.pack(expand=True)
        self._parts = []
        self.icon_lbl = self.text_lbl = self.kbd_lbl = None
        if icon and ICON_FONT:
            self.icon_lbl = tk.Label(self.box, text=ICONS[icon], font=(ICON_FONT, T["body"] + 1), pady=py)
            self.icon_lbl.pack(side="left", padx=(px, S(6) if text else px))
        if text or not self.icon_lbl:
            self.text_lbl = tk.Label(self.box, text=text, font=fnt(T["body"], "bold"), pady=py)
            self.text_lbl.pack(side="left", padx=(0 if self.icon_lbl else px, S(6) if kbd else px))
        if kbd:
            self.kbd_lbl = tk.Label(self.box, text=kbd, font=fnt(T["caption"]), pady=py)
            self.kbd_lbl.pack(side="left", padx=(0, px))
        self._parts = [w for w in (self.icon_lbl, self.text_lbl, self.kbd_lbl) if w]
        for w in (self, self.box, *self._parts):
            w.bind("<Enter>", self._enter, add="+")
            w.bind("<Leave>", self._leave, add="+")
            w.bind("<ButtonPress-1>", self._press, add="+")
            w.bind("<ButtonRelease-1>", self._release, add="+")
        self.bind("<Return>", lambda e: self.invoke())
        self.bind("<space>", lambda e: self.invoke())
        self._paint()

    def _inside(self, e):
        try:
            w = self.winfo_containing(e.x_root, e.y_root)
        except KeyError:
            return False
        return w is not None and (w is self or str(w).startswith(str(self) + "."))

    def _enter(self, _e=None):
        if self._state == "disabled":
            return
        self._hover = True
        self._paint()
        if self.tip:
            self.tip.schedule()

    def _leave(self, e):
        if self._inside(e):
            return
        self._hover = self._down = False
        self._paint()
        if self.tip:
            self.tip.hide()

    def _press(self, _e):
        if self._state == "disabled":
            return
        self._down = True
        self._paint()
        if self.tip:
            self.tip.hide()

    def _release(self, e):
        was = self._down
        self._down = False
        self._paint()
        if was and self._inside(e):
            self.invoke()

    def invoke(self):
        if self._state != "disabled" and self.command:
            self.command()

    def _paint(self):
        v = _VARIANTS[self.variant]
        if self._state == "disabled":
            solid = v["bg"] is not None
            bg = C.raised if solid else self._pbg
            border = C.line if solid else self._pbg
            fg = kbd = C.subtle
        else:
            bg = v["press"] if self._down else v["hover"] if self._hover else (v["bg"] or self._pbg)
            border = v["border"] or bg
            fg, kbd = v["fg"], v.get("kbd", v["fg"])
        tk.Frame.configure(self, bg=bg, highlightbackground=border,
                           cursor="arrow" if self._state == "disabled" else "hand2")
        self.box.configure(bg=bg)
        for w in self._parts:
            w.configure(bg=bg, fg=kbd if w is self.kbd_lbl else fg)

    def configure(self, cnf=None, **kw):
        kw.update(cnf or {})
        for key in ("state", "variant", "command", "text"):
            if key not in kw:
                continue
            val = kw.pop(key)
            if key == "state":
                self._state = val
                self._hover = self._down = False
            elif key == "variant":
                self.variant = val
            elif key == "command":
                self.command = val
            elif self.text_lbl:
                self.text_lbl.configure(text=val)
        if kw:
            tk.Frame.configure(self, **kw)
        self._paint()

    config = configure


class Field(tk.Frame):
    """Text / number entry with a border that shows focus (blue), bad input (red) and disabled."""

    def __init__(self, parent, var, width=8, unit=None, numeric=True):
        super().__init__(parent, bg=C.field_edge)
        self.var, self.numeric, self.focused, self.invalid, self._off = var, numeric, False, False, False
        self.ring = tk.Frame(self, bg=C.sunken)
        self.ring.pack(padx=S(1), pady=S(1))
        self.well = tk.Frame(self.ring, bg=C.sunken)
        self.well.pack(padx=S(1), pady=S(1))
        self.entry = tk.Entry(self.well, textvariable=var, width=width, bg=C.sunken, fg=C.text,
                              insertbackground=C.text, relief="flat", bd=0, highlightthickness=0,
                              disabledbackground=C.base, disabledforeground=C.subtle,
                              font=mono(T["body"] + 1) if numeric else fnt(T["body"] + 1),
                              justify="right" if numeric else "left")
        self.entry.pack(side="left", padx=(S(8), S(4) if unit else S(8)), pady=S(4))
        self.unit_lbl = None
        if unit:
            self.unit_lbl = tk.Label(self.well, text=unit, bg=C.sunken, fg=C.muted, font=fnt(T["caption"]))
            self.unit_lbl.pack(side="left", padx=(0, S(8)))
        self.entry.bind("<FocusIn>", lambda e: self._focus(True), add="+")
        self.entry.bind("<FocusOut>", lambda e: self._focus(False), add="+")
        var.trace_add("write", self._check)
        self._check()

    def _focus(self, on):
        self.focused = on
        self._paint()

    def _check(self, *_):
        if self.numeric:
            try:
                float(self.var.get())
                self.invalid = False
            except (ValueError, tk.TclError):
                self.invalid = True
        self._paint()

    def _paint(self):
        if self._off:
            edge, ring, well = C.line, C.base, C.base
        else:
            edge = C.danger if self.invalid else C.accent if self.focused else C.field_edge
            ring = edge if (self.focused or self.invalid) else C.sunken
            well = C.sunken
        self.configure(bg=edge)
        self.ring.configure(bg=ring)
        self.well.configure(bg=well)
        self.entry.configure(bg=well)
        if self.unit_lbl:
            self.unit_lbl.configure(bg=well, fg=C.subtle if self._off else C.muted)

    def set_state(self, state):
        self._off = state == "disabled"
        self.entry.configure(state=state)
        self._paint()


class Note(tk.Label):
    """Wrapping helper text. Pack it with fill='x' so it knows how wide it may be."""

    def __init__(self, parent, text="", fg=None, size=T["body"], **kw):
        super().__init__(parent, text=text, bg=parent.cget("bg"), fg=fg or C.muted, font=fnt(size),
                         justify="left", anchor="w", **kw)
        self.bind("<Configure>", lambda e: self.configure(wraplength=max(S(120), e.width)))


class Section(tk.Frame):
    """A titled block on the page: heading, optional one-line description, body. No box."""

    def __init__(self, parent, title, desc=None):
        bg = parent.cget("bg")
        super().__init__(parent, bg=bg)
        self.head = tk.Frame(self, bg=bg)
        self.head.pack(fill="x")
        tk.Label(self.head, text=title, bg=bg, fg=C.text, font=fnt(T["heading"], "bold"), anchor="w").pack(side="left")
        self.meta = tk.Label(self.head, text="", bg=bg, fg=C.muted, font=fnt(T["caption"]))
        self.meta.pack(side="left", padx=(S(8), 0))
        if desc:
            Note(self, desc).pack(fill="x", pady=(S(2), 0))
        self.body = tk.Frame(self, bg=bg)
        self.body.pack(fill="both", expand=True, pady=(S(10), 0))


def rule(parent, pady=(S(16), S(16))):
    tk.Frame(parent, bg=C.line, height=1).pack(fill="x", pady=pady)


class AutoScroll(ttk.Scrollbar):
    """Scrollbar that only shows when there is something to scroll (parent must use grid)."""

    def set(self, lo, hi):
        if float(lo) <= 0.0 and float(hi) >= 1.0:
            self.grid_remove()
        else:
            self.grid()
        super().set(lo, hi)


class ScrollPage(tk.Frame):
    """Vertical scroll container. Content fills the page when it fits and scrolls when it doesn't,
    so a short window or a scaled display never cuts a control off."""

    def __init__(self, parent):
        super().__init__(parent, bg=C.base)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(self, bg=C.base, highlightthickness=0, bd=0, yscrollincrement=S(30))
        self.bar = AutoScroll(self, orient="vertical", command=self.canvas.yview, style="App.Vertical.TScrollbar")
        self.canvas.configure(yscrollcommand=self.bar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.bar.grid(row=0, column=1, sticky="ns")
        self.body = tk.Frame(self.canvas, bg=C.base)
        self._win = self.canvas.create_window(0, 0, window=self.body, anchor="nw")
        self.body.bind("<Configure>", self._fit)
        self.canvas.bind("<Configure>", self._fit)

    def _fit(self, _e=None):
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        h = max(ch, self.body.winfo_reqheight())
        self.canvas.itemconfigure(self._win, width=cw, height=h)
        self.canvas.configure(scrollregion=(0, 0, cw, h))


class ListTable(tk.Frame):
    """A read-only table with one selected row. Programmatic set_rows() never fires on_select;
    only the user picking a different row does."""

    def __init__(self, parent, columns, empty_text="", height=6, on_select=None, on_activate=None):
        super().__init__(parent, bg=C.line_strong)
        self.on_select, self.on_activate, self._sel = on_select, on_activate, None
        inner = tk.Frame(self, bg=C.panel)
        inner.pack(fill="both", expand=True, padx=S(1), pady=S(1))
        inner.columnconfigure(0, weight=1)
        inner.rowconfigure(0, weight=1)
        self.tree = ttk.Treeview(inner, columns=[c[0] for c in columns], show="headings", height=height,
                                 selectmode="browse", style="App.Treeview")
        for cid, head, width, anchor, stretch in columns:
            self.tree.heading(cid, text=head, anchor=anchor)
            self.tree.column(cid, width=S(width), minwidth=S(24), anchor=anchor, stretch=stretch)
        bar = AutoScroll(inner, orient="vertical", command=self.tree.yview, style="App.Vertical.TScrollbar")
        self.tree.configure(yscrollcommand=bar.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        bar.grid(row=0, column=1, sticky="ns")
        self.tree.tag_configure("now", foreground=C.accent)
        self.empty = tk.Label(inner, text=empty_text, bg=C.panel, fg=C.muted, font=fnt(), justify="center",
                              wraplength=S(240))
        self.tree.bind("<<TreeviewSelect>>", self._picked)
        self.tree.bind("<Double-Button-1>", lambda e: self._activate())
        self.tree.bind("<Return>", lambda e: self._activate())

    def _activate(self):
        i = self.selected()
        if i is not None and self.on_activate:
            self.on_activate(i)

    def _picked(self, _e=None):
        i = self.selected()
        if i is None or i == self._sel:
            return
        self._sel = i
        if self.on_select:
            self.on_select(i)

    def selected(self):
        sel = self.tree.selection()
        return self.tree.index(sel[0]) if sel else None

    def set_empty_text(self, text):
        self.empty.configure(text=text)

    def set_rows(self, rows, selected=None, tags=None):
        self.tree.delete(*self.tree.get_children())
        for i, r in enumerate(rows):
            self.tree.insert("", "end", values=r, tags=(tags[i],) if tags and tags[i] else ())
        kids = self.tree.get_children()
        self._sel = None
        if selected is not None and 0 <= selected < len(kids):
            self.tree.selection_set(kids[selected])
            self.tree.see(kids[selected])
            self._sel = selected
        if kids:
            self.empty.place_forget()
        else:
            self.empty.place(relx=0.5, rely=0.62, anchor="center")


class Tabs(tk.Frame):
    """Tab strip with an underline on the current tab. Arrow keys move between tabs."""

    def __init__(self, parent):
        super().__init__(parent, bg=C.base)
        self.bar = tk.Frame(self, bg=C.base)
        self.bar.pack(fill="x", padx=S(24))
        tk.Frame(self, bg=C.line, height=1).pack(fill="x")
        self.stage = tk.Frame(self, bg=C.base)
        self.stage.pack(fill="both", expand=True)
        self.stage.columnconfigure(0, weight=1)
        self.stage.rowconfigure(0, weight=1)
        self.cells, self.index = [], -1

    def add(self, title):
        page = tk.Frame(self.stage, bg=C.base)
        i = len(self.cells)
        cell = tk.Frame(self.bar, bg=C.base, takefocus=True, cursor="hand2", highlightthickness=S(2),
                        highlightbackground=C.base, highlightcolor=C.focus)
        lbl = tk.Label(cell, text=title, bg=C.base, fg=C.muted, font=fnt(T["body"], "bold"), padx=S(14), pady=S(9))
        under = tk.Frame(cell, bg=C.base, height=S(2))
        lbl.pack()
        under.pack(fill="x")
        cell.pack(side="left")
        self.cells.append((cell, lbl, under, page))
        for w in (cell, lbl):
            w.bind("<Button-1>", lambda e, i=i: self.select(i, focus=True))
            w.bind("<Enter>", lambda e, i=i: self._paint(i, True))
            w.bind("<Leave>", lambda e, i=i: self._paint(i, False))
        cell.bind("<Return>", lambda e, i=i: self.select(i))
        cell.bind("<space>", lambda e, i=i: self.select(i))
        cell.bind("<Left>", lambda e, i=i: self.select(max(0, i - 1), focus=True))
        cell.bind("<Right>", lambda e, i=i: self.select(min(len(self.cells) - 1, i + 1), focus=True))
        if self.index < 0:
            self.select(0)
        return page

    def _paint(self, i, hover=False):
        cell, lbl, under, _page = self.cells[i]
        on = i == self.index
        lbl.configure(fg=C.text if (on or hover) else C.muted)
        under.configure(bg=C.accent if on else C.base)

    def select(self, i, focus=False):
        self.index = i
        for j, (cell, _lbl, _under, page) in enumerate(self.cells):
            if j == i:
                page.grid(row=0, column=0, sticky="nsew")
            else:
                page.grid_remove()
            self._paint(j)
        if focus:
            self.cells[i][0].focus_set()


def dot(parent, bg):
    c = tk.Canvas(parent, width=S(10), height=S(10), bg=bg, highlightthickness=0)
    return c


def paint_dot(c, color):
    c.delete("all")
    c.create_oval(1, 1, S(10) - 1, S(10) - 1, fill=color, outline="")


def num(var, default=0.0):
    try:
        return float(var.get())
    except (ValueError, tk.TclError):
        return default


def nice_step(span, target=6):
    raw = max(span, 1e-6) / target
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


class StepList(tk.Frame):
    """A table of steps with an editor underneath. kind='combo' (a step has a delay) or
    'action' (an angle function: has an angle and a direction)."""

    def __init__(self, parent, app, kind, on_change, on_test, live_angle):
        bg = parent.cget("bg")
        super().__init__(parent, bg=bg)
        self.app, self.kind, self.on_change, self.on_test, self.live_angle = app, kind, on_change, on_test, live_angle
        self.items, self.locked = [], False
        self.combo_names, self.combo_labels = [], []
        self.noun = "function" if kind == "action" else "step"
        if kind == "combo":
            cols = [("n", "#", 36, "w", False), ("what", "Step", 380, "w", True)]
            empty = "No steps yet."
        else:
            cols = [("n", "#", 36, "w", False), ("when", "Trigger", 170, "w", False), ("btn", "Button", 96, "w", False),
                    ("what", "Then do", 300, "w", True)]
            empty = "No angle functions yet."
        self.table = ListTable(self, cols, empty_text=empty, height=3 if kind == "combo" else 4,
                               on_select=lambda i: (self.load_form(), self.update_state()))
        self.hint = Note(self, "")

        form = tk.Frame(self, bg=bg)
        self.v_angle = tk.StringVar(value="0")
        self.v_when = tk.StringVar(value=WHENS[0])
        self.v_delay = tk.StringVar(value="0")
        self.v_dev = tk.StringVar(value=DEVICES[4])
        self.v_cmd = tk.StringVar(value="Forward")
        self.v_speed = tk.StringVar(value="100")
        self.v_dur = tk.StringVar(value="500")
        self.v_button = tk.StringVar(value=BUTTONS[0])
        self.v_mode = tk.StringVar(value=BTN_MODES[0])
        self.v_do = tk.StringVar(value=SINGLE_STEP)

        def cell(r, c, title):
            f = tk.Frame(form, bg=bg)
            f.grid(row=r, column=c, sticky="nw", padx=(0, S(16)), pady=(0, S(8)))
            tk.Label(f, text=title, bg=bg, fg=C.muted, font=fnt(T["caption"])).pack(anchor="w", pady=(0, S(3)))
            return f

        if kind == "action":
            c = cell(0, 0, "When the lift passes")
            row = tk.Frame(c, bg=bg)
            row.pack(anchor="w")
            Field(row, self.v_angle, 7, unit="deg").pack(side="left")
            Btn(row, "Use live", self.use_live, compact=True, tip="Copies the lift's current angle into this box."
                ).pack(side="left", padx=(S(8), 0))
            ttk.Combobox(cell(0, 1, "Direction"), textvariable=self.v_when, values=WHENS, state="readonly",
                         width=11, style="App.TCombobox").pack(anchor="w")
            self.cb_btn = ttk.Combobox(cell(0, 2, "Button"), textvariable=self.v_button, values=BUTTONS,
                                       state="readonly", width=13, style="App.TCombobox")
            self.cb_btn.pack(anchor="w")
            self.cb_btn.bind("<<ComboboxSelected>>", lambda e: self.refresh_action_fields())
            self.cb_mode = ttk.Combobox(cell(0, 3, "Hold or tap"), textvariable=self.v_mode, values=BTN_MODES,
                                        state="readonly", width=7, style="App.TCombobox")
            self.cb_mode.pack(anchor="w")
            self.cb_do = ttk.Combobox(cell(1, 0, "Then do"), textvariable=self.v_do, values=[SINGLE_STEP],
                                      state="readonly", width=24, style="App.TCombobox")
            self.cb_do.pack(anchor="w")
            self.cb_do.bind("<<ComboboxSelected>>", lambda e: self.refresh_action_fields())
            r_dev, r_par = 2, 2
        else:
            r_dev, r_par = 0, 1
            c = cell(1, 0, "Starts after")
            Field(c, self.v_delay, 6, unit="ms").pack(anchor="w")
        self.cb_dev = ttk.Combobox(cell(r_dev, 0, "Device"), textvariable=self.v_dev, values=DEVICES,
                                   state="readonly", width=20, style="App.TCombobox")
        self.cb_dev.pack(anchor="w")
        self.cb_dev.bind("<<ComboboxSelected>>", lambda e: self.refresh_cmds())
        self.cb_cmd = ttk.Combobox(cell(r_dev, 1, "Command"), textvariable=self.v_cmd, state="readonly",
                                   width=9, style="App.TCombobox")
        self.cb_cmd.pack(anchor="w")
        self.cb_cmd.bind("<<ComboboxSelected>>", lambda e: self.refresh_cmds())
        speed_col, dur_col = (2, 3) if kind == "action" else (1, 2)
        self.f_speed = Field(cell(r_par, speed_col, "Speed"), self.v_speed, 5, unit="rpm")
        self.f_speed.pack(anchor="w")
        self.f_dur = Field(cell(r_par, dur_col, "Run for"), self.v_dur, 6, unit="ms")
        self.f_dur.pack(anchor="w")
        self.refresh_cmds()

        row = tk.Frame(self, bg=bg)
        Btn(row, f"Add {self.noun}", self.add, icon="add", compact=True,
            tip=f"Adds the fields above as a NEW {self.noun} at the end of the list.").pack(side="left")
        self.btnUpdate = Btn(row, f"Update {self.noun}", self.update_sel, compact=True,
                             tip=f"Overwrites the selected {self.noun} with the fields above.")
        self.btnUpdate.pack(side="left", padx=S(6))
        self.btnRemove = Btn(row, "", self.remove, icon="delete", variant="dangerq", compact=True,
                             tip=f"Remove the selected {self.noun}.")
        self.btnRemove.pack(side="left")
        self.btnUp = Btn(row, "", lambda: self.move(-1), icon="up", variant="quiet", compact=True,
                         tip=f"Move the selected {self.noun} earlier.")
        self.btnUp.pack(side="left", padx=(S(12), S(2)))
        self.btnDown = Btn(row, "", lambda: self.move(1), icon="down", variant="quiet", compact=True,
                           tip=f"Move the selected {self.noun} later.")
        self.btnDown.pack(side="left")
        self.btnTest = Btn(row, f"Test {self.noun}", self.test, icon="play", compact=True,
                           tip=f"Runs just the selected {self.noun} on the robot now. Applies unsent edits first.")
        self.btnTest.pack(side="right")
        # Pack from the bottom up so that, when the window is short, the table gives up
        # rows first and the editor underneath stays fully visible.
        row.pack(side="bottom", fill="x")
        form.pack(side="bottom", fill="x")
        self.hint.pack(side="bottom", fill="x", pady=(S(8), S(10)))
        self.table.pack(fill="both", expand=True)
        self.update_state()

    def use_live(self):
        self.v_angle.set(f"{self.live_angle():.1f}")

    def refresh_cmds(self):
        dev = DEVICES.index(self.v_dev.get())
        cmds = PISTON_CMDS if is_piston(dev) else MOTOR_CMDS
        self.cb_cmd["values"] = [c[0] for c in cmds]
        if self.v_cmd.get() not in [c[0] for c in cmds]:
            self.v_cmd.set(cmds[0][0])
        self.refresh_action_fields()

    def refresh_action_fields(self):
        """Greys out what does not apply: the single-step fields when a combo is chosen, and
        hold / tap when there is no button."""
        use_combo = self.kind == "action" and self.v_do.get() != SINGLE_STEP
        if self.kind == "action":
            self.cb_mode.configure(state="readonly" if self.v_button.get() != BUTTONS[0] else "disabled")
        dev = DEVICES.index(self.v_dev.get())
        motor_params = not (is_piston(dev) or self.v_cmd.get() == "Stop")
        for cb in (self.cb_dev, self.cb_cmd):
            cb.configure(state="disabled" if use_combo else "readonly")
        for f in (self.f_speed, self.f_dur):
            f.set_state("normal" if (motor_params and not use_combo) else "disabled")

    def set_combo_names(self, names):
        """The combos an angle function can play (kept in sync by the app)."""
        self.combo_names = list(names)
        self.combo_labels = [f"{i + 1}. {n}" for i, n in enumerate(names)]
        if self.kind != "action":
            return
        self.cb_do["values"] = [SINGLE_STEP] + self.combo_labels
        if self.v_do.get() not in [SINGLE_STEP] + self.combo_labels:
            self.v_do.set(SINGLE_STEP)
        self.refresh_action_fields()
        self.table.set_rows(self._rows(), self.selected())

    def read_form(self):
        dev = DEVICES.index(self.v_dev.get())
        cmds = dict(PISTON_CMDS if is_piston(dev) else MOTOR_CMDS)
        step = new_step(dev=dev, cmd=cmds[self.v_cmd.get()],
                        speed=int(max(1, min(200, num(self.v_speed, 100)))),
                        dur=int(max(50, min(10000, num(self.v_dur, 500)))),
                        delay=int(max(0, min(20000, num(self.v_delay, 0)))) if self.kind == "combo" else 0)
        if self.kind == "action":
            button = BUTTONS.index(self.v_button.get())
            labels = [SINGLE_STEP] + self.combo_labels
            combo = labels.index(self.v_do.get()) - 1 if self.v_do.get() in labels else -1
            return new_action(round(num(self.v_angle), 1), WHENS.index(self.v_when.get()), step, combo, button,
                              BTN_MODES.index(self.v_mode.get()) if button else 0)
        return step

    def load_form(self):
        i = self.selected()
        if i is None:
            return
        it = self.items[i]
        step = it["step"] if self.kind == "action" else it
        if self.kind == "action":
            self.v_angle.set(f"{it['angle']:.1f}")
            self.v_when.set(WHENS[it["when"]])
            self.v_button.set(BUTTONS[it.get("button", 0)])
            self.v_mode.set(BTN_MODES[it.get("mode", 0)])
            ci = it.get("combo", -1)
            self.v_do.set(self.combo_labels[ci] if 0 <= ci < len(self.combo_labels) else SINGLE_STEP)
        else:
            self.v_delay.set(str(step["delay"]))
        self.v_dev.set(DEVICES[step["dev"]])
        self.refresh_cmds()
        self.v_cmd.set(cmd_name(step["dev"], step["cmd"]))
        self.v_speed.set(str(step["speed"]))
        self.v_dur.set(str(step["dur"]))
        self.refresh_cmds()

    def update_state(self):
        i = self.selected()
        for b in (self.btnUpdate, self.btnRemove, self.btnUp, self.btnDown, self.btnTest):
            b.configure(state="normal" if i is not None else "disabled")
        if self.locked:
            self.hint.configure(text="No combo selected. Press “New combo” on the left to make one.")
            self.table.set_empty_text("Select or create a combo first.")
        elif not self.items:
            self.hint.configure(text=f"No {self.noun}s yet. Fill in the fields below, then press “Add {self.noun}”.")
        elif i is None:
            self.hint.configure(text=f"Click a {self.noun} to edit it, or fill in the fields and press “Add {self.noun}” for a new one.")
        else:
            self.hint.configure(text=f"Editing {self.noun} {i + 1}. Change the fields, then press “Update {self.noun}”.")

    def selected(self):
        return self.table.selected()

    def set_items(self, items, keep=None):
        self.items = items
        self.redraw(keep)

    def _rows(self):
        rows = []
        for i, it in enumerate(self.items):
            if self.kind == "action":
                rows.append((i + 1, f"{WHENS[it['when']]} at {it['angle']:.1f}°", button_summary(it),
                             action_summary(it, self.combo_names)))
            else:
                rows.append((i + 1, step_summary(it)))
        return rows

    def redraw(self, keep=None):
        rows = self._rows()
        has_keep = keep is not None and 0 <= keep < len(rows)
        self.table.set_rows(rows, keep if has_keep else None)
        if has_keep:
            self.load_form()
        self.update_state()

    def add(self):
        if self.locked:
            self.app.notify("Make a combo first: press “New combo” on the left, then add steps to it.", "warning")
            return
        limit = MAX_ACTIONS if self.kind == "action" else MAX_COMBO_STEPS
        if len(self.items) >= limit:
            self.app.notify(f"At most {limit} entries fit in the robot's memory.", "warning")
            return
        self.items.append(self.read_form())
        self.redraw(len(self.items) - 1)
        self.on_change()

    def update_sel(self):
        i = self.selected()
        if i is not None:
            self.items[i] = self.read_form()
            self.redraw(i)
            self.on_change()

    def remove(self):
        i = self.selected()
        if i is not None:
            del self.items[i]
            self.redraw(min(i, len(self.items) - 1))
            self.on_change()

    def move(self, d):
        i = self.selected()
        if i is not None and 0 <= i + d < len(self.items):
            self.items[i], self.items[i + d] = self.items[i + d], self.items[i]
            self.redraw(i + d)
            self.on_change()

    def test(self):
        i = self.selected()
        if i is None:
            self.app.notify(f"Select a {self.noun} in the table first.", "warning")
            return
        self.on_test(i)


class App(tk.Tk):
    def __init__(self, demo):
        super().__init__()
        init_look(self)
        self.title("Lift Tuner — V5")
        self.configure(bg=C.base)
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w, h = min(S(1180), sw - S(60)), min(S(740), sh - S(90))
        self.geometry(f"{w}x{h}+{(sw - w) // 2}+{max(0, (sh - h) // 2 - S(20))}")
        self.minsize(min(S(1040), w), min(S(640), h))
        self.model = default_model()
        self.loaded_from_robot = False
        self.dirty = False
        self.live = 0.0
        self.moving = False
        self.combo_running = False
        self.curLevel = -1
        self.link = None
        self.demo = demo
        self.levelIdx = 0
        self.comboIdx = 0
        self.suspend = False
        self._noteJob = None
        if demo:
            self.model = sample_model()
        else:
            try:
                with open(BACKUP, encoding="utf-8") as f:
                    self.model = normalize_model(json.load(f))
            except (OSError, ValueError):
                pass
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self._build()
        self.bind("<Escape>", lambda e: self.stop())
        self.bind("<Control-Return>", lambda e: self.apply())
        self.bind("<Control-s>", lambda e: self.save_all())
        self.bind_all("<MouseWheel>", self._wheel)
        for n in range(len(self.tabs.cells)):
            self.bind(f"<Control-Key-{n + 1}>", lambda e, n=n: self.tabs.select(n))
        self.refresh_all()
        self.connect()
        self.after(60, self.poll)

    # ── layout ──
    def _build(self):
        self._build_header()
        body = tk.Frame(self, bg=C.base)
        body.grid(row=1, column=0, sticky="nsew")
        self._build_gauge_panel(body)
        tk.Frame(body, bg=C.line, width=1).pack(side="left", fill="y")
        self.tabs = Tabs(body)
        self.tabs.pack(side="left", fill="both", expand=True)
        self._tab_start()
        self._tab_combos()
        self._tab_levels()
        self._tab_actions()
        self.tabs.select(0)
        self._build_status()

    def _build_header(self):
        bar = tk.Frame(self, bg=C.panel)
        bar.grid(row=0, column=0, sticky="ew")
        inner = tk.Frame(bar, bg=C.panel)
        inner.pack(fill="x", padx=S(20), pady=S(10))
        tk.Label(inner, text="Lift tuner", bg=C.panel, fg=C.text, font=fnt(T["title"], "bold")).pack(side="left")
        conn = tk.Frame(inner, bg=C.panel)
        conn.pack(side="left", padx=(S(24), 0))
        self.dotConn = dot(conn, C.panel)
        self.dotConn.pack(side="left", padx=(0, S(8)))
        self.lblConn = tk.Label(conn, text="Connecting…", bg=C.panel, fg=C.warning, font=fnt(T["body"], "bold"))
        self.lblConn.pack(side="left")
        paint_dot(self.dotConn, C.warning)

        Btn(inner, "Stop", self.stop, icon="stop", variant="danger", kbd="Esc",
            tip="Halts the lift and every running step on the robot, immediately. (Esc)").pack(side="right")
        tk.Frame(inner, bg=C.line, width=1).pack(side="right", fill="y", padx=S(14), pady=S(4))
        self.btnSave = Btn(inner, "Save", self.save_all, icon="save",
                           tip="Sends everything to the robot, saves it on the brain's SD card, and writes it into the code "
                               "(defaults + one function per combo). Rebuild and upload afterwards. (Ctrl+S)")
        self.btnSave.pack(side="right", padx=(0, S(8)))
        self.btnApply = Btn(inner, "Apply to robot", self.apply, icon="upload",
                            tip="Sends everything on screen to the robot. It takes effect immediately, but is lost on "
                                "power-off until saved. (Ctrl+Enter)")
        self.btnApply.pack(side="right", padx=(0, S(8)))
        Btn(inner, "Load from robot", self.pull, icon="download", variant="quiet",
            tip="Throws away what's on screen and shows what the robot currently has."
            ).pack(side="right", padx=(0, S(8)))
        Btn(inner, "Reconnect", self.connect, icon="refresh", variant="quiet", tip="Reconnect to the brain."
            ).pack(side="right", padx=(0, S(8)))
        tk.Frame(bar, bg=C.line, height=1).pack(fill="x")

    def _build_status(self):
        bar = tk.Frame(self, bg=C.panel)
        bar.grid(row=2, column=0, sticky="ew")
        tk.Frame(bar, bg=C.line, height=1).pack(fill="x")
        row = tk.Frame(bar, bg=C.panel)
        row.pack(fill="x", padx=S(20), pady=S(7))
        self.dotSync = dot(row, C.panel)
        self.dotSync.pack(side="left", padx=(0, S(8)))
        self.lblSync = tk.Label(row, text="", bg=C.panel, fg=C.muted, font=fnt(T["body"], "bold"))
        self.lblSync.pack(side="left")
        self.lblHint = tk.Label(row, text="", bg=C.panel, fg=C.muted, font=fnt(T["body"]))
        self.lblHint.pack(side="left", padx=(S(12), 0))
        self.lblNote = tk.Label(row, text="", bg=C.panel, fg=C.muted, font=fnt(T["body"]), anchor="e")
        self.lblNote.pack(side="right")

    def _build_gauge_panel(self, body):
        gp = tk.Frame(body, bg=C.panel, width=S(280))
        gp.pack(side="left", fill="y")
        gp.pack_propagate(False)
        inner = tk.Frame(gp, bg=C.panel)
        inner.pack(fill="both", expand=True, padx=S(18), pady=S(16))
        tk.Label(inner, text="Lift position", bg=C.panel, fg=C.text, font=fnt(T["heading"], "bold"), anchor="w").pack(fill="x")
        self.lblLive = tk.Label(inner, text="0.0°", bg=C.panel, fg=C.text, font=mono(T["readout"], "bold"), anchor="w")
        self.lblLive.pack(fill="x", pady=(S(6), 0))
        self.lblMoving = tk.Label(inner, text="Idle", bg=C.panel, fg=C.muted, font=fnt(), anchor="w")
        self.lblMoving.pack(fill="x")
        # Controls under the gauge are packed first, so a short window shrinks the gauge, not them.
        low = tk.Frame(inner, bg=C.panel)
        low.pack(side="bottom", fill="x")
        self.gauge = tk.Canvas(inner, bg=C.panel, highlightthickness=0, cursor="hand2", width=S(200), height=S(60))
        self.gauge.pack(fill="both", expand=True, pady=S(10))
        self.gauge.bind("<Button-1>", self.gauge_click)
        self.gauge.bind("<Motion>", self.gauge_hover)
        self.gauge.bind("<Leave>", lambda e: self.gauge.delete("hover"))
        self.gauge.bind("<Configure>", lambda e: self.draw_gauge())
        Note(low, "Click the gauge to send the lift there.", size=T["caption"]).pack(fill="x")

        tk.Label(low, text="Jog (degrees)", bg=C.panel, fg=C.muted, font=fnt(T["caption"]), anchor="w"
                 ).pack(fill="x", pady=(S(14), S(4)))
        jog = tk.Frame(low, bg=C.panel)
        jog.pack(fill="x")
        for i, j in enumerate(JOGS):
            jog.columnconfigure(i, weight=1, uniform="jog")
            Btn(jog, f"{j:+d}", lambda j=j: self.send([f"LT JOG {j}"]), compact=True, pad_x=S(2)).grid(
                row=0, column=i, sticky="ew", padx=S(1))
        tk.Label(low, text="Go to angle", bg=C.panel, fg=C.muted, font=fnt(T["caption"]), anchor="w"
                 ).pack(fill="x", pady=(S(12), S(4)))
        go = tk.Frame(low, bg=C.panel)
        go.pack(fill="x")
        self.v_go = tk.StringVar(value="0")
        Field(go, self.v_go, 7, unit="deg").pack(side="left")
        Btn(go, "Go", lambda: self.send([f"LT GO {num(self.v_go):.2f}"]), icon="go", compact=True
            ).pack(side="left", padx=(S(8), 0))

    def _page(self, title):
        page = ScrollPage(self.tabs.add(title))
        page.pack(fill="both", expand=True)
        wrap = tk.Frame(page.body, bg=C.base)
        wrap.pack(fill="both", expand=True, padx=S(24), pady=S(14))
        return wrap

    def _wheel(self, e):
        """Mouse wheel scrolls the page under the pointer, unless a table under it wants the wheel."""
        try:
            w = self.winfo_containing(e.x_root, e.y_root)
        except KeyError:
            return
        if isinstance(w, (ttk.Treeview, tk.Listbox)):
            return
        while w is not None and not isinstance(w, ScrollPage):
            w = getattr(w, "master", None)
        if w is not None and w.bar.winfo_ismapped():
            w.canvas.yview_scroll(-1 if e.delta > 0 else 1, "units")

    def _tab_start(self):
        wrap = self._page("Start & limits")
        s1 = Section(wrap, "Start position",
                     "Where the lift is assumed to be when driver control starts, in motor degrees. "
                     "Turn it off to keep whatever the encoder says (zeroed at power-on).")
        s1.pack(fill="x")
        r = tk.Frame(s1.body, bg=C.base)
        r.pack(fill="x")
        self.v_startEn = tk.BooleanVar()
        ttk.Checkbutton(r, text="Set the lift to this position when driver control starts", variable=self.v_startEn,
                        command=self.on_start_edit, style="App.TCheckbutton").pack(side="left")
        self.v_start = tk.StringVar()
        Field(r, self.v_start, 8, unit="deg").pack(side="left", padx=(S(20), S(8)))
        Btn(r, "Use live", lambda: (self.v_start.set(f"{self.live:.1f}"), self.v_startEn.set(True), self.on_start_edit()),
            compact=True, tip="Copies the lift's current angle into this box and turns the start position on."
            ).pack(side="left")
        rule(wrap, (S(22), S(22)))
        s2 = Section(wrap, "Forward limit", "X stops driving the lift forward at this angle. A (backward) is never limited.")
        s2.pack(fill="x")
        r2 = tk.Frame(s2.body, bg=C.base)
        r2.pack(fill="x")
        self.v_limit = tk.StringVar()
        Field(r2, self.v_limit, 8, unit="deg").pack(side="left")
        Btn(r2, "Use live", lambda: (self.v_limit.set(f"{self.live:.1f}"), self.on_start_edit()), compact=True,
            tip="Copies the lift's current angle into this box.").pack(side="left", padx=(S(8), 0))
        for v in (self.v_start, self.v_limit):
            v.trace_add("write", lambda *a: self.on_start_edit())

    def _tab_combos(self):
        wrap = self._page("Combos")
        Note(wrap, "A combo is a named list of steps. Pick one for each scoring level; every combo is also saved into "
                   "the code as a function.").pack(fill="x", pady=(0, S(10)))
        row = tk.Frame(wrap, bg=C.base)
        row.pack(fill="both", expand=True)
        left = tk.Frame(row, bg=C.base)
        left.pack(side="left", fill="y", padx=(0, S(24)))
        self.secCombos = Section(left, "Your combos")
        self.secCombos.pack(fill="both", expand=True)
        bt = tk.Frame(self.secCombos.body, bg=C.base)
        bt.pack(side="bottom", fill="x", pady=(S(8), 0))      # packed first: stays visible in a short window
        self.lstCombos = ListTable(self.secCombos.body, [("n", "#", 34, "w", False), ("name", "Name", 130, "w", True),
                                                          ("steps", "Steps", 54, "e", False)],
                                   empty_text="No combos yet.\nPress “New combo” to make one.", height=5,
                                   on_select=lambda i: self.pick_combo())
        self.lstCombos.pack(fill="both", expand=True)
        bt.columnconfigure((0, 1), weight=1, uniform="cb")
        Btn(bt, "New combo", self.add_combo, icon="add", tip="Creates an empty combo you can then fill with steps."
            ).grid(row=0, column=0, columnspan=2, sticky="ew")
        Btn(bt, "Duplicate", self.dup_combo, icon="copy", tip="Copies the selected combo so you can tweak a variation."
            ).grid(row=1, column=0, sticky="ew", pady=(S(4), 0), padx=(0, S(2)))
        Btn(bt, "Remove", self.remove_combo, icon="delete", variant="dangerq",
            tip="Deletes the selected combo. Levels that used it fall back to no combo."
            ).grid(row=1, column=1, sticky="ew", pady=(S(4), 0), padx=(S(2), 0))

        right = tk.Frame(row, bg=C.base)
        right.pack(side="left", fill="both", expand=True)
        hr = tk.Frame(right, bg=C.base)
        hr.pack(fill="x")
        self.v_cname = tk.StringVar()
        tk.Label(hr, text="Combo name", bg=C.base, fg=C.text, font=fnt(T["heading"], "bold")).pack(side="left", padx=(0, S(10)))
        Field(hr, self.v_cname, 14, numeric=False).pack(side="left")
        self.lblFunc = tk.Label(hr, text="", bg=C.base, fg=C.accent, font=mono(T["body"]))
        self.lblFunc.pack(side="left", padx=(S(14), 0))
        self.v_cname.trace_add("write", lambda *a: self.on_combo_name())
        self.lblUsed = Note(right, "")
        self.lblUsed.pack(fill="x", pady=(S(6), 0))
        rule(right, (S(10), S(10)))
        st = Section(right, "Steps", "Steps play in order. Each delay counts from when the previous step started.")
        st.pack(fill="both", expand=True)
        Btn(st.head, "Test whole combo", self.test_combo, icon="play", compact=True,
            tip="Plays this combo right now without moving the lift. Applies unsent edits first.").pack(side="right")
        self.comboList = StepList(st.body, self, "combo", self.on_combo_steps, self.test_combo_step, lambda: self.live)
        self.comboList.pack(fill="both", expand=True)

    def _tab_levels(self):
        wrap = self._page("Scoring levels")
        Note(wrap, "UP / LEFT on the controller step through these levels. The lift drives to the angle, "
                   "then the level's combo plays.").pack(fill="x", pady=(0, S(14)))
        row = tk.Frame(wrap, bg=C.base)
        row.pack(fill="both", expand=True)
        left = tk.Frame(row, bg=C.base)
        left.pack(side="left", fill="y", padx=(0, S(24)))
        self.secLevels = Section(left, "Levels")
        self.secLevels.pack(fill="both", expand=True)
        bt = tk.Frame(self.secLevels.body, bg=C.base)
        bt.pack(side="bottom", fill="x", pady=(S(8), 0))      # packed first: stays visible in a short window
        self.lstLevels = ListTable(self.secLevels.body,
                                   [("n", "#", 30, "w", False), ("name", "Name", 100, "w", True),
                                    ("angle", "Angle", 62, "e", False), ("combo", "Combo", 86, "w", False),
                                    ("at", "Robot", 50, "w", False)],
                                   empty_text="No levels yet.\nPress “Add” to make one.", height=7,
                                   on_select=lambda i: self.pick_level(),
                                   on_activate=lambda i: self.send([f"LT GO {self.cur()['angle']:.2f}"]) if self.cur() else None)
        self.lstLevels.pack(fill="both", expand=True)
        Tooltip(self.lstLevels.tree, "Click a level to edit it. Double-click to drive the lift there. "
                                     "“here” marks the level the robot is on.")
        bt.columnconfigure((0, 1), weight=1, uniform="lv")
        Btn(bt, "Add", self.add_level, icon="add").grid(row=0, column=0, sticky="ew", padx=(0, S(2)))
        Btn(bt, "Remove", self.remove_level, icon="delete", variant="dangerq").grid(row=0, column=1, sticky="ew", padx=(S(2), 0))
        Btn(bt, "Move up", lambda: self.move_level(-1), icon="up", variant="quiet").grid(
            row=1, column=0, sticky="ew", pady=(S(4), 0), padx=(0, S(2)))
        Btn(bt, "Move down", lambda: self.move_level(1), icon="down", variant="quiet").grid(
            row=1, column=1, sticky="ew", pady=(S(4), 0), padx=(S(2), 0))

        right = tk.Frame(row, bg=C.base)
        right.pack(side="left", fill="both", expand=True)
        det = Section(right, "Level")
        det.pack(fill="x")
        self.v_lname, self.v_langle = tk.StringVar(), tk.StringVar()
        fr = tk.Frame(det.body, bg=C.base)
        fr.pack(fill="x")

        def cell(c, title):
            f = tk.Frame(fr, bg=C.base)
            f.grid(row=0, column=c, sticky="w", padx=(0, S(16)))
            tk.Label(f, text=title, bg=C.base, fg=C.muted, font=fnt(T["caption"])).pack(anchor="w", pady=(0, S(3)))
            return f

        Field(cell(0, "Name"), self.v_lname, 11, numeric=False).pack(anchor="w")
        a = cell(1, "Lift angle")
        ar = tk.Frame(a, bg=C.base)
        ar.pack(anchor="w")
        Field(ar, self.v_langle, 7, unit="deg").pack(side="left")
        Btn(ar, "Use live", lambda: (self.v_langle.set(f"{self.live:.1f}"), self.on_level_edit()), compact=True,
            tip="Copies the lift's current angle into this level.").pack(side="left", padx=(S(8), 0))
        ar2 = tk.Frame(det.body, bg=C.base)
        ar2.pack(fill="x", pady=(S(12), 0))
        Btn(ar2, "Go there", lambda: self.send([f"LT GO {num(self.v_langle):.2f}"]), icon="go",
            tip="Drives the lift to this angle (no combo).").pack(side="left")
        Btn(ar2, "Run level (go + combo)", self.run_level, icon="play", variant="secondary",
            tip="Exactly what UP / LEFT do in a match: drive to this level, then play its combo. Applies unsent edits first."
            ).pack(side="left", padx=(S(8), 0))
        for v in (self.v_lname, self.v_langle):
            v.trace_add("write", lambda *a: self.on_level_edit())
        rule(right, (S(14), S(14)))
        cc = Section(right, "Combo that plays when the lift arrives")
        cc.pack(fill="both", expand=True)
        cr = tk.Frame(cc.body, bg=C.base)
        cr.pack(fill="x")
        self.v_lcombo = tk.StringVar()
        self.cbCombo = ttk.Combobox(cr, textvariable=self.v_lcombo, state="readonly", width=18, style="App.TCombobox")
        self.cbCombo.pack(side="left")
        self.cbCombo.bind("<<ComboboxSelected>>", lambda e: self.on_level_combo())
        Btn(cr, "Edit combo", self.edit_level_combo, icon="go", tip="Jumps to the Combos tab with this combo selected."
            ).pack(side="left", padx=(S(8), 0))
        Btn(cr, "New combo", lambda: self.add_combo(assign=True), icon="add",
            tip="Creates an empty combo, gives it to this level, and opens it on the Combos tab."
            ).pack(side="left", padx=(S(8), 0))
        self.lblSteps = tk.Label(cc.body, text="", bg=C.base, fg=C.text, font=fnt(), justify="left", anchor="nw")
        self.lblSteps.pack(fill="both", expand=True, pady=(S(12), 0))

    def _tab_actions(self):
        wrap = self._page("Angle functions")
        sec = Section(wrap, "Angle functions",
                      "When the lift passes an angle, play a combo or one step. Tie it to a button to make it fire only "
                      "while you hold that button, or once after you tap it.")
        sec.meta.configure(text=f"up to {MAX_ACTIONS}")
        sec.pack(fill="both", expand=True)
        self.actions = StepList(sec.body, self, "action", self.on_model_edit, self.test_action, lambda: self.live)
        self.actions.pack(fill="both", expand=True)

    # ── model <-> widgets ──
    def refresh_all(self):
        self.suspend = True
        m = self.model
        self.v_startEn.set(m["startEnabled"])
        self.v_start.set(f"{m['start']:.1f}")
        self.v_limit.set(f"{m['limit']:.1f}")
        if m["levels"]:
            self.levelIdx = min(self.levelIdx, len(m["levels"]) - 1)
        if m["combos"]:
            self.comboIdx = min(self.comboIdx, len(m["combos"]) - 1)
        self.refresh_combo_list()
        self.refresh_level_list()
        self.actions.set_items(m["actions"])
        self.pick_combo(refresh=False)
        self.pick_level(refresh=False)
        self.suspend = False
        self.draw_gauge()
        self.update_status()

    def combo_name(self, idx):
        c = self.model["combos"]
        return c[idx]["name"] if 0 <= idx < len(c) else None

    # -- combos
    def cur_combo(self):
        c = self.model["combos"]
        return c[self.comboIdx] if c and 0 <= self.comboIdx < len(c) else None

    def refresh_combo_list(self):
        combos = self.model["combos"]
        self.lstCombos.set_rows([(i + 1, c["name"], len(c["steps"])) for i, c in enumerate(combos)],
                                self.comboIdx if combos else None)
        self.secCombos.meta.configure(text=f"{len(combos)} of {MAX_COMBOS}")
        self.actions.set_combo_names([c["name"] for c in combos])
        self.cbCombo["values"] = ["(no combo)"] + [c["name"] for c in combos]

    def pick_combo(self, refresh=True):
        if refresh:
            sel = self.lstCombos.selected()
            if sel is not None:
                self.comboIdx = sel
        c = self.cur_combo()
        was = self.suspend
        self.suspend = True
        self.v_cname.set(c["name"] if c else "")
        self.comboList.locked = c is None
        self.comboList.set_items(c["steps"] if c else [])
        self.suspend = was
        self.update_combo_info()

    def update_combo_info(self):
        c = self.cur_combo()
        if not c:
            self.lblFunc.configure(text="")
            self.lblUsed.configure(text="Nothing here yet. Press “New combo”.")
            return
        names = combo_idents(self.model)
        clash = sum(1 for x in self.model["combos"] if ident(x["name"]) == ident(c["name"])) > 1
        self.lblFunc.configure(text=f"{names[self.comboIdx]}()" + ("  (name clash, renamed)" if clash else ""),
                               fg=C.warning if clash else C.accent)
        used = [lv["name"] for lv in self.model["levels"] if lv["combo"] == self.comboIdx]
        self.lblUsed.configure(text=("Used by level: " + ", ".join(used)) if used else
                               "Not used by any level yet. Pick it on the Scoring levels tab.")

    def on_combo_name(self):
        c = self.cur_combo()
        if self.suspend or not c:
            return
        c["name"] = self.v_cname.get() or "Combo"
        self.refresh_combo_list()
        self.refresh_level_list()
        self.pick_level(refresh=False)
        self.update_combo_info()
        self.mark_dirty()

    def on_combo_steps(self):
        self.refresh_combo_list()
        self.pick_level(refresh=False)
        self.mark_dirty()

    def add_combo(self, assign=False):
        if len(self.model["combos"]) >= MAX_COMBOS:
            self.notify(f"The robot holds at most {MAX_COMBOS} combos.", "warning")
            return
        self.model["combos"].append({"name": f"Combo{len(self.model['combos']) + 1}", "steps": []})
        self.comboIdx = len(self.model["combos"]) - 1
        if assign and self.cur():
            self.cur()["combo"] = self.comboIdx
        self.mark_dirty()
        self.refresh_all()
        if assign:
            self.tabs.select(1)

    def dup_combo(self):
        c = self.cur_combo()
        if not c:
            return
        if len(self.model["combos"]) >= MAX_COMBOS:
            self.notify(f"The robot holds at most {MAX_COMBOS} combos.", "warning")
            return
        self.model["combos"].append({"name": (c["name"] + "2")[:11], "steps": copy.deepcopy(c["steps"])})
        self.comboIdx = len(self.model["combos"]) - 1
        self.mark_dirty()
        self.refresh_all()

    def remove_combo(self):
        c = self.cur_combo()
        if not c:
            return
        i = self.comboIdx
        if any(lv["combo"] == i for lv in self.model["levels"]) and not messagebox.askyesno(
                "Remove combo", f"“{c['name']}” is used by a level. Remove it anyway? That level will have no combo."):
            return
        del self.model["combos"][i]
        for a in self.model["actions"]:
            k = a.get("combo", -1)
            a["combo"] = -1 if k == i else (k - 1 if k > i else k)
        for lv in self.model["levels"]:
            lv["combo"] = -1 if lv["combo"] == i else (lv["combo"] - 1 if lv["combo"] > i else lv["combo"])
        self.comboIdx = max(0, i - 1)
        self.mark_dirty()
        self.refresh_all()

    def edit_level_combo(self):
        lv = self.cur()
        if lv and lv["combo"] >= 0:
            self.comboIdx = lv["combo"]
            self.refresh_all()
            self.tabs.select(1)
        else:
            self.notify("This level has no combo yet. Pick one in the box, or press “New combo”.", "warning")

    # -- levels
    def refresh_level_list(self):
        levels = self.model["levels"]
        rows, tags = [], []
        for i, lv in enumerate(levels):
            here = i == self.curLevel
            rows.append((i + 1, lv["name"], f"{lv['angle']:.0f}°", self.combo_name(lv["combo"]) or "(none)",
                         "here" if here else ""))
            tags.append("now" if here else None)
        self.lstLevels.set_rows(rows, self.levelIdx if levels else None, tags)
        self.secLevels.meta.configure(text=f"{len(levels)} of {MAX_LEVELS}")

    def cur(self):
        return self.model["levels"][self.levelIdx] if self.model["levels"] else None

    def pick_level(self, refresh=True):
        if refresh:
            sel = self.lstLevels.selected()
            if sel is not None:
                self.levelIdx = sel
        lv = self.cur()
        was = self.suspend
        self.suspend = True
        self.v_lname.set(lv["name"] if lv else "")
        self.v_langle.set(f"{lv['angle']:.1f}" if lv else "")
        self.cbCombo.current((lv["combo"] + 1) if lv and 0 <= lv["combo"] < len(self.model["combos"]) else 0)
        self.suspend = was
        steps = []
        if lv and 0 <= lv["combo"] < len(self.model["combos"]):
            steps = self.model["combos"][lv["combo"]]["steps"]
        self.lblSteps.configure(text=("\n".join(f"{i + 1}.  {step_summary(s)}" for i, s in enumerate(steps)) if steps else
                                      ("This combo has no steps yet." if lv and lv["combo"] >= 0 else
                                       "No combo: the lift just drives to this angle and stops.")),
                                fg=C.text if steps else C.muted)
        self.draw_gauge()

    def on_level_combo(self):
        lv = self.cur()
        if self.suspend or not lv:
            return
        lv["combo"] = self.cbCombo.current() - 1
        self.refresh_level_list()
        self.pick_level(refresh=False)
        self.update_combo_info()
        self.mark_dirty()

    def on_start_edit(self):
        if self.suspend:
            return
        m = self.model
        m["startEnabled"] = self.v_startEn.get()
        m["start"] = num(self.v_start, m["start"])
        m["limit"] = max(1.0, num(self.v_limit, m["limit"]))
        self.mark_dirty()

    def on_level_edit(self):
        lv = self.cur()
        if self.suspend or not lv:
            return
        lv["name"] = self.v_lname.get() or "Level"
        lv["angle"] = num(self.v_langle, lv["angle"])
        self.refresh_level_list()
        self.mark_dirty()

    def on_model_edit(self):
        self.refresh_level_list()
        self.mark_dirty()

    def mark_dirty(self):
        self.dirty = True
        self.draw_gauge()
        self.update_status()

    def add_level(self):
        if len(self.model["levels"]) >= MAX_LEVELS:
            self.notify(f"The robot holds at most {MAX_LEVELS} levels.", "warning")
            return
        self.model["levels"].append({"name": f"Level{len(self.model['levels']) + 1}", "angle": round(self.live, 1), "combo": -1})
        self.levelIdx = len(self.model["levels"]) - 1
        self.mark_dirty()
        self.refresh_all()

    def remove_level(self):
        if self.cur():
            del self.model["levels"][self.levelIdx]
            self.levelIdx = max(0, self.levelIdx - 1)
            self.mark_dirty()
            self.refresh_all()

    def move_level(self, d):
        lv, i = self.model["levels"], self.levelIdx
        if lv and 0 <= i + d < len(lv):
            lv[i], lv[i + d] = lv[i + d], lv[i]
            self.levelIdx = i + d
            self.mark_dirty()
            self.refresh_all()

    # ── status + messages ──
    def set_conn(self, text, kind):
        col = {"ok": C.success, "wait": C.warning, "bad": C.danger}[kind]
        self.lblConn.configure(text=text, fg=col)
        paint_dot(self.dotConn, col)

    def notify(self, text, kind="info"):
        """One line at the right of the status bar. Success and errors stay until replaced."""
        col = {"info": C.muted, "success": C.success, "warning": C.warning, "danger": C.danger}[kind]
        self.lblNote.configure(text=text, fg=col)
        if self._noteJob:
            self.after_cancel(self._noteJob)
            self._noteJob = None
        if kind in ("info", "warning"):
            self._noteJob = self.after(8000, lambda: self.lblNote.configure(text=""))

    def update_status(self):
        if self.link:
            self.set_conn("Demo robot" if self.demo else f"Connected to {getattr(self.link, 'port', 'brain')}", "ok")
        if self.dirty:
            col, text, hint = C.warning, "Unsent edits", "Press Apply to robot (Ctrl+Enter) to try them."
        elif self.link:
            col, text, hint = C.success, "In sync with robot", ""
        else:
            col, text, hint = C.subtle, "Not connected", ""
        paint_dot(self.dotSync, col)
        self.lblSync.configure(text=text, fg=C.text if self.dirty else C.muted)
        self.lblHint.configure(text=hint)
        self.btnApply.configure(variant="primary" if self.dirty else "secondary")

    # ── robot ──
    def connect(self):
        if self.link:
            self.link.close()
            self.link = None
        if self.demo:
            self.link = FakeLink(self.model)
        else:
            try:
                import serial  # noqa: F401
            except ImportError:
                self.set_conn("pyserial missing", "bad")
                messagebox.showerror("pyserial missing", "Install it with:\n    pip install pyserial")
                return
            port = find_brain_port()
            if not port:
                self.set_conn("No brain found", "bad")
                self.notify("Plug in the brain over USB and close pros terminal, then press Reconnect.", "danger")
                self.update_status()
                return
            try:
                self.link = SerialLink(port)
            except Exception as e:
                self.set_conn(f"Can't open {port}", "bad")
                self.notify(str(e), "danger")
                return
        self.loaded_from_robot = False
        self.update_status()

    def send(self, lines):
        if not self.link:
            self.set_conn("Not connected", "bad")
            self.notify("Not connected. Press Reconnect.", "danger")
            return
        self.link.send(lines)

    def stop(self):
        self.send(["LT STOP"])

    def apply(self):
        self.send(model_to_commands(self.model))
        self.save_backup()
        self.dirty = False
        self.update_status()

    def pull(self):
        if self.dirty and not messagebox.askyesno("Discard edits?", "Replace what's on screen with what the robot has?"):
            return
        self.loaded_from_robot = False
        self.dirty = False
        self.update_status()
        self.notify("Asked the robot for its settings…", "info")

    def save_all(self):
        """One tap: code + robot (apply and SD card) + local backup."""
        ok, note = self.write_code()
        if not ok:
            return
        parts = [note]
        if self.link:
            self.apply()
            self.send(["LT SAVE"])
            parts.append("robot SD card")
        else:
            self.save_backup()
            self.dirty = False
            self.update_status()
            parts.append("robot not connected, so not sent")
        self.notify(f"Saved {time.strftime('%H:%M:%S')}: " + ", ".join(parts), "success")

    def save_backup(self):
        if self.demo:
            return
        try:
            with open(BACKUP, "w", encoding="utf-8") as f:
                json.dump(self.model, f, indent=1)
        except OSError:
            pass

    def write_code(self):
        """Writes the defaults + one function per combo into the source. -> (ok, note)"""
        m = self.model
        n = len(m["combos"])
        if self.demo:
            return True, f"demo mode, code not written ({n} combo{'s' if n != 1 else ''} would become functions)"
        try:
            with open(SOURCE, encoding="utf-8", newline="") as f:
                src = f.read()
            with open(HEADER_H, encoding="utf-8", newline="") as f:
                hdr = f.read()
            nl = "\r\n" if "\r\n" in src else "\n"
            src = replace_region(src, BEGIN, END, code_block(m), nl, "    ")
            src = replace_region(src, FUNCS_BEGIN, FUNCS_END, combo_functions(m), nl)
            hdr = replace_region(hdr, DECLS_BEGIN, DECLS_END, combo_decls(m), "\r\n" if "\r\n" in hdr else "\n")
        except (OSError, ValueError) as e:
            messagebox.showerror("Can't write the code", str(e))
            return False, str(e)
        try:
            with open(SOURCE, "w", encoding="utf-8", newline="") as f:
                f.write(src)
            with open(HEADER_H, "w", encoding="utf-8", newline="") as f:
                f.write(hdr)
        except OSError as e:
            messagebox.showerror("Can't write the code", str(e))
            return False, str(e)
        self.save_backup()
        return True, f"code updated, {n} combo function{'s' if n != 1 else ''}"

    def run_level(self):
        if self.cur():
            if self.dirty:
                self.apply()
            self.send([f"LT RUNLEVEL {self.levelIdx}"])

    def test_combo(self):
        if self.cur_combo():
            if self.dirty:
                self.apply()
            self.send([f"LT TESTCOMBO {self.comboIdx}"])

    def test_combo_step(self, i):
        if self.dirty:
            self.apply()
        self.send([f"LT TESTSTEP {self.comboIdx} {i}"])

    def test_action(self, i):
        if self.dirty:
            self.apply()
        self.send([f"LT TESTACT {i}"])

    def poll(self):
        if self.link:
            if getattr(self.link, "tick", None):
                self.link.tick()
            if self.link.dead:
                self.set_conn("Connection lost", "bad")
                self.notify("Lost the connection to the brain. Press Reconnect.", "danger")
                self.link = None
                self.update_status()
            else:
                try:
                    while True:
                        self.on_line(self.link.lines.get_nowait())
                except queue.Empty:
                    pass
        self.after(60, self.poll)

    def on_line(self, line):
        if line.startswith("LTL|"):
            s = parse_ltl(line)
            if s:
                prevLevel = self.curLevel
                self.live, self.moving, self.combo_running, self.curLevel = s["live"], s["moving"], s["combo"], s["level"]
                if prevLevel != self.curLevel and self.model["levels"]:
                    self.refresh_level_list()
                self.lblLive.configure(text=f"{self.live:.1f}°", fg=C.warning if self.moving else C.text)
                self.lblMoving.configure(text="Running a combo…" if self.combo_running else ("Moving…" if self.moving else "Idle"))
                self.draw_gauge()
        elif line.startswith("LTC|") and not self.dirty and not self.loaded_from_robot:
            m = parse_ltc(line)
            if m and m != self.model:
                self.model = m
                self.refresh_all()
                self.notify("Loaded the robot's settings.", "info")
            self.loaded_from_robot = True
            self.update_status()

    # ── gauge ──
    def scale(self):
        m = self.model
        vals = [m["limit"], self.live, 0.0] + [l["angle"] for l in m["levels"]] + ([m["start"]] if m["startEnabled"] else [])
        lo, hi = min(vals + [0.0]), max(vals + [10.0])
        span = hi - lo
        return lo - span * 0.02, hi + span * 0.05

    def _gauge_y(self):
        return S(16), self.gauge.winfo_height() - S(16)

    def _gauge_angle(self, y):
        lo, hi = self.scale()
        top, bot = self._gauge_y()
        y = max(top, min(bot, y))
        return lo + (bot - y) / max(1, bot - top) * (hi - lo)

    def gauge_click(self, e):
        a = self._gauge_angle(e.y)
        self.send([f"LT GO {a:.2f}"])
        self.v_go.set(f"{a:.1f}")

    def gauge_hover(self, e):
        top, bot = self._gauge_y()
        g = self.gauge
        g.delete("hover")
        y = max(top, min(bot, e.y))
        g.create_line(S(44), y, S(84), y, fill=C.muted, dash=(2, 3), tags="hover")
        t = g.create_text(S(92), y, text=f"Go to {self._gauge_angle(e.y):.0f}°", fill=C.text, anchor="w",
                          font=fnt(T["caption"], "bold"), tags="hover")
        x0, y0, x1, y1 = g.bbox(t)
        r = g.create_rectangle(x0 - S(4), y0 - S(2), x1 + S(4), y1 + S(2), fill=C.sunken, outline=C.line_strong, tags="hover")
        g.tag_lower(r, t)

    def draw_gauge(self):
        g = self.gauge
        g.delete("all")
        W, H = g.winfo_width(), g.winfo_height()
        if W < S(120) or H < S(80):
            return
        lo, hi = self.scale()
        top, bot = self._gauge_y()
        yo = lambda a: bot - (a - lo) / (hi - lo) * (bot - top)
        tx0, tx1 = S(48), S(76)
        # scale
        step = nice_step(hi - lo)
        a = math.ceil(lo / step) * step
        while a <= hi:
            g.create_line(S(38), yo(a), S(44), yo(a), fill=C.line_strong)
            g.create_text(S(33), yo(a), text=f"{a:.0f}", fill=C.muted, anchor="e", font=fnt(T["caption"]))
            a += step
        # track + live fill
        g.create_rectangle(tx0, top, tx1, bot, fill=C.sunken, outline=C.line_strong)
        g.create_rectangle(tx0 + 1, yo(max(self.live, lo)), tx1 - 1, bot - 1,
                           fill=C.warning if self.moving else C.fill_idle, outline="")
        g.create_polygon(tx0 - S(2), yo(self.live), tx0 - S(9), yo(self.live) - S(5), tx0 - S(9), yo(self.live) + S(5),
                         fill=C.text, outline="")
        # markers, with labels that never overlap
        m = self.model
        marks = []   # (y, label, colour, bold)
        g.create_line(S(44), yo(m["limit"]), S(82), yo(m["limit"]), fill=C.danger, width=S(2))
        marks.append((yo(m["limit"]), f"Limit {m['limit']:.0f}°", C.danger, False))
        if m["startEnabled"]:
            g.create_line(S(44), yo(m["start"]), S(82), yo(m["start"]), fill=C.marker, width=S(2), dash=(4, 2))
            marks.append((yo(m["start"]), f"Start {m['start']:.0f}°", C.marker, False))
        for i, lv in enumerate(m["levels"]):
            sel = i == self.levelIdx
            g.create_line(S(44), yo(lv["angle"]), S(82), yo(lv["angle"]), fill=C.accent, width=S(3) if sel else S(2))
            marks.append((yo(lv["angle"]), f"{lv['name']} {lv['angle']:.0f}°", C.accent if sel else C.muted, sel))
        for ac in m["actions"]:
            y = yo(ac["angle"])
            cx, r = (tx0 + tx1) // 2, S(4)
            g.create_polygon(cx, y - r, cx + r, y, cx, y + r, cx - r, y, fill=C.warning, outline=C.sunken)
            marks.append((y, f"Fn {ac['angle']:.0f}° {('up', 'down', 'any')[ac['when']]}", C.warning, False))
        gap, ys = S(14), []
        if len(marks) * gap > bot - top:            # too crowded for this height: keep the important labels
            marks = [t for t in marks if not t[1].startswith("Fn ")]
        marks.sort(key=lambda t: t[0])
        for y, *_ in marks:
            ys.append(max(y, ys[-1] + gap) if ys else y)
        ceiling = bot + S(8)
        for k in range(len(ys) - 1, -1, -1):
            lim = ceiling if k == len(ys) - 1 else ys[k + 1] - gap
            ys[k] = max(top - S(6), min(ys[k], lim))
        for (y, label, col, bold), ly in zip(marks, ys):
            if abs(ly - y) > 1:
                g.create_line(S(82), y, S(88), ly, fill=C.line_strong)
            g.create_text(S(92), ly, text=label, fill=col, anchor="w", font=fnt(T["caption"], "bold" if bold else "normal"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="fake robot, no brain needed")
    args = ap.parse_args()
    enable_hidpi()
    App(args.demo).mainloop()


if __name__ == "__main__":
    main()
