#!/usr/bin/env python3
"""
Lift Tuner -- the laptop half of the robot's LIFT TUNER (hold DPAD-RIGHT when
driver control starts).

    python tools/lift_tuner.py           connect to the brain over the USB cable
    python tools/lift_tuner.py --demo    no robot: a fake lift, to try the layout

Needs pyserial (pip install pyserial). Close `pros terminal` first -- only one
program can hold the brain's serial port.

WHAT YOU CAN SET (four tabs)
    Start & Limits      START POSITION: the angle the lift is assumed to be at when
                        driver control starts. FORWARD LIMIT: how far X may drive it.
    Combos              Your library of named combos -- add as many as you like and
                        edit any of them any time. A combo is a list of steps on any
                        motor or piston (left/right drive, intake, lift, claw arm,
                        claw piston, lift-arm orientation). Each
                        combo is saved into the code as a function, e.g. combo_Score().
    Scoring Levels      Named lift heights; each one plays a combo of your choice once
                        the lift gets there (UP / LEFT on the controller step through
                        the levels).
    Angle Functions     "When the lift passes ANGLE going up / down / either, do this
                        step" -- fires however the lift got there.

THE WORKFLOW
    1. Move the lift (jog buttons, click the gauge, or X / B in the on-robot tuner) and
       press "Use live" next to a number to capture the lift's real angle.
    2. Build combos, pick them for levels. Test buttons run them on the robot.
    3. APPLY TO ROBOT  (Ctrl+Enter) sends everything to the brain to try it.
    4. SAVE  (Ctrl+S) applies it, stores it on the brain's SD card AND writes it into
       the code (src/lift_tuning.cpp + include/lift_tuning.hpp: defaults plus one
       function per combo). Rebuild and upload afterwards.
    STOP (Esc) halts the lift and every running step.

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
from tkinter import messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(os.path.dirname(HERE), "src", "lift_tuning.cpp")
BACKUP = os.path.join(HERE, "lift_tuning.json")
HEADER_H = os.path.join(os.path.dirname(HERE), "include", "lift_tuning.hpp")
BEGIN, END = "// LIFT-TUNING-BEGIN", "// LIFT-TUNING-END"
FUNCS_BEGIN, FUNCS_END = "// LIFT-COMBO-FUNCS-BEGIN", "// LIFT-COMBO-FUNCS-END"
DECLS_BEGIN, DECLS_END = "// LIFT-COMBO-DECLS-BEGIN", "// LIFT-COMBO-DECLS-END"

# Palette -- same family as the brain's screens.
BG = "#0d1119"
HEADER = "#111620"
CARD = "#161c26"
EDGE = "#262f3c"
TRACK = "#2a323e"
TEXT = "#e6edf3"
DIM = "#8b949e"
OK = "#3fb950"
WARM = "#f0a830"
HOT = "#f85149"
INFO = "#58a6ff"
GOLD = "#f2c230"
GAUGE_H = 320
FONT = "Segoe UI"
MONO = "Consolas"

DEVICES = ["Left drive", "Right drive", "Intake", "Lift", "Claw arm",
           "Claw piston", "Lift-arm orientation"]   # index == LiftDevice
FIRST_PISTON = 5
MOTOR_CMDS = [("Forward", 0), ("Backward", 1), ("Stop", 2)]
PISTON_CMDS = [("Extend", 3), ("Retract", 4), ("Toggle", 5)]
WHENS = ["Going up", "Going down", "Either way"]
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
                    new_step(dev=2, cmd=1, speed=200, dur=800)]}],
            "levels": [
                {"name": "Ground", "angle": 0.0, "combo": 0},
                {"name": "Low", "angle": 180.0, "combo": 1},
                {"name": "Mid", "angle": 420.0, "combo": 2},
                {"name": "High", "angle": 700.0, "combo": -1}],
            "actions": [
                {"angle": 300.0, "when": 0, "step": new_step(dev=6, cmd=3)},
                {"angle": 200.0, "when": 1, "step": new_step(dev=6, cmd=4)}]}


def step_summary(s, with_delay=True):
    what = f"{DEVICES[s['dev']]}: {cmd_name(s['dev'], s['cmd'])}"
    if not is_piston(s["dev"]) and s["cmd"] != 2:
        what += f" {s['speed']} rpm for {s['dur']} ms"
    if with_delay:
        what = (f"+{s['delay']} ms  " if s["delay"] else "now  ") + what
    return what


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
        out.append(f"LT ACT {a['angle']:.2f} {a['when']} {s['dev']} {s['cmd']} {s['speed']} {s['dur']}")
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
            ang, when, d, dev, cmd, sp, du = ac.split(",")
            m["actions"].append({"angle": float(ang), "when": int(when),
                                 "step": {"delay": int(d), "dev": int(dev), "cmd": int(cmd),
                                          "speed": int(sp), "dur": int(du)}})
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
                 f"{{0, {s['dev']}, {s['cmd']}, {s['speed']}, {s['dur']}}}}};")
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
                a, w, dev, c, sp, du = rest.split()
                pd["actions"].append({"angle": float(a), "when": int(w), "step": {
                    "delay": 0, "dev": int(dev), "cmd": int(c), "speed": int(sp), "dur": int(du)}})
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
                          f"{a['step']['speed']},{a['step']['dur']}" for a in m["actions"])
            self.lines.put(f"LTC|{1 if m['startEnabled'] else 0},{m['start']:.2f},{m['limit']:.2f}|{cb}|{lv}|{ac}|")

    def close(self):
        pass


# -- UI -------------------------------------------------------------------------
def style_ttk(root):
    st = ttk.Style(root)
    st.theme_use("clam")
    st.configure(".", background=BG, foreground=TEXT, fieldbackground=CARD, font=(FONT, 10))
    st.configure("TNotebook", background=BG, borderwidth=0)
    st.configure("TNotebook.Tab", background=HEADER, foreground=DIM, padding=(18, 8), font=(FONT, 10, "bold"))
    st.map("TNotebook.Tab", background=[("selected", CARD)], foreground=[("selected", INFO)])
    st.configure("Treeview", background=CARD, fieldbackground=CARD, foreground=TEXT, rowheight=26,
                 borderwidth=0, font=(FONT, 10))
    st.configure("Treeview.Heading", background=HEADER, foreground=DIM, font=(FONT, 9, "bold"), relief="flat")
    st.map("Treeview", background=[("selected", "#1f3a5c")], foreground=[("selected", TEXT)])
    st.configure("TCombobox", fieldbackground=CARD, background=EDGE, foreground=TEXT, arrowcolor=TEXT)
    st.map("TCombobox", fieldbackground=[("readonly", CARD)], foreground=[("readonly", TEXT)])
    root.option_add("*TCombobox*Listbox.background", CARD)
    root.option_add("*TCombobox*Listbox.foreground", TEXT)


class Tooltip:
    """Small hover hint. Appears after a short pause, never steals focus."""

    def __init__(self, widget, text):
        self.widget, self.text, self.win, self.job = widget, text, None, None
        widget.bind("<Enter>", self.schedule, add="+")
        widget.bind("<Leave>", self.hide, add="+")
        widget.bind("<ButtonPress>", self.hide, add="+")

    def schedule(self, _e=None):
        self.job = self.widget.after(450, self.show)

    def show(self):
        if self.win or not self.text:
            return
        x, y = self.widget.winfo_rootx() + 8, self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        self.win = tk.Toplevel(self.widget)
        self.win.wm_overrideredirect(True)
        self.win.wm_geometry(f"+{x}+{y}")
        tk.Label(self.win, text=self.text, bg="#0b0e14", fg=TEXT, font=(FONT, 9), justify="left", wraplength=300,
                 padx=9, pady=6, highlightthickness=1, highlightbackground=EDGE).pack()

    def hide(self, _e=None):
        if self.job:
            self.widget.after_cancel(self.job)
            self.job = None
        if self.win:
            self.win.destroy()
            self.win = None


def button(parent, text, cmd, bg=EDGE, fg=TEXT, **kw):
    tip = kw.pop("tip", None)
    b = _button(parent, text, cmd, bg, fg, **kw)
    if tip:
        Tooltip(b, tip)
    return b


def _button(parent, text, cmd, bg=EDGE, fg=TEXT, **kw):
    b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg, activebackground="#34404f",
                  activeforeground=fg, disabledforeground="#4a5360", relief="flat", font=(FONT, 9, "bold"), cursor="hand2",
                  padx=kw.pop("padx", 10), pady=kw.pop("pady", 5), bd=0, **kw)
    return b


def label(parent, text, fg=DIM, size=9, bold=False, **kw):
    return tk.Label(parent, text=text, bg=kw.pop("bg", CARD), fg=fg,
                    font=(FONT, size, "bold" if bold else "normal"), **kw)


def entry(parent, var, width=8):
    return tk.Entry(parent, textvariable=var, width=width, bg=BG, fg=TEXT, insertbackground=TEXT,
                    relief="flat", font=(MONO, 11), highlightthickness=1, highlightbackground=EDGE,
                    highlightcolor=INFO, justify="right")


def card(parent, title=None, **kw):
    f = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=EDGE, **kw)
    if title:
        label(f, title.upper(), size=8, bold=True).pack(anchor="w", padx=12, pady=(10, 2))
    return f


def num(var, default=0.0):
    try:
        return float(var.get())
    except (ValueError, tk.TclError):
        return default


class StepList(tk.Frame):
    """A table of steps with an editor underneath. kind='combo' (levels: has an
    'after' delay) or 'action' (angle functions: has an angle + when)."""

    def __init__(self, parent, app, kind, on_change, on_test, live_angle):
        super().__init__(parent, bg=CARD)
        self.app, self.kind, self.on_change, self.on_test, self.live_angle = app, kind, on_change, on_test, live_angle
        self.items = []
        self.locked = False
        cols = ("n", "what") if kind == "combo" else ("n", "when", "what")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=4, selectmode="browse")
        heads = {"n": ("#", 36), "what": ("Step", 380), "when": ("Trigger", 170)}
        for c in cols:
            self.tree.heading(c, text=heads[c][0] if c != "what" else
                              ("Step (starts after the previous step started)" if kind == "combo" else "Then do"))
            self.tree.column(c, width=heads[c][1], anchor="w", stretch=(c == "what"))
        self.tree.pack(fill="x", pady=(0, 4))
        self.tree.bind("<<TreeviewSelect>>", lambda e: (self.load_form(), self.update_state()))
        self.hint = label(self, "", fg=INFO, size=9)
        self.hint.pack(anchor="w", pady=(0, 8))

        form = tk.Frame(self, bg=CARD)
        form.pack(fill="x")
        self.v_angle = tk.StringVar(value="0")
        self.v_when = tk.StringVar(value=WHENS[0])
        self.v_delay = tk.StringVar(value="0")
        self.v_dev = tk.StringVar(value=DEVICES[4])
        self.v_cmd = tk.StringVar(value="Forward")
        self.v_speed = tk.StringVar(value="100")
        self.v_dur = tk.StringVar(value="500")
        col = 0

        def field(title, widget):
            nonlocal col
            label(form, title, size=8).grid(row=0, column=col, sticky="w", padx=(0, 8))
            widget.grid(row=1, column=col, sticky="w", padx=(0, 8), pady=(0, 4))
            col += 1

        if kind == "action":
            f = tk.Frame(form, bg=CARD)
            entry(f, self.v_angle, 7).pack(side="left")
            button(f, "Use live", self.use_live, pady=2, padx=6).pack(side="left", padx=(4, 0))
            field("When the lift passes (deg)", f)
            field("Direction", ttk.Combobox(form, textvariable=self.v_when, values=WHENS, state="readonly", width=11))
        else:
            field("After (ms)", entry(form, self.v_delay, 6))
        self.cb_dev = ttk.Combobox(form, textvariable=self.v_dev, values=DEVICES, state="readonly", width=20)
        self.cb_dev.bind("<<ComboboxSelected>>", lambda e: self.refresh_cmds())
        field("Device", self.cb_dev)
        self.cb_cmd = ttk.Combobox(form, textvariable=self.v_cmd, state="readonly", width=9)
        field("Command", self.cb_cmd)
        self.e_speed, self.e_dur = entry(form, self.v_speed, 5), entry(form, self.v_dur, 6)
        field("Speed (rpm)", self.e_speed)
        field("Run for (ms)", self.e_dur)
        self.refresh_cmds()

        row = tk.Frame(self, bg=CARD)
        row.pack(fill="x", pady=(6, 0))
        button(row, "+ Add step", self.add, bg="#143a1e", fg=OK,
               tip="Adds the fields above as a NEW step at the end of the list.").pack(side="left")
        self.btnUpdate = button(row, "Update selected", self.update_sel,
                                tip="Overwrites the selected step with the fields above.")
        self.btnUpdate.pack(side="left", padx=6)
        self.btnRemove = button(row, "Remove", self.remove, bg="#3a1414", fg=HOT)
        self.btnRemove.pack(side="left")
        self.btnUp = button(row, "▲", lambda: self.move(-1), padx=8, tip="Move the selected step earlier.")
        self.btnUp.pack(side="left", padx=(14, 2))
        self.btnDown = button(row, "▼", lambda: self.move(1), padx=8, tip="Move the selected step later.")
        self.btnDown.pack(side="left")
        self.btnTest = button(row, "▶ Test step on robot", self.test, bg="#1c2838", fg=INFO,
                              tip="Runs just the selected step on the robot now. Applies unsent edits first.")
        self.btnTest.pack(side="right")
        self.update_state()

    def use_live(self):
        self.v_angle.set(f"{self.live_angle():.1f}")

    def refresh_cmds(self):
        dev = DEVICES.index(self.v_dev.get())
        cmds = PISTON_CMDS if is_piston(dev) else MOTOR_CMDS
        self.cb_cmd["values"] = [c[0] for c in cmds]
        if self.v_cmd.get() not in [c[0] for c in cmds]:
            self.v_cmd.set(cmds[0][0])
        state = "disabled" if (is_piston(dev) or self.v_cmd.get() == "Stop") else "normal"
        for e in (self.e_speed, self.e_dur):
            e.config(state=state)

    def read_form(self):
        dev = DEVICES.index(self.v_dev.get())
        cmds = dict(PISTON_CMDS if is_piston(dev) else MOTOR_CMDS)
        step = new_step(dev=dev, cmd=cmds[self.v_cmd.get()],
                        speed=int(max(1, min(200, num(self.v_speed, 100)))),
                        dur=int(max(50, min(10000, num(self.v_dur, 500)))),
                        delay=int(max(0, min(20000, num(self.v_delay, 0)))) if self.kind == "combo" else 0)
        if self.kind == "action":
            return {"angle": round(num(self.v_angle), 1), "when": WHENS.index(self.v_when.get()), "step": step}
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
            b.config(state="normal" if i is not None else "disabled")
        if self.locked:
            self.hint.config(text="No combo selected. Press “+ New combo” on the left to make one.")
        elif not self.items:
            self.hint.config(text="No steps yet. Pick a device and command below, then press “+ Add step”.")
        elif i is None:
            self.hint.config(text="Click a step to edit it, or fill in the fields and press “+ Add step” for a new one.")
        else:
            self.hint.config(text=f"Editing step {i + 1}: change the fields, then press “Update selected”.")

    def selected(self):
        sel = self.tree.selection()
        return self.tree.index(sel[0]) if sel else None

    def set_items(self, items, keep=None):
        self.items = items
        self.redraw(keep)

    def redraw(self, keep=None):
        self.tree.delete(*self.tree.get_children())
        for i, it in enumerate(self.items):
            if self.kind == "action":
                self.tree.insert("", "end", values=(i + 1, f"{WHENS[it['when']]} @ {it['angle']:.1f}°",
                                                    step_summary(it["step"], False)))
            else:
                self.tree.insert("", "end", values=(i + 1, step_summary(it)))
        kids = self.tree.get_children()
        if keep is not None and 0 <= keep < len(kids):
            self.tree.selection_set(kids[keep])
        self.update_state()

    def add(self):
        if self.locked:
            messagebox.showinfo("Make a combo first", "Press “+ New combo” on the left, then add steps to it.")
            return
        limit = 8 if self.kind == "action" else 6
        if len(self.items) >= limit:
            messagebox.showinfo("Full", f"At most {limit} entries fit in the robot's memory.")
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
            messagebox.showinfo("Pick a step", "Select a step in the table first.")
            return
        self.on_test(i)


class App(tk.Tk):
    def __init__(self, demo):
        super().__init__()
        self.title("Lift Tuner — V5")
        self.configure(bg=BG)
        self.geometry("1120x640")
        self.minsize(980, 600)
        style_ttk(self)
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
        if demo:
            self.model = sample_model()
        else:
            try:
                with open(BACKUP, encoding="utf-8") as f:
                    self.model = json.load(f)
            except (OSError, ValueError):
                pass
        self._build()
        self.bind("<Escape>", lambda e: self.stop())
        self.bind("<Control-Return>", lambda e: self.apply())
        self.bind("<Control-s>", lambda e: self.save_all())
        self.refresh_all()
        self.connect()
        self.after(60, self.poll)

    # ── layout ──
    def _build(self):
        top = tk.Frame(self, bg=HEADER)
        top.pack(fill="x")
        tk.Label(top, text="LIFT TUNER", bg=HEADER, fg=TEXT, font=(FONT, 15, "bold")).pack(side="left", padx=16, pady=10)
        self.chipConn = tk.Label(top, text="●  connecting…", bg=HEADER, fg=WARM, font=(FONT, 9, "bold"))
        self.chipConn.pack(side="left", padx=8)
        self.chipSync = tk.Label(top, text="", bg=HEADER, fg=DIM, font=(FONT, 9, "bold"))
        self.chipSync.pack(side="left", padx=8)
        self.chipSaved = tk.Label(top, text="", bg=HEADER, fg=OK, font=(FONT, 9, "bold"))
        self.chipSaved.pack(side="left", padx=8)
        button(top, "■ STOP  (Esc)", self.stop, bg="#3a1414", fg=HOT,
               tip="Halts the lift and every running step on the robot, immediately.").pack(side="right", padx=(6, 14), pady=8)
        self.btnSave = button(top, "💾 SAVE  (Ctrl+S)", self.save_all, bg="#1c3a66", fg="#ffffff",
                              tip="One tap: sends everything to the robot, saves it on the brain's SD card, and writes it into the code "
                                  "(defaults + one function per combo). Rebuild and upload afterwards.")
        self.btnSave.pack(side="right", padx=4, pady=8)
        self.btnApply = button(top, "▲ APPLY TO ROBOT  (Ctrl+Enter)", self.apply, bg="#143a1e", fg=OK,
                               tip="Sends everything on screen to the robot. It takes effect immediately, but is lost on power-off until saved.")
        self.btnApply.pack(side="right", padx=4, pady=8)
        button(top, "Load from robot", self.pull,
               tip="Throws away what's on screen and shows what the robot currently has.").pack(side="right", padx=4, pady=8)
        button(top, "⟳", self.connect, padx=8, tip="Reconnect to the brain.").pack(side="right", padx=4, pady=8)

        strip = tk.Frame(self, bg=BG)
        strip.pack(fill="x", padx=14, pady=(10, 0))
        self.stepLabels = []
        for n, txt in enumerate(("1  Move the lift, press “Use live”", "2  Edit levels and steps",
                                 "3  Apply to robot", "4  Save (code + SD)")):
            l = tk.Label(strip, text=txt, bg=CARD, fg=DIM, font=(FONT, 9, "bold"), padx=12, pady=5,
                         highlightthickness=1, highlightbackground=EDGE)
            l.pack(side="left", padx=(0, 6))
            self.stepLabels.append(l)

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=14, pady=10)

        # left: live gauge + jog
        left = card(body, "Lift — live")
        left.pack(side="left", fill="y", padx=(0, 12))
        self.lblLive = tk.Label(left, text="0.0°", bg=CARD, fg=OK, font=(FONT, 26, "bold"))
        self.lblLive.pack(padx=14)
        self.lblMoving = label(left, "idle", size=9)
        self.lblMoving.pack()
        self.gauge = tk.Canvas(left, width=190, height=GAUGE_H, bg=CARD, highlightthickness=0, cursor="hand2")
        self.gauge.pack(padx=10, pady=6)
        self.gauge.bind("<Button-1>", self.gauge_click)
        self.gauge.bind("<Motion>", self.gauge_hover)
        self.gauge.bind("<Leave>", lambda e: self.gauge.delete("hover"))
        label(left, "click the gauge to send the lift there", size=8).pack()
        jog = tk.Frame(left, bg=CARD)
        jog.pack(pady=8)
        for i, j in enumerate(JOGS):
            button(jog, f"{j:+d}", lambda j=j: self.send([f"LT JOG {j}"]), padx=6, pady=3).grid(row=0, column=i, padx=1)
        goRow = tk.Frame(left, bg=CARD)
        goRow.pack(pady=(0, 12))
        self.v_go = tk.StringVar(value="0")
        label(goRow, "go to").pack(side="left")
        entry(goRow, self.v_go, 7).pack(side="left", padx=4)
        button(goRow, "GO", lambda: self.send([f"LT GO {num(self.v_go):.2f}"]), bg="#1c2838", fg=INFO, pady=2).pack(side="left")

        # right: tabs
        self.nb = ttk.Notebook(body)
        self.nb.pack(side="left", fill="both", expand=True)
        self._tab_start()
        self._tab_combos()
        self._tab_levels()
        self._tab_actions()

    def _tab_start(self):
        f = tk.Frame(self.nb, bg=BG)
        self.nb.add(f, text="  Start & Limits  ")
        c = card(f, "Starting position")
        c.pack(fill="x", padx=2, pady=(10, 8))
        label(c, "Where the lift is assumed to be when driver control starts, in motor degrees. "
                 "Turn it off to keep whatever the encoder says (zeroed at power-on).", size=9, wraplength=620,
              justify="left").pack(anchor="w", padx=12)
        r = tk.Frame(c, bg=CARD)
        r.pack(fill="x", padx=12, pady=12)
        self.v_startEn = tk.BooleanVar()
        tk.Checkbutton(r, text="Set the lift to this position when driver control starts", variable=self.v_startEn,
                       command=self.on_start_edit, bg=CARD, fg=TEXT, selectcolor=BG, activebackground=CARD,
                       activeforeground=TEXT, font=(FONT, 10, "bold")).pack(side="left")
        self.v_start = tk.StringVar()
        entry(r, self.v_start, 8).pack(side="left", padx=(16, 4))
        label(r, "deg").pack(side="left")
        button(r, "Use live", lambda: (self.v_start.set(f"{self.live:.1f}"), self.v_startEn.set(True), self.on_start_edit()),
               pady=3, tip="Copies the lift's current angle into this box and turns the start position on.").pack(side="left", padx=8)
        c2 = card(f, "Forward limit")
        c2.pack(fill="x", padx=2, pady=8)
        label(c2, "X stops driving the lift forward at this angle. A (backward) is never limited.", size=9).pack(anchor="w", padx=12)
        r2 = tk.Frame(c2, bg=CARD)
        r2.pack(fill="x", padx=12, pady=12)
        self.v_limit = tk.StringVar()
        entry(r2, self.v_limit, 8).pack(side="left")
        label(r2, "deg").pack(side="left", padx=4)
        button(r2, "Use live", lambda: (self.v_limit.set(f"{self.live:.1f}"), self.on_start_edit()), pady=3).pack(side="left", padx=8)
        for v in (self.v_start, self.v_limit):
            v.trace_add("write", lambda *a: self.on_start_edit())

    def _tab_combos(self):
        f = tk.Frame(self.nb, bg=BG)
        self.nb.add(f, text="  Combos  ")
        label(f, "A combo is a named list of steps. Make as many as you like, edit any of them later, then pick one for each "
                 "scoring level. Every combo is also saved into the code as a function.", size=9, bg=BG, wraplength=760,
              justify="left").pack(anchor="w", padx=2, pady=(10, 6))
        row = tk.Frame(f, bg=BG)
        row.pack(fill="both", expand=True)
        lc = card(row, "Your combos")
        lc.pack(side="left", fill="y", padx=(0, 10))
        self.lstCombos = tk.Listbox(lc, width=26, height=8, bg=CARD, fg=TEXT, selectbackground="#1f3a5c", selectforeground=TEXT,
                                    relief="flat", highlightthickness=0, font=(FONT, 11), activestyle="none", exportselection=False)
        self.lstCombos.pack(padx=10, pady=4)
        self.lstCombos.bind("<<ListboxSelect>>", lambda e: self.pick_combo())
        bt = tk.Frame(lc, bg=CARD)
        bt.pack(pady=(0, 10), padx=10, fill="x")
        button(bt, "+ New combo", self.add_combo, bg="#143a1e", fg=OK, pady=3,
               tip="Creates an empty combo you can then fill with steps.").grid(row=0, column=0, columnspan=2, sticky="ew", padx=1)
        button(bt, "Duplicate", self.dup_combo, pady=3, tip="Copies the selected combo so you can tweak a variation.").grid(
            row=1, column=0, sticky="ew", padx=1, pady=2)
        button(bt, "Remove", self.remove_combo, bg="#3a1414", fg=HOT, pady=3,
               tip="Deletes the selected combo. Levels that used it fall back to no combo.").grid(row=1, column=1, sticky="ew", padx=1, pady=2)
        bt.columnconfigure((0, 1), weight=1)

        right = tk.Frame(row, bg=BG)
        right.pack(side="left", fill="both", expand=True)
        hc = card(right, "This combo")
        hc.pack(fill="x")
        hr = tk.Frame(hc, bg=CARD)
        hr.pack(fill="x", padx=12, pady=(0, 4))
        self.v_cname = tk.StringVar()
        label(hr, "Name").pack(side="left")
        entry(hr, self.v_cname, 14).pack(side="left", padx=(4, 14))
        self.lblFunc = label(hr, "", fg=INFO, size=9)
        self.lblFunc.pack(side="left")
        self.v_cname.trace_add("write", lambda *a: self.on_combo_name())
        self.lblUsed = label(hc, "", size=9)
        self.lblUsed.pack(anchor="w", padx=12, pady=(0, 8))
        cc = card(right, "Steps — they play in order")
        cc.pack(fill="both", expand=True, pady=(10, 0))
        self.comboList = StepList(cc, self, "combo", self.on_combo_steps, self.test_combo_step, lambda: self.live)
        self.comboList.pack(fill="both", expand=True, padx=12, pady=(0, 6))
        button(cc, "▶ Test whole combo on robot", self.test_combo, bg="#1c2838", fg=INFO,
               tip="Plays this combo right now without moving the lift. Applies unsent edits first.").pack(anchor="e", padx=12, pady=(0, 10))

    def _tab_levels(self):
        f = tk.Frame(self.nb, bg=BG)
        self.nb.add(f, text="  Scoring Levels  ")
        label(f, "UP / LEFT on the controller step through these levels. The lift drives to the angle, then the level's combo plays.",
              size=9, bg=BG).pack(anchor="w", padx=2, pady=(10, 6))
        row = tk.Frame(f, bg=BG)
        row.pack(fill="both", expand=True)
        lc = card(row, "Levels")
        lc.pack(side="left", fill="y", padx=(0, 10))
        self.lstLevels = tk.Listbox(lc, width=26, height=8, bg=CARD, fg=TEXT, selectbackground="#1f3a5c", selectforeground=TEXT,
                                    relief="flat", highlightthickness=0, font=(FONT, 11), activestyle="none", exportselection=False)
        self.lstLevels.pack(padx=10, pady=4)
        self.lstLevels.bind("<<ListboxSelect>>", lambda e: self.pick_level())
        self.lstLevels.bind("<Double-Button-1>", lambda e: self.send([f"LT GO {self.cur()['angle']:.2f}"]) if self.cur() else None)
        Tooltip(self.lstLevels, "Click a level to edit it. Double-click to drive the lift there. ▶ marks the level the robot is on.")
        bt = tk.Frame(lc, bg=CARD)
        bt.pack(pady=(0, 10), padx=10, fill="x")
        button(bt, "+ Add", self.add_level, bg="#143a1e", fg=OK, pady=3).grid(row=0, column=0, sticky="ew", padx=1)
        button(bt, "Remove", self.remove_level, bg="#3a1414", fg=HOT, pady=3).grid(row=0, column=1, sticky="ew", padx=1)
        button(bt, "▲", lambda: self.move_level(-1), pady=3).grid(row=1, column=0, sticky="ew", padx=1, pady=2)
        button(bt, "▼", lambda: self.move_level(1), pady=3).grid(row=1, column=1, sticky="ew", padx=1, pady=2)
        bt.columnconfigure((0, 1), weight=1)

        right = tk.Frame(row, bg=BG)
        right.pack(side="left", fill="both", expand=True)
        hc = card(right, "This level")
        hc.pack(fill="x")
        hr = tk.Frame(hc, bg=CARD)
        hr.pack(fill="x", padx=12, pady=(0, 10))
        self.v_lname, self.v_langle = tk.StringVar(), tk.StringVar()
        label(hr, "Name").pack(side="left")
        entry(hr, self.v_lname, 11).pack(side="left", padx=(4, 14))
        label(hr, "Lift angle").pack(side="left")
        entry(hr, self.v_langle, 8).pack(side="left", padx=4)
        button(hr, "Use live", lambda: (self.v_langle.set(f"{self.live:.1f}"), self.on_level_edit()), pady=3,
               tip="Copies the lift's current angle into this level.").pack(side="left")
        button(hr, "Go there", lambda: self.send([f"LT GO {num(self.v_langle):.2f}"]), bg="#1c2838", fg=INFO, pady=3,
               tip="Drives the lift to this angle (no combo).").pack(side="left", padx=6)
        button(hr, "▶ Run level (go + combo)", self.run_level, bg="#143a1e", fg=OK, pady=3,
               tip="Exactly what UP / LEFT do in a match: drive to this level, then play its combo. Applies unsent edits first.").pack(side="right")
        for v in (self.v_lname, self.v_langle):
            v.trace_add("write", lambda *a: self.on_level_edit())

        cc = card(right, "Combo that plays when the lift arrives")
        cc.pack(fill="both", expand=True, pady=(10, 0))
        cr = tk.Frame(cc, bg=CARD)
        cr.pack(fill="x", padx=12, pady=(0, 6))
        self.v_lcombo = tk.StringVar()
        self.cbCombo = ttk.Combobox(cr, textvariable=self.v_lcombo, state="readonly", width=22)
        self.cbCombo.pack(side="left")
        self.cbCombo.bind("<<ComboboxSelected>>", lambda e: self.on_level_combo())
        button(cr, "Edit this combo →", self.edit_level_combo, pady=3,
               tip="Jumps to the Combos tab with this combo selected.").pack(side="left", padx=8)
        button(cr, "+ New combo for this level", lambda: self.add_combo(assign=True), bg="#143a1e", fg=OK, pady=3).pack(side="left")
        self.lblSteps = tk.Label(cc, text="", bg=CARD, fg=TEXT, font=(FONT, 10), justify="left", anchor="nw")
        self.lblSteps.pack(fill="both", expand=True, padx=14, pady=(4, 10))

    def _tab_actions(self):
        f = tk.Frame(self.nb, bg=BG)
        self.nb.add(f, text="  Angle Functions  ")
        label(f, "Do something whenever the lift passes an angle — by hand or on its way to a level.", size=9, bg=BG).pack(
            anchor="w", padx=2, pady=(10, 6))
        c = card(f, "Functions (up to 8)")
        c.pack(fill="both", expand=True)
        self.actions = StepList(c, self, "action", self.on_model_edit, self.test_action, lambda: self.live)
        self.actions.pack(fill="both", expand=True, padx=12, pady=(0, 10))

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
        self.update_chips()

    def combo_name(self, idx):
        c = self.model["combos"]
        return c[idx]["name"] if 0 <= idx < len(c) else None

    # -- combos
    def cur_combo(self):
        c = self.model["combos"]
        return c[self.comboIdx] if c and 0 <= self.comboIdx < len(c) else None

    def combo_text(self, i):
        n = len(self.model["combos"][i]["steps"])
        return f"   {i + 1}. {self.model['combos'][i]['name']}  · {n} step{'s' if n != 1 else ''}"

    def refresh_combo_list(self):
        self.lstCombos.delete(0, "end")
        for i in range(len(self.model["combos"])):
            self.lstCombos.insert("end", self.combo_text(i))
        if self.model["combos"]:
            self.lstCombos.selection_set(self.comboIdx)
        self.cbCombo["values"] = ["(no combo)"] + [c["name"] for c in self.model["combos"]]

    def pick_combo(self, refresh=True):
        if refresh:
            sel = self.lstCombos.curselection()
            if sel:
                self.comboIdx = sel[0]
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
            self.lblFunc.config(text="")
            self.lblUsed.config(text="Nothing here yet — press “+ New combo”.")
            return
        names = combo_idents(self.model)
        clash = sum(1 for x in self.model["combos"] if ident(x["name"]) == ident(c["name"])) > 1
        self.lblFunc.config(text=f"saved in code as  {names[self.comboIdx]}()" + ("   (name clash — renamed)" if clash else ""),
                            fg=WARM if clash else INFO)
        used = [lv["name"] for lv in self.model["levels"] if lv["combo"] == self.comboIdx]
        self.lblUsed.config(text="Used by level: " + ", ".join(used) if used else "Not used by any level yet — pick it on the Scoring Levels tab.")

    def on_combo_name(self):
        c = self.cur_combo()
        if self.suspend or not c:
            return
        c["name"] = self.v_cname.get() or "Combo"
        self.refresh_combo_list()
        self.pick_level(refresh=False)
        self.update_combo_info()
        self.mark_dirty()

    def on_combo_steps(self):
        self.refresh_combo_list()
        self.pick_level(refresh=False)
        self.mark_dirty()

    def add_combo(self, assign=False):
        if len(self.model["combos"]) >= 8:
            messagebox.showinfo("Full", "The robot holds at most 8 combos.")
            return
        self.model["combos"].append({"name": f"Combo{len(self.model['combos']) + 1}", "steps": []})
        self.comboIdx = len(self.model["combos"]) - 1
        if assign and self.cur():
            self.cur()["combo"] = self.comboIdx
        self.mark_dirty()
        self.refresh_all()
        if assign:
            self.nb.select(1)

    def dup_combo(self):
        c = self.cur_combo()
        if not c or len(self.model["combos"]) >= 8:
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
            self.nb.select(1)
        else:
            messagebox.showinfo("No combo", "This level has no combo yet. Pick one in the box, or press “+ New combo for this level”.")

    # -- levels
    def level_text(self, i):
        lv = self.model["levels"][i]
        mark = "▶ " if i == self.curLevel else "   "
        cn = self.combo_name(lv["combo"])
        return f"{mark}{i + 1}. {lv['name']}  {lv['angle']:.0f}°" + (f"  · {cn}" if cn else "")

    def refresh_level_list(self):
        self.lstLevels.delete(0, "end")
        for i in range(len(self.model["levels"])):
            self.lstLevels.insert("end", self.level_text(i))
        if self.model["levels"]:
            self.lstLevels.selection_set(self.levelIdx)

    def cur(self):
        return self.model["levels"][self.levelIdx] if self.model["levels"] else None

    def pick_level(self, refresh=True):
        if refresh:
            sel = self.lstLevels.curselection()
            if sel:
                self.levelIdx = sel[0]
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
        self.lblSteps.config(text=("\n".join(f"{i + 1}.  {step_summary(s)}" for i, s in enumerate(steps)) if steps else
                                   ("This combo has no steps yet." if lv and lv["combo"] >= 0 else
                                    "No combo: the lift just drives to this angle and stops.")), fg=TEXT if steps else DIM)

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
        self.update_chips()

    def add_level(self):
        if len(self.model["levels"]) >= 8:
            messagebox.showinfo("Full", "The robot holds at most 8 levels.")
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
                messagebox.showerror("pyserial missing", "Install it with:\n    pip install pyserial")
                return
            port = find_brain_port()
            if not port:
                self.chipConn.config(text="●  no brain found — plug in USB, close pros terminal, press ⟳", fg=HOT)
                return
            try:
                self.link = SerialLink(port)
            except Exception as e:
                self.chipConn.config(text=f"●  {port}: {e}", fg=HOT)
                return
        self.loaded_from_robot = False
        self.update_chips()

    def send(self, lines):
        if not self.link:
            self.chipConn.config(text="●  not connected — press ⟳", fg=HOT)
            return
        self.link.send(lines)

    def stop(self):
        self.send(["LT STOP"])

    def apply(self):
        self.send(model_to_commands(self.model))
        self.save_backup()
        self.dirty = False
        self.update_chips()

    def pull(self):
        if self.dirty and not messagebox.askyesno("Discard edits?", "Replace what's on screen with what the robot has?"):
            return
        self.loaded_from_robot = False
        self.dirty = False

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
            self.update_chips()
            parts.append("robot not connected, so not sent")
        self.chipSaved.config(text=f"✓ Saved {time.strftime('%H:%M:%S')}: " + ", ".join(parts), fg=OK)

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

    def gauge_click(self, e):
        lo, hi = self.scale()
        y0, y1 = 14, GAUGE_H - 14
        a = lo + (y1 - e.y) / (y1 - y0) * (hi - lo)
        self.send([f"LT GO {a:.2f}"])
        self.v_go.set(f"{a:.1f}")

    def gauge_hover(self, e):
        lo, hi = self.scale()
        y0, y1 = 14, GAUGE_H - 14
        a = lo + (y1 - e.y) / (y1 - y0) * (hi - lo)
        self.gauge.delete("hover")
        self.gauge.create_line(60, e.y, 130, e.y, fill=DIM, dash=(2, 3), tags="hover")
        self.gauge.create_text(e.x + 10, e.y - 8, text=f"click → {a:.0f}°", fill=TEXT, anchor="w", font=(FONT, 8, "bold"), tags="hover")

    def poll(self):
        if self.link:
            if getattr(self.link, "tick", None):
                self.link.tick()
            if self.link.dead:
                self.chipConn.config(text="●  connection lost — press ⟳", fg=HOT)
                self.link = None
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
                self.lblLive.config(text=f"{self.live:.1f}°", fg=WARM if self.moving else OK)
                self.lblMoving.config(text="running a combo…" if self.combo_running else ("moving…" if self.moving else "idle"))
                self.draw_gauge()
        elif line.startswith("LTC|") and not self.dirty and not self.loaded_from_robot:
            m = parse_ltc(line)
            if m and m != self.model:
                self.model = m
                self.refresh_all()
            self.loaded_from_robot = True
            self.update_chips()

    def update_chips(self):
        if self.link:
            self.chipConn.config(text=f"●  {'demo robot' if self.demo else 'connected ' + getattr(self.link, 'port', '')}", fg=OK)
        self.chipSync.config(text="● unsent edits — Apply to robot" if self.dirty else "● in sync with robot", fg=WARM if self.dirty else DIM)
        self.btnApply.config(bg="#1f7a35" if self.dirty else "#143a1e", fg="#ffffff" if self.dirty else OK)
        hot = 2 if self.dirty else 0
        for i, l in enumerate(self.stepLabels):
            l.config(fg=TEXT if i == hot else DIM, highlightbackground=OK if (i == hot and self.dirty) else EDGE)

    # ── gauge ──
    def scale(self):
        m = self.model
        vals = [m["limit"], self.live, 0.0] + [l["angle"] for l in m["levels"]] + ([m["start"]] if m["startEnabled"] else [])
        lo, hi = min(vals + [0.0]), max(vals + [10.0])
        span = hi - lo
        return lo - span * 0.02, hi + span * 0.05

    def draw_gauge(self):
        g = self.gauge
        g.delete("all")
        lo, hi = self.scale()
        y0, y1 = 14, GAUGE_H - 14
        yo = lambda a: y1 - (a - lo) / (hi - lo) * (y1 - y0)
        g.create_rectangle(70, y0, 110, y1, fill=TRACK, outline="")
        g.create_rectangle(70, yo(self.live), 110, y1, fill=WARM if self.moving else OK, outline="")
        m = self.model
        g.create_line(60, yo(m["limit"]), 130, yo(m["limit"]), fill=HOT, width=2)
        g.create_text(134, yo(m["limit"]), text="limit", fill=HOT, anchor="w", font=(FONT, 8))
        if m["startEnabled"]:
            g.create_line(54, yo(m["start"]), 130, yo(m["start"]), fill=GOLD, width=2, dash=(4, 2))
            g.create_text(134, yo(m["start"]), text="start", fill=GOLD, anchor="w", font=(FONT, 8))
        for i, lv in enumerate(m["levels"]):
            y = yo(lv["angle"])
            sel = i == self.levelIdx
            g.create_line(60, y, 120, y, fill=INFO, width=3 if sel else 1)
            g.create_text(56, y, text=lv["name"], fill=TEXT if sel else INFO, anchor="e", font=(FONT, 8, "bold" if sel else "normal"))
        for a in m["actions"]:
            y = yo(a["angle"])
            g.create_polygon(112, y, 120, y - 4, 120, y + 4, fill=WARM, outline="")
        g.create_polygon(68, yo(self.live), 58, yo(self.live) - 5, 58, yo(self.live) + 5, fill=TEXT, outline="")
        g.create_text(14, y1 + 2, text=f"{lo:.0f}", fill=DIM, anchor="w", font=(FONT, 8))
        g.create_text(14, y0 - 2, text=f"{hi:.0f}", fill=DIM, anchor="w", font=(FONT, 8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="fake robot, no brain needed")
    args = ap.parse_args()
    App(args.demo).mainloop()


if __name__ == "__main__":
    main()
