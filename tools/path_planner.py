#!/usr/bin/env python3
"""
Path Planner -- the laptop half of the brain's PATH PLANNER screen.

Opened automatically by tools/remote_touch.py when the brain enters that
screen (HOME -> PATH PLANNER). Not meant to be run on its own: it talks
to the robot through remote_touch's link and gets the robot's live pose
from the same RUI| lines the mirror uses.

WHAT'S ON IT
    Top       a big field map. Drop the VEX field render as
              tools/field.png (e.g. from field-rendering.jerryio.com --
              a top-down view, any size, square) and it's used as the
              background; without it you get a plain gridded field. The
              planned route is drawn over it, plus the robot's live pose
              and the path it actually drove.
    Middle    the block list -- one line per movement, in run order --
              with move up / down / duplicate / delete.
    Bottom    movement buttons that append a block (turnToHeading,
              turnToPoint, swingToHeading, swingToPoint, moveToPoint,
              moveToPose, wait, setPose) and, for the selected block,
              every lemlib parameter that movement takes.

    Clicking the map sets the selected block's target point (for blocks
    that have one). "Pick start" then a click sets the robot's start pose
    instead (sent to the robot as chassis.setPose).

RUN / EXPORT
    F5 / Run all       sends the whole list to the brain and runs it there,
                       block after block, exactly like an autonomous would.
    F6 / Run selected  runs just the highlighted block.
    Esc / Stop         cancels (same as controller X).
    Export code        writes the C++ (chassis.* calls) for the list to
                       tools/exported_path.cpp, copies it to the clipboard,
                       and shows it in a window.
    Save / Load        the block list as JSON (tools/plans/*.json).

COORDINATES
    LemLib's: inches, origin at field centre, +x right, +y up (away from
    you), heading 0 = +y and increasing clockwise. Same as the brain.
"""

import json
import math
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD_PNG = os.path.join(HERE, "field.png")
PLANS_DIR = os.path.join(HERE, "plans")
EXPORT_PATH = os.path.join(HERE, "exported_path.cpp")

FIELD_IN = 144.0            # field is 12 ft square
MAP_PX = 560                # map is drawn this many px square

# ─── Block definitions ──────────────────────────────────────────────────────
# Order == StepKind on the brain (PLAN ADD sends the index).
KINDS = ["turnToHeading", "turnToPoint", "swingToHeading", "swingToPoint",
         "moveToPoint", "moveToPose", "wait", "setPose"]

DIRECTIONS = ["AUTO", "CW", "CCW"]
SIDES = ["LEFT", "RIGHT"]

# Every parameter, its default, and how it's edited: "num", "int", "bool",
# or a list of choices. Per kind, the parameters that movement actually
# takes (in lemlib's order), so the panel only shows what applies.
PARAM_SPEC = {
    "x": (0.0, "num"), "y": (0.0, "num"), "theta": (0.0, "num"),
    "timeout": (2000, "int"), "forwards": (True, "bool"),
    "direction": ("AUTO", DIRECTIONS), "side": ("LEFT", SIDES),
    "maxSpeed": (127, "num"), "minSpeed": (0, "num"), "earlyExitRange": (0.0, "num"),
    "lead": (0.6, "num"), "horizontalDrift": (0.0, "num"),
    "ms": (500, "int"),
}
KIND_PARAMS = {
    "turnToHeading": ["theta", "timeout", "direction", "maxSpeed", "minSpeed", "earlyExitRange"],
    "turnToPoint": ["x", "y", "timeout", "forwards", "direction", "maxSpeed", "minSpeed",
                    "earlyExitRange"],
    "swingToHeading": ["theta", "side", "timeout", "direction", "maxSpeed", "minSpeed",
                       "earlyExitRange"],
    "swingToPoint": ["x", "y", "side", "timeout", "forwards", "direction", "maxSpeed",
                     "minSpeed", "earlyExitRange"],
    "moveToPoint": ["x", "y", "timeout", "forwards", "maxSpeed", "minSpeed", "earlyExitRange"],
    "moveToPose": ["x", "y", "theta", "timeout", "forwards", "horizontalDrift", "lead",
                   "maxSpeed", "minSpeed", "earlyExitRange"],
    "wait": ["ms"],
    "setPose": ["x", "y", "theta"],
}
HAS_POINT = {"turnToPoint", "swingToPoint", "moveToPoint", "moveToPose", "setPose"}
HAS_HEADING = {"turnToHeading", "swingToHeading", "moveToPose", "setPose"}


def new_block(kind, **overrides):
    b = {"kind": kind}
    for name in KIND_PARAMS[kind]:
        b[name] = PARAM_SPEC[name][0]
    b.update(overrides)
    return b


def fmt(v):
    return f"{v:.2f}" if isinstance(v, float) else str(v)


def block_code(b):
    """The C++ line(s) this block becomes -- same shape as the brain's
    formatStepCode(), so the export matches the on-screen preview."""
    k = b["kind"]
    if k == "wait":
        return f"pros::delay({int(b['ms'])});"
    if k == "setPose":
        return f"chassis.setPose({b['x']:.2f}, {b['y']:.2f}, {b['theta']:.2f});"
    fwd = "true" if b.get("forwards", True) else "false"
    opts = []
    if k in ("turnToPoint", "swingToPoint", "moveToPoint", "moveToPose"):
        opts.append(f".forwards = {fwd}")
    if k in ("turnToHeading", "turnToPoint", "swingToHeading", "swingToPoint"):
        opts.append(f".direction = lemlib::AngularDirection::{DIR_ENUM[b['direction']]}")
    if k == "moveToPose":
        opts.append(f".horizontalDrift = {b['horizontalDrift']:.2f}")
        opts.append(f".lead = {b['lead']:.2f}")
    opts.append(f".maxSpeed = {b['maxSpeed']:.0f}")
    opts.append(f".minSpeed = {b['minSpeed']:.0f}")
    opts.append(f".earlyExitRange = {b['earlyExitRange']:.2f}")
    o = "{" + ", ".join(opts) + "}"
    t = int(b["timeout"])
    side = f"lemlib::DriveSide::{b.get('side', 'LEFT')}"
    if k == "turnToHeading":
        return f"chassis.turnToHeading({b['theta']:.2f}, {t}, {o});"
    if k == "turnToPoint":
        return f"chassis.turnToPoint({b['x']:.2f}, {b['y']:.2f}, {t}, {o});"
    if k == "swingToHeading":
        return f"chassis.swingToHeading({b['theta']:.2f}, {side}, {t}, {o});"
    if k == "swingToPoint":
        return f"chassis.swingToPoint({b['x']:.2f}, {b['y']:.2f}, {side}, {t}, {o});"
    if k == "moveToPoint":
        return f"chassis.moveToPoint({b['x']:.2f}, {b['y']:.2f}, {t}, {o});"
    if k == "moveToPose":
        return f"chassis.moveToPose({b['x']:.2f}, {b['y']:.2f}, {b['theta']:.2f}, {t}, {o});"
    return f"// unknown block {k}"


DIR_ENUM = {"AUTO": "AUTO", "CW": "CW_CLOCKWISE", "CCW": "CCW_COUNTERCLOCKWISE"}


def block_summary(b):
    k = b["kind"]
    if k == "wait":
        return f"wait {int(b['ms'])} ms"
    if k == "setPose":
        return f"setPose ({b['x']:.1f}, {b['y']:.1f}) hdg {b['theta']:.1f}"
    parts = []
    if k in HAS_POINT:
        parts.append(f"({b['x']:.1f}, {b['y']:.1f})")
    if k in HAS_HEADING:
        parts.append(f"hdg {b['theta']:.1f}")
    if "side" in b:
        parts.append(b["side"])
    if "forwards" in b and not b["forwards"]:
        parts.append("BWD")
    parts.append(f"{int(b['timeout'])}ms")
    return f"{k} " + " ".join(parts)


def block_to_plan_add(b):
    """'PLAN ADD k x y th t fwd dir side max min exit lead drift' for the brain."""
    k = KINDS.index(b["kind"])
    t = int(b["ms"]) if b["kind"] == "wait" else int(b.get("timeout", 0))
    return ("PLAN ADD {} {:.2f} {:.2f} {:.2f} {} {} {} {} {:.1f} {:.1f} {:.2f} {:.2f} {:.2f}"
            .format(k, b.get("x", 0.0), b.get("y", 0.0), b.get("theta", 0.0), t,
                    1 if b.get("forwards", True) else 0,
                    DIRECTIONS.index(b.get("direction", "AUTO")),
                    SIDES.index(b.get("side", "LEFT")),
                    b.get("maxSpeed", 127), b.get("minSpeed", 0), b.get("earlyExitRange", 0.0),
                    b.get("lead", 0.6), b.get("horizontalDrift", 0.0)))


def export_code(blocks):
    lines = ["// Generated by tools/path_planner.py -- paste into an autonomous routine.",
             "// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +."]
    for i, b in enumerate(blocks):
        lines.append(f"{block_code(b)}  // {i + 1}: {block_summary(b)}")
    return "\n".join(lines) + "\n"


# ─── Simulated route (for drawing the plan) ────────────────────────────────
def simulate(blocks, start):
    """Walks the block list from `start` = (x, y, theta) and returns, per
    block, the pose it ends at -- enough to draw the route the way the
    robot will (roughly) take it. Turns/swings only change heading."""
    x, y, th = start
    out = []
    for b in blocks:
        k = b["kind"]
        if k == "setPose":
            x, y, th = b["x"], b["y"], b["theta"]
        elif k in ("moveToPoint", "moveToPose"):
            x, y = b["x"], b["y"]
            th = b["theta"] if k == "moveToPose" else th
        elif k in ("turnToHeading", "swingToHeading"):
            th = b["theta"]
        elif k in ("turnToPoint", "swingToPoint"):
            dx, dy = b["x"] - x, b["y"] - y
            if dx or dy:
                th = math.degrees(math.atan2(dx, dy))
                if not b.get("forwards", True):
                    th += 180
        out.append((x, y, th % 360))
    return out


# ─── The window ──────────────────────────────────────────────────────────────
class PlannerWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.title("Path Planner -- V5")
        self.blocks = []
        self.selected = None
        self.start = [0.0, 0.0, 0.0]   # start pose used to draw the route
        self.pickStart = False
        self.robotPose = None
        self.robotPath = []
        self.planStatus = ("IDLE", -1, 0)
        self.onBrainScreen = False
        self.fieldImage = None
        self.protocol("WM_DELETE_WINDOW", self.withdraw)  # hide, don't destroy

        self._buildUi()
        self._bindKeys()
        self.redrawMap()
        self.refreshList()

    # -- layout --
    def _buildUi(self):
        self.configure(bg="#14141a")
        top = tk.Frame(self, bg="#14141a")
        top.pack(side="top", fill="both", expand=True)

        # Map (left) -----------------------------------------------------
        mapFrame = tk.Frame(top, bg="#14141a")
        mapFrame.pack(side="left", padx=8, pady=8)
        self.map = tk.Canvas(mapFrame, width=MAP_PX, height=MAP_PX, bg="#1b1b22",
                             highlightthickness=1, highlightbackground="#2d2d38")
        self.map.pack()
        self.map.bind("<Button-1>", self.onMapClick)
        self.map.bind("<Motion>", self.onMapMotion)
        self.mapStatus = tk.Label(mapFrame, text="", anchor="w", fg="#999", bg="#14141a",
                                  font=("Consolas", 9))
        self.mapStatus.pack(fill="x")
        self._loadFieldImage()

        # Right column: blocks + parameters --------------------------------
        right = tk.Frame(top, bg="#14141a")
        right.pack(side="left", fill="both", expand=True, padx=(0, 8), pady=8)

        hdr = tk.Frame(right, bg="#14141a")
        hdr.pack(fill="x")
        self.brainStatus = tk.Label(hdr, text="brain: waiting...", anchor="w", fg="#00F0FF",
                                    bg="#14141a", font=("Segoe UI", 10, "bold"))
        self.brainStatus.pack(side="left")

        # start pose row
        sp = tk.Frame(right, bg="#14141a")
        sp.pack(fill="x", pady=(6, 2))
        tk.Label(sp, text="start pose", fg="#ccc", bg="#14141a").pack(side="left")
        self.startVars = []
        for i, lab in enumerate(("x", "y", "hdg")):
            tk.Label(sp, text=lab, fg="#888", bg="#14141a").pack(side="left", padx=(8, 2))
            v = tk.StringVar(value="0")
            e = tk.Entry(sp, textvariable=v, width=7, font=("Consolas", 10))
            e.pack(side="left")
            e.bind("<Return>", lambda _e: self.applyStartPose())
            self.startVars.append(v)
        tk.Button(sp, text="Send to robot", command=self.sendStartPose).pack(side="left", padx=6)
        self.pickBtn = tk.Button(sp, text="Pick start on map", command=self.togglePickStart)
        self.pickBtn.pack(side="left")
        tk.Button(sp, text="Use robot's pose", command=self.useRobotPose).pack(side="left", padx=6)

        # block list
        lf = tk.Frame(right, bg="#14141a")
        lf.pack(fill="both", expand=True, pady=(6, 0))
        self.listbox = tk.Listbox(lf, font=("Consolas", 10), bg="#161B22", fg="#ddd",
                                  selectbackground="#1c2838", selectforeground="#00F0FF",
                                  activestyle="none", height=12, exportselection=False)
        self.listbox.pack(side="left", fill="both", expand=True)
        self.listbox.bind("<<ListboxSelect>>", self.onSelect)
        sb = tk.Scrollbar(lf, command=self.listbox.yview)
        sb.pack(side="left", fill="y")
        self.listbox.config(yscrollcommand=sb.set)
        ops = tk.Frame(lf, bg="#14141a")
        ops.pack(side="left", fill="y", padx=(6, 0))
        for text, cmd in (("Up", lambda: self.moveBlock(-1)), ("Down", lambda: self.moveBlock(1)),
                          ("Duplicate", self.duplicateBlock), ("Delete", self.deleteBlock),
                          ("Clear all", self.clearBlocks)):
            tk.Button(ops, text=text, width=10, command=cmd).pack(pady=2)

        # parameters panel
        self.paramFrame = tk.LabelFrame(right, text="parameters", fg="#ccc", bg="#14141a",
                                        font=("Segoe UI", 9))
        self.paramFrame.pack(fill="x", pady=6)
        self.paramWidgets = {}
        self.codeLabel = tk.Label(right, text="", anchor="w", justify="left", fg="#2ECC71",
                                  bg="#14141a", font=("Consolas", 9), wraplength=520)
        self.codeLabel.pack(fill="x")

        # movement buttons
        mv = tk.LabelFrame(right, text="add block", fg="#ccc", bg="#14141a", font=("Segoe UI", 9))
        mv.pack(fill="x", pady=(2, 6))
        for i, kind in enumerate(KINDS):
            tk.Button(mv, text=kind, width=14, command=lambda k=kind: self.addBlock(k)) \
                .grid(row=i // 4, column=i % 4, padx=3, pady=3)

        # run / export bar
        bar = tk.Frame(right, bg="#14141a")
        bar.pack(fill="x")
        for text, cmd, color in (("Run all  (F5)", self.runAll, "#2ECC71"),
                                 ("Run selected  (F6)", self.runSelected, "#00F0FF"),
                                 ("Stop  (Esc)", self.stop, "#FF6B81")):
            tk.Button(bar, text=text, command=cmd, fg="black", bg=color).pack(side="left", padx=3)
        tk.Button(bar, text="Export code", command=self.exportCode).pack(side="left", padx=(16, 3))
        tk.Button(bar, text="Save", command=self.savePlan).pack(side="left", padx=3)
        tk.Button(bar, text="Load", command=self.loadPlan).pack(side="left", padx=3)

        self.hint = tk.Label(right, text="Click the map to set the selected block's point.  "
                             "Delete = remove block.  Ctrl+S / Ctrl+O save / load.  Ctrl+E export.",
                             anchor="w", fg="#777", bg="#14141a", font=("Segoe UI", 8))
        self.hint.pack(fill="x", pady=(6, 0))

    def _bindKeys(self):
        self.bind("<F5>", lambda e: self.runAll())
        self.bind("<F6>", lambda e: self.runSelected())
        self.bind("<Escape>", lambda e: self.stop())
        self.bind("<Control-s>", lambda e: self.savePlan())
        self.bind("<Control-o>", lambda e: self.loadPlan())
        self.bind("<Control-e>", lambda e: self.exportCode())
        self.listbox.bind("<Delete>", lambda e: self.deleteBlock())

    def _loadFieldImage(self):
        """tools/field.png scaled to the map; PIL if present (any size),
        else Tk's own loader (integer downscale only)."""
        if not os.path.exists(FIELD_PNG):
            return
        try:
            from PIL import Image, ImageTk
            img = Image.open(FIELD_PNG).convert("RGB").resize((MAP_PX, MAP_PX), Image.LANCZOS)
            self.fieldImage = ImageTk.PhotoImage(img)
        except ImportError:
            try:
                img = tk.PhotoImage(file=FIELD_PNG)
                f = max(1, round(img.width() / MAP_PX))
                self.fieldImage = img.subsample(f, f)
            except tk.TclError:
                self.fieldImage = None
        except Exception:
            self.fieldImage = None

    # -- coordinates --
    def toMap(self, x, y):
        ppi = MAP_PX / FIELD_IN
        return MAP_PX / 2 + x * ppi, MAP_PX / 2 - y * ppi

    def toField(self, px, py):
        ppi = MAP_PX / FIELD_IN
        return (px - MAP_PX / 2) / ppi, (MAP_PX / 2 - py) / ppi

    # -- map drawing --
    def redrawMap(self):
        c = self.map
        c.delete("all")
        if self.fieldImage:
            c.create_image(0, 0, anchor="nw", image=self.fieldImage)
        else:
            c.create_rectangle(0, 0, MAP_PX, MAP_PX, fill="#1b1b22", outline="")
        # tile grid (24 in) either way -- faint so it reads over the render
        for v in range(-72, 73, 24):
            px, _ = self.toMap(v, 0)
            _, py = self.toMap(0, v)
            col = "#66666a" if v == 0 else "#3a3a44"
            c.create_line(px, 0, px, MAP_PX, fill=col, dash=(2, 4))
            c.create_line(0, py, MAP_PX, py, fill=col, dash=(2, 4))

        # planned route
        poses = simulate(self.blocks, tuple(self.start))
        prev = tuple(self.start)
        self.drawRobotMarker(prev, "#FF8C00", "S")
        for i, (b, pose) in enumerate(zip(self.blocks, poses)):
            sel = (i == self.selected)
            col = "#00F0FF" if sel else "#FF6B81"
            w = 3 if sel else 2
            if b["kind"] in ("moveToPoint", "moveToPose"):
                x0, y0 = self.toMap(prev[0], prev[1])
                x1, y1 = self.toMap(pose[0], pose[1])
                c.create_line(x0, y0, x1, y1, fill=col, width=w, arrow="last",
                              dash=(6, 3) if not b.get("forwards", True) else None)
            if b["kind"] in ("turnToPoint", "swingToPoint"):
                x0, y0 = self.toMap(pose[0], pose[1])
                x1, y1 = self.toMap(b["x"], b["y"])
                c.create_line(x0, y0, x1, y1, fill=col, width=1, dash=(3, 3))
                c.create_oval(x1 - 4, y1 - 4, x1 + 4, y1 + 4, outline=col)
            self.drawRobotMarker(pose, col, str(i + 1), small=True)
            prev = pose

        # robot: driven path then live pose
        if len(self.robotPath) > 1:
            pts = [self.toMap(x, y) for x, y in self.robotPath]
            c.create_line(*[v for p in pts for v in p], fill="#2EFF8C", width=2)
        if self.robotPose:
            self.drawRobotMarker(self.robotPose, "#2EFF8C", "", robot=True)

    def drawRobotMarker(self, pose, color, label, small=False, robot=False):
        x, y, th = pose
        px, py = self.toMap(x, y)
        r = 9 if not small else 6
        rad = math.radians(th)
        tx, ty = px + math.sin(rad) * (r + 10), py - math.cos(rad) * (r + 10)
        if robot:
            # robot outline (18 in square) rotated to heading
            half = 9 * MAP_PX / FIELD_IN
            corners = []
            for cx, cy in ((-half, -half), (half, -half), (half, half), (-half, half)):
                rx = cx * math.cos(rad) - cy * math.sin(rad)
                ry = cx * math.sin(rad) + cy * math.cos(rad)
                corners += [px + rx, py + ry]
            self.map.create_polygon(*corners, outline=color, fill="", width=2)
        self.map.create_oval(px - r, py - r, px + r, py + r, outline=color, width=2,
                             fill="#14141a" if not robot else "")
        self.map.create_line(px, py, tx, ty, fill=color, width=2, arrow="last")
        if label:
            self.map.create_text(px, py, text=label, fill=color, font=("Segoe UI", 8, "bold"))

    def onMapMotion(self, e):
        x, y = self.toField(e.x, e.y)
        self.mapStatus.config(text=f"({x:6.1f}, {y:6.1f}) in" +
                              ("   -- click to set START pose" if self.pickStart else ""))

    def onMapClick(self, e):
        x, y = self.toField(e.x, e.y)
        x, y = round(x, 1), round(y, 1)
        if self.pickStart:
            self.start[0], self.start[1] = x, y
            self.startVars[0].set(fmt(x))
            self.startVars[1].set(fmt(y))
            self.togglePickStart()
            self.redrawMap()
            return
        if self.selected is None or self.blocks[self.selected]["kind"] not in HAS_POINT:
            return
        b = self.blocks[self.selected]
        b["x"], b["y"] = x, y
        self.showParams()
        self.refreshList()

    # -- start pose --
    def togglePickStart(self):
        self.pickStart = not self.pickStart
        self.pickBtn.config(relief="sunken" if self.pickStart else "raised")

    def applyStartPose(self):
        try:
            self.start = [float(v.get()) for v in self.startVars]
        except ValueError:
            return
        self.redrawMap()

    def sendStartPose(self):
        self.applyStartPose()
        self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")

    def useRobotPose(self):
        if self.robotPose:
            self.start = [round(v, 1) for v in self.robotPose]
            for v, val in zip(self.startVars, self.start):
                v.set(fmt(val))
            self.redrawMap()

    # -- block list --
    def refreshList(self):
        self.listbox.delete(0, "end")
        for i, b in enumerate(self.blocks):
            self.listbox.insert("end", f"{i + 1:2d}. {block_summary(b)}")
        if self.selected is not None and self.selected < len(self.blocks):
            self.listbox.selection_set(self.selected)
            self.listbox.see(self.selected)
        self.redrawMap()

    def onSelect(self, _e=None):
        sel = self.listbox.curselection()
        self.selected = sel[0] if sel else None
        self.showParams()
        self.redrawMap()

    def addBlock(self, kind):
        # New blocks start where the route currently ends, so the map shows
        # something sensible before you've typed anything.
        poses = simulate(self.blocks, tuple(self.start))
        end = poses[-1] if poses else tuple(self.start)
        b = new_block(kind)
        if kind in HAS_POINT:
            b["x"], b["y"] = round(end[0], 1), round(end[1], 1)
        if kind in HAS_HEADING:
            b["theta"] = round(end[2], 1)
        at = len(self.blocks) if self.selected is None else self.selected + 1
        self.blocks.insert(at, b)
        self.selected = at
        self.refreshList()
        self.showParams()

    def moveBlock(self, delta):
        i = self.selected
        if i is None:
            return
        j = i + delta
        if 0 <= j < len(self.blocks):
            self.blocks[i], self.blocks[j] = self.blocks[j], self.blocks[i]
            self.selected = j
            self.refreshList()

    def duplicateBlock(self):
        if self.selected is None:
            return
        self.blocks.insert(self.selected + 1, dict(self.blocks[self.selected]))
        self.selected += 1
        self.refreshList()

    def deleteBlock(self):
        if self.selected is None:
            return
        del self.blocks[self.selected]
        self.selected = min(self.selected, len(self.blocks) - 1) if self.blocks else None
        self.refreshList()
        self.showParams()

    def clearBlocks(self):
        if self.blocks and not messagebox.askyesno("Clear", "Remove every block?", parent=self):
            return
        self.blocks = []
        self.selected = None
        self.refreshList()
        self.showParams()

    # -- parameter panel --
    def showParams(self):
        for w in self.paramFrame.winfo_children():
            w.destroy()
        self.paramWidgets = {}
        if self.selected is None:
            tk.Label(self.paramFrame, text="select a block (or add one)", fg="#777",
                     bg="#14141a").grid(row=0, column=0, padx=6, pady=4)
            self.codeLabel.config(text="")
            return
        b = self.blocks[self.selected]
        self.paramFrame.config(text=f"{self.selected + 1}. {b['kind']}")
        col = 0
        for name in KIND_PARAMS[b["kind"]]:
            default, kind = PARAM_SPEC[name]
            tk.Label(self.paramFrame, text=name, fg="#aaa", bg="#14141a",
                     font=("Segoe UI", 8)).grid(row=(col // 5) * 2, column=col % 5, sticky="w",
                                                padx=6)
            if kind == "bool":
                var = tk.BooleanVar(value=bool(b[name]))
                w = tk.Checkbutton(self.paramFrame, variable=var, bg="#14141a",
                                   text="forwards" if var.get() else "backwards",
                                   fg="#ddd", selectcolor="#161B22",
                                   command=lambda n=name: self.paramChanged(n))
            elif isinstance(kind, list):
                var = tk.StringVar(value=str(b[name]))
                w = ttk.Combobox(self.paramFrame, textvariable=var, values=kind, width=7,
                                 state="readonly")
                w.bind("<<ComboboxSelected>>", lambda _e, n=name: self.paramChanged(n))
            else:
                var = tk.StringVar(value=fmt(b[name]))
                w = tk.Entry(self.paramFrame, textvariable=var, width=9, font=("Consolas", 10))
                w.bind("<Return>", lambda _e, n=name: self.paramChanged(n))
                w.bind("<FocusOut>", lambda _e, n=name: self.paramChanged(n))
            w.grid(row=(col // 5) * 2 + 1, column=col % 5, sticky="w", padx=6, pady=(0, 4))
            self.paramWidgets[name] = var
            col += 1
        self.codeLabel.config(text=block_code(b))

    def paramChanged(self, name):
        if self.selected is None:
            return
        b = self.blocks[self.selected]
        var = self.paramWidgets[name]
        _, kind = PARAM_SPEC[name]
        try:
            if kind == "bool":
                b[name] = bool(var.get())
            elif isinstance(kind, list):
                b[name] = var.get()
            elif kind == "int":
                b[name] = int(float(var.get()))
            else:
                b[name] = float(var.get())
        except (ValueError, tk.TclError):
            return
        self.codeLabel.config(text=block_code(b))
        # forwards checkbox text follows its value
        if kind == "bool":
            for w in self.paramFrame.winfo_children():
                if isinstance(w, tk.Checkbutton):
                    w.config(text="forwards" if b[name] else "backwards")
        self.refreshList()

    # -- robot --
    def uploadPlan(self):
        self.link._send("PLAN CLEAR")
        for b in self.blocks:
            self.link._send(block_to_plan_add(b))

    def runAll(self):
        if not self.blocks:
            return
        self.uploadPlan()
        self.link._send("PLAN RUN")

    def runSelected(self):
        if self.selected is None:
            return
        self.uploadPlan()
        self.link._send(f"PLAN RUN {self.selected}")

    def stop(self):
        self.link._send("PLAN STOP")

    def applyState(self, state):
        """Called by remote_touch with every parsed RUI state."""
        self.onBrainScreen = state["screen"] == "PLANNER"
        plot = state.get("plot")
        if plot:
            self.robotPose = (plot["poseX"], plot["poseY"], plot["poseTheta"])
        if state.get("path"):
            self.robotPath = state["path"]
        status, idx, count = state.get("plan") or ("IDLE", -1, 0)
        self.planStatus = (status, idx, count)
        if self.onBrainScreen:
            txt = f"brain: PLANNER  {status}"
            if idx >= 0:
                txt += f"  step {idx + 1}/{count}"
            if status == "RUNNING" and 0 <= idx < len(self.blocks):
                self.listbox.selection_clear(0, "end")
                self.listbox.selection_set(idx)
                self.listbox.see(idx)
            self.brainStatus.config(text=txt, fg={"RUNNING": "#2ECC71", "CANCELLED": "#FF6B81"}
                                    .get(status, "#00F0FF"))
        else:
            self.brainStatus.config(text=f"brain: on {state['screen']} -- tap PATH PLANNER on the "
                                    "brain (or mirror) to run from here", fg="#FF8C00")
        self.redrawMap()

    # -- files --
    def exportCode(self):
        code = export_code(self.blocks)
        with open(EXPORT_PATH, "w", encoding="utf-8") as f:
            f.write(code)
        self.clipboard_clear()
        self.clipboard_append(code)
        win = tk.Toplevel(self)
        win.title("Exported code")
        txt = tk.Text(win, width=100, height=min(30, len(self.blocks) + 4), font=("Consolas", 10),
                      bg="#161B22", fg="#2ECC71")
        txt.pack(fill="both", expand=True)
        txt.insert("1.0", code)
        tk.Label(win, text=f"saved to {EXPORT_PATH} and copied to the clipboard",
                 anchor="w").pack(fill="x")

    def savePlan(self):
        os.makedirs(PLANS_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(parent=self, initialdir=PLANS_DIR,
                                            defaultextension=".json",
                                            filetypes=[("plan", "*.json")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"start": self.start, "blocks": self.blocks}, f, indent=2)

    def loadPlan(self):
        os.makedirs(PLANS_DIR, exist_ok=True)
        path = filedialog.askopenfilename(parent=self, initialdir=PLANS_DIR,
                                          filetypes=[("plan", "*.json")])
        if not path:
            return
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        self.start = list(data.get("start", [0, 0, 0]))
        for v, val in zip(self.startVars, self.start):
            v.set(fmt(val))
        self.blocks = [new_block(b["kind"], **{k: v for k, v in b.items() if k != "kind"})
                       for b in data.get("blocks", []) if b.get("kind") in KINDS]
        self.selected = 0 if self.blocks else None
        self.refreshList()
        self.showParams()
