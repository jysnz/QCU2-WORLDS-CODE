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
              moveToPose, wait, setPose, waitUntilDone) and, for the
              selected block, every lemlib parameter that movement takes.
              Adding a movement also adds a waitUntilDone after it, since
              lemlib motions are asynchronous -- without it the exported
              code would start the next movement immediately.

    Each block's movement is drawn on the map from wherever the previous
    block left the robot, by *simulating* lemlib's motion controllers on
    a model of this robot (track width, wheel size, rpm and the PID /
    exit settings are read from src/main.cpp): the curve a moveToPoint
    really takes as it turns while driving, moveToPose's boomerang
    approach, swings pivoting around the locked side (which moves the
    robot), backwards approaches, min/max speed and early-exit effects.
    So the stack of blocks reads as the route the robot will drive.

    Clicking the map sets the selected block's target point (for blocks
    that have one), and any block's point can be dragged around the map
    directly. "Pick start" then a click sets the robot's start pose
    instead (sent to the robot as chassis.setPose). "Reset robot" puts
    the robot's pose back to the start pose and clears the driven path /
    trail and the last run's status, on the map and on the brain.

DRAWING A PATH WITH THE MOUSE (path.jerryio style)
    "Draw path" (D) turns the map into a path editor, and which mouse
    button you use says what kind of point you're dropping:

        left click    a CURVE point -- the route bends smoothly through
                      it (a Catmull-Rom spline, the curve path.jerryio
                      draws through its control points), resampled every
                      "spacing" inches.
        right click   a STRAIGHT point -- the legs either side of it are
                      dead straight, and the robot squares up and stops
                      on it before carrying on.

    So a run of left-clicks is one flowing curve, and a right-click puts
    a hard corner in the middle of it. Right-clicking a point you've
    already dropped flips it between the two, so a curve can be pinned
    straight (or a corner rounded off) after the fact.

    Dragging a point moves it, shift-click or middle-click removes one,
    Ctrl+Z / Backspace undoes the last.

    Untick "smooth" for a turn-and-drive route instead: every leg is
    straight, and wherever the route turns left or right the robot is
    aimed down the next leg with a turnToHeading first, so each
    moveToPoint drives a straight line rather than steering itself onto
    the target as it goes. Legs that carry straight on don't get a turn.

    "Apply" turns the drawn route into ordinary blocks: the first point
    becomes the start pose (facing the second point) and every point
    after it becomes a moveToPoint. Points along a curve get a minSpeed
    and an earlyExitRange of half the spacing, so the robot flows
    through them instead of stopping at each one; straight points (and
    the end of the route) get neither, so it arrives on them properly.
    From there they're just blocks -- editable, runnable, exportable --
    and "Trace blocks" goes the other way, loading the existing blocks'
    points back in as control points to re-draw.

RUN / EXPORT
    F5 / Run all       sends the whole list to the brain and runs it there,
                       block after block, exactly like an autonomous would.
    F6 / Run selected  runs just the highlighted block.
    Esc / Stop         cancels (same as controller X).
    Export code        writes the C++ for the list to tools/exported_path.cpp,
                       copies it to the clipboard, and shows it in a window.
                       The export opens with the robot's own configuration
                       (drivetrain + both PID controllers, read from
                       src/main.cpp) as a record of what the route was
                       planned against, then chassis.calibrate(), then the
                       start pose, then the movements.
    Save / Load        the block list as JSON (tools/plans/*.json).

COORDINATES
    LemLib's: inches, origin at field centre, +x right, +y up (away from
    you), heading 0 = +y and increasing clockwise. Same as the brain.
"""

import json
import math
import os
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD_PNG = os.path.join(HERE, "field.png")
PLANS_DIR = os.path.join(HERE, "plans")
EXPORT_PATH = os.path.join(HERE, "exported_path.cpp")

FIELD_IN = 144.0            # field is 12 ft square
MAP_PX = 500                # map is drawn this many px square
# tools/field.png must be MAP_PX square: without Pillow, Tk can only scale
# an image by whole numbers, so anything else would land skewed on the grid.

# ─── Block definitions ──────────────────────────────────────────────────────
# Order == StepKind on the brain (PLAN ADD sends the index).
KINDS = ["turnToHeading", "turnToPoint", "swingToHeading", "swingToPoint",
         "moveToPoint", "moveToPose", "wait", "setPose", "waitUntilDone"]
MOTIONS = set(KINDS[:6])   # the ones that actually drive (async on the robot)

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
    "waitUntilDone": [],
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
    if k == "waitUntilDone":
        return "chassis.waitUntilDone();"
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
    if k == "waitUntilDone":
        return "waitUntilDone"
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


def catmull_rom(points, samples_per_leg=24):
    """Smooth curve through every point (Catmull-Rom, the curve
    path.jerryio draws through its control points). Returns a dense
    polyline; endpoints are duplicated so the curve starts and ends
    exactly on the first and last control point."""
    if len(points) < 3:
        return [tuple(p) for p in points]
    pts = [points[0]] + [tuple(p) for p in points] + [points[-1]]
    out = []
    for i in range(len(pts) - 3):
        (x0, y0), (x1, y1), (x2, y2), (x3, y3) = pts[i:i + 4]
        n = max(6, min(60, samples_per_leg))
        for s in range(n):
            t = s / n
            t2, t3 = t * t, t * t * t
            out.append((
                0.5 * ((2 * x1) + (-x0 + x2) * t + (2 * x0 - 5 * x1 + 4 * x2 - x3) * t2
                       + (-x0 + 3 * x1 - 3 * x2 + x3) * t3),
                0.5 * ((2 * y1) + (-y0 + y2) * t + (2 * y0 - 5 * y1 + 4 * y2 - y3) * t2
                       + (-y0 + 3 * y1 - 3 * y2 + y3) * t3)))
    out.append(tuple(points[-1]))
    return out


CURVE, STRAIGHT = "curve", "straight"


def path_points(waypoints, spacing, smooth=True):
    """The route through the drawn control points, as (x, y, stop) --
    stop marking the points the robot should actually arrive on rather
    than flow through.

    A point is either CURVE or STRAIGHT (left / right click on the map).
    Runs of neighbouring curve points are splined together; any leg that
    touches a straight point stays a straight line, and the robot stops
    where a curve meets a straight leg, because the two don't share a
    tangent there."""
    pts = [(w[0], w[1]) for w in waypoints]
    modes = [w[2] if len(w) > 2 else CURVE for w in waypoints]
    if len(pts) < 2:
        return [(x, y, True) for x, y in pts]
    if not smooth:
        modes = [STRAIGHT] * len(pts)
    out = [(pts[0][0], pts[0][1], True)]
    i, n = 0, len(pts)
    while i < n - 1:
        if modes[i] == CURVE and modes[i + 1] == CURVE:
            j = i + 1                       # end of this run of curve legs
            while j < n - 1 and modes[j + 1] == CURVE:
                j += 1
            curve = resample(catmull_rom(pts[i:j + 1]), spacing)
            for x, y in curve[1:-1]:
                out.append((x, y, False))
            # the run's last point: a corner unless the route ends there
            out.append((pts[j][0], pts[j][1], True))
            i = j
        else:
            out.append((pts[i + 1][0], pts[i + 1][1], True))
            i += 1
    return out


def resample(poly, spacing):
    """Points every `spacing` inches along a polyline, last one kept."""
    if len(poly) < 2:
        return [tuple(p) for p in poly]
    spacing = max(1.0, spacing)
    out = [tuple(poly[0])]
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        d = spacing - carry
        while d <= seg:
            t = d / seg
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
            d += spacing
        carry = (carry + seg) % spacing
    if math.hypot(out[-1][0] - poly[-1][0], out[-1][1] - poly[-1][1]) > 0.25:
        out.append(tuple(poly[-1]))
    elif len(out) > 1:
        out[-1] = tuple(poly[-1])
    return out


PID_FIELDS = ("kP", "kI", "kD", "windup", "smallErr", "smallTime", "largeErr", "largeTime",
              "slew")


def config_lines():
    """The robot's lemlib setup, quoted from src/main.cpp -- the drivetrain
    and the two PID controllers the route was planned against, so a path
    read back later says what tuning it was made for. Falls back to
    RobotSpec's numbers if main.cpp isn't readable."""
    out = []
    path = os.path.join(HERE, "..", "src", "main.cpp")
    try:
        src = open(path, encoding="utf-8").read()
    except OSError:
        src = ""
    import re
    for pattern in (r"^\s*lemlib::Drivetrain\s+.*$", r"^\s*lemlib::OdomSensors\s+.*$",
                    r"^\s*lemlib::ControllerSettings\s+.*$", r"^\s*lemlib::Chassis\s+.*$"):
        out += [m.group(0).strip() for m in re.finditer(pattern, src, re.M)]
    if not out:      # no main.cpp -- write what the preview is actually using
        out = [f"lemlib::Drivetrain drivetrain(&left, &right, {SPEC.trackWidth:g}, "
               f"lemlib::Omniwheel::NEW_{str(SPEC.wheelDiameter).replace('.', '')}, "
               f"{SPEC.rpm:g}, 2);"]
        for name, gains in (("lateral_controller", SPEC.lateral),
                            ("angular_controller", SPEC.angular)):
            vals = ", ".join(f"{gains[k]:g}" for k in PID_FIELDS)
            out.append(f"lemlib::ControllerSettings {name}({vals});")
    return out


def pid_summary():
    """Both controllers' gains spelled out, so the numbers in the brackets
    above don't have to be counted off by hand."""
    out = []
    for label, gains in (("lateral (drive)", SPEC.lateral), ("angular (turn)", SPEC.angular)):
        out.append(f"{label:16} kP {gains['kP']:g}  kI {gains['kI']:g}  kD {gains['kD']:g}  "
                   f"windup {gains['windup']:g}")
        out.append(f"{'':16} exit: small {gains['smallErr']:g} for {gains['smallTime']:.0f} ms, "
                   f"large {gains['largeErr']:g} for {gains['largeTime']:.0f} ms, "
                   f"slew {gains['slew']:g}")
    return out


def export_code(blocks, start=None, calibrate=True):
    lines = ["// Generated by tools/path_planner.py -- paste into an autonomous routine.",
             "// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.",
             "//",
             "// Planned against this robot configuration (src/main.cpp):"]
    lines += [f"//   {line}" for line in config_lines()]
    lines.append("//")
    lines += [f"//   {line}" for line in pid_summary()]
    lines.append("")
    if calibrate:
        lines += [
            "// Calibrates the IMU and starts odometry. It blocks for ~3 s and the",
            "// robot must be still and untouched while it runs. initialize() already",
            "// calibrates on startup, so drop this line if the robot has been sitting",
            "// since then -- it re-zeroes the heading, hence the setPose below it.",
            "chassis.calibrate();",
        ]
    if start is not None:
        lines.append(f"chassis.setPose({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f});"
                     "  // start pose")
    if calibrate or start is not None:
        lines.append("")
    for i, b in enumerate(blocks):
        lines.append(f"{block_code(b)}  // {i + 1}: {block_summary(b)}")
    return "\n".join(lines) + "\n"


# ─── Simulated route (for drawing the plan) ────────────────────────────────
# A small kinematic simulation of what lemlib 0.5 actually does with each
# block, on a differential-drive model of this robot, so the preview
# shows the curve a moveToPoint really takes (the robot turns *while*
# driving), the boomerang path of moveToPose, that a swing turn moves the
# robot (it pivots around the locked side), how backwards moves approach,
# and where min/max speed and early-exit cut things short.
#
# Control laws are transcribed from lemlib's motions (moveToPoint.cpp,
# moveToPose.cpp, turnTo.cpp, swingTo.cpp): same error terms, same PID
# form (per-loop derivative, 10 ms loop), same clamps. What it can't know
# is friction, wheel slip and motor lag, so treat it as "the shape and
# roughly where it ends", not a guarantee to the inch.

LOOP_S = 0.010   # lemlib motion loop period


class RobotSpec:
    """Drivetrain + PID numbers, read from src/main.cpp when it's there so
    the preview follows the robot's tuning; these are the fallbacks."""
    trackWidth = 15.0        # in
    wheelDiameter = 3.25     # in
    rpm = 458.0
    lateral = dict(kP=10, kI=0, kD=28, windup=3, smallErr=1, smallTime=100, largeErr=3,
                   largeTime=500, slew=20)
    angular = dict(kP=5.6, kI=0.001, kD=28.59, windup=0, smallErr=0, smallTime=0, largeErr=0,
                   largeTime=0, slew=0)

    @property
    def maxVel(self):   # in/s at full power
        return self.rpm * math.pi * self.wheelDiameter / 60.0

    @classmethod
    def fromMainCpp(cls):
        spec = cls()
        path = os.path.join(HERE, "..", "src", "main.cpp")
        try:
            src = open(path, encoding="utf-8").read()
        except OSError:
            return spec
        import re
        m = re.search(r"lemlib::Drivetrain\s+\w+\s*\([^,]+,[^,]+,\s*([\d.]+)\s*,\s*"
                      r"lemlib::Omniwheel::(\w+)\s*,\s*([\d.]+)", src)
        if m:
            spec.trackWidth = float(m.group(1))
            spec.wheelDiameter = {"NEW_2": 2.0, "NEW_275": 2.75, "OLD_275": 2.75, "NEW_325": 3.25,
                                  "OLD_325": 3.25, "NEW_4": 4.0, "OLD_4": 4.0}.get(m.group(2), 3.25)
            spec.rpm = float(m.group(3))
        keys = ("kP", "kI", "kD", "windup", "smallErr", "smallTime", "largeErr", "largeTime", "slew")
        for name, attr in (("lateral_controller", "lateral"), ("angular_controller", "angular")):
            m = re.search(r"lemlib::ControllerSettings\s+" + name + r"\s*\(([^)]*)\)", src)
            if m:
                vals = [float(v) for v in m.group(1).split(",")]
                if len(vals) == 9:
                    setattr(spec, attr, dict(zip(keys, vals)))
        return spec


SPEC = RobotSpec.fromMainCpp()


class PID:
    """lemlib::PID -- derivative is per loop, not per second."""
    def __init__(self, c):
        self.kP, self.kI, self.kD, self.windup = c["kP"], c["kI"], c["kD"], c["windup"]
        self.integral = 0.0
        self.prev = 0.0

    def update(self, err):
        self.integral += err
        if (err > 0) != (self.prev > 0) or (self.windup and abs(err) > self.windup):
            self.integral = 0.0
        d = err - self.prev
        self.prev = err
        return err * self.kP + self.integral * self.kI + d * self.kD


class ExitCondition:
    """lemlib::ExitCondition -- true once |err| has stayed inside `range`
    for `time` ms. range 0 (as in this robot's angular settings) never
    exits, exactly like the real thing: those motions run to timeout."""
    def __init__(self, rng, ms):
        self.rng, self.ms = rng, ms
        self.since = None

    def update(self, err, now):
        if self.rng <= 0 or abs(err) > self.rng:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.ms


def angle_err(target, current, direction="AUTO"):
    """Signed degrees to get from `current` to `target` heading: shortest
    way for AUTO, always clockwise (positive) for CW, counter (negative)
    for CCW -- lemlib::angleError."""
    e = (target - current) % 360
    if direction == "CW":
        return e
    if direction == "CCW":
        return e - 360
    return e - 360 if e > 180 else e


def heading_to(x0, y0, x1, y1, forwards=True):
    """Compass heading (deg, 0 = +y, clockwise) from (x0,y0) to (x1,y1)."""
    th = math.degrees(math.atan2(x1 - x0, y1 - y0))
    return (th + (0 if forwards else 180)) % 360


def slew(target, current, step):
    if step <= 0:
        return target
    return max(current - step, min(current + step, target))


class Robot:
    """Differential-drive integration. Powers are -127..127 per side;
    heading is compass degrees (clockwise positive), lemlib's convention:
    left = lateral + angular, so positive angular turns clockwise."""
    # Motors and the robot's inertia can't change wheel speed instantly;
    # the sides follow their commanded speed with roughly this time
    # constant. Without it the model would flip full-forward/full-reverse
    # every loop and the D terms would turn that into a vibration that
    # never happens on a real drivetrain.
    TAU_S = 0.12

    def __init__(self, x, y, th):
        self.x, self.y, self.th = x, y, th
        self.vl = self.vr = 0.0

    def step(self, left, right):
        k = SPEC.maxVel / 127.0
        a = LOOP_S / self.TAU_S
        self.vl += (max(-127, min(127, left)) * k - self.vl) * a
        self.vr += (max(-127, min(127, right)) * k - self.vr) * a
        vl, vr = self.vl, self.vr
        v = (vl + vr) / 2
        w = (vl - vr) / SPEC.trackWidth          # rad/s, clockwise positive
        self.th = (self.th + math.degrees(w * LOOP_S)) % 360
        r = math.radians(self.th)
        self.x += v * math.sin(r) * LOOP_S
        self.y += v * math.cos(r) * LOOP_S

    def pose(self):
        return (self.x, self.y, self.th)


def _apply_speed_limits(lateralOut, angularOut, maxSpeed, minSpeed, forwards, close):
    lateralOut = max(-maxSpeed, min(maxSpeed, lateralOut))
    angularOut = max(-maxSpeed, min(maxSpeed, angularOut))
    if not close:
        # never drive the wrong way while still far from the target
        lateralOut = max(lateralOut, 0) if forwards else min(lateralOut, 0)
        # keep at least minSpeed so chained motions don't stall
        if forwards and lateralOut < abs(minSpeed):
            lateralOut = abs(minSpeed)
        elif not forwards and lateralOut > -abs(minSpeed):
            lateralOut = -abs(minSpeed)
    # prevent motor saturation from eating the turn
    overturn = abs(angularOut) + abs(lateralOut) - maxSpeed
    if overturn > 0:
        lateralOut -= math.copysign(overturn, lateralOut)
    return lateralOut, angularOut


def sim_move_to_point(robot, b, trace):
    """lemlib moveToPoint: chase the point, turning while driving; once
    within 7.5 in it stops steering and just closes the distance."""
    fwd = b.get("forwards", True)
    tx, ty = b["x"], b["y"]
    lat, ang = PID(SPEC.lateral), PID(SPEC.angular)
    small = ExitCondition(SPEC.lateral["smallErr"], SPEC.lateral["smallTime"])
    large = ExitCondition(SPEC.lateral["largeErr"], SPEC.lateral["largeTime"])
    close = False
    prevLat = 0.0
    t = 0
    while t < b["timeout"]:
        x, y, th = robot.pose()
        eff = th if fwd else (th + 180) % 360
        d = math.hypot(tx - x, ty - y)
        if d < 7.5:
            close = True
        if d < b.get("earlyExitRange", 0):
            break
        angErr = angle_err(heading_to(x, y, tx, ty), eff)
        latErr = d * math.cos(math.radians(angErr))
        if not fwd:
            latErr = -latErr
        angOut = 0.0 if close else ang.update(angErr)
        latOut = slew(lat.update(latErr), prevLat, SPEC.lateral["slew"])
        latOut, angOut = _apply_speed_limits(latOut, angOut, b["maxSpeed"], b["minSpeed"], fwd, close)
        prevLat = latOut
        if small.update(latErr, t) or large.update(latErr, t):
            break
        robot.step(latOut + angOut, latOut - angOut)
        t += LOOP_S * 1000
        trace.append((robot.x, robot.y))
    return t


def sim_move_to_pose(robot, b, trace):
    """lemlib moveToPose (boomerang): chase a carrot point pulled back from
    the target along the target heading by lead * distance, so the robot
    arrives facing the right way; horizontalDrift caps speed in tight
    curves; when close, the carrot becomes the target itself."""
    fwd = b.get("forwards", True)
    tx, ty = b["x"], b["y"]
    tth = b["theta"] if fwd else (b["theta"] + 180) % 360
    lead, drift = b.get("lead", 0.6), b.get("horizontalDrift", 0.0)
    lat, ang = PID(SPEC.lateral), PID(SPEC.angular)
    small = ExitCondition(SPEC.lateral["smallErr"], SPEC.lateral["smallTime"])
    large = ExitCondition(SPEC.lateral["largeErr"], SPEC.lateral["largeTime"])
    close = False
    prevLat = 0.0
    t = 0
    while t < b["timeout"]:
        x, y, th = robot.pose()
        eff = th if fwd else (th + 180) % 360
        d = math.hypot(tx - x, ty - y)
        if d < 7.5:
            close = True
        if d < b.get("earlyExitRange", 0):
            break
        if close:
            cx, cy = tx, ty
        else:
            r = math.radians(tth)
            cx, cy = tx - d * lead * math.sin(r), ty - d * lead * math.cos(r)
        toCarrot = angle_err(heading_to(x, y, cx, cy), eff) if (cx, cy) != (x, y) else 0.0
        angErr = angle_err(tth, eff) if close else toCarrot
        dc = math.hypot(cx - x, cy - y)
        latErr = dc * math.cos(math.radians(toCarrot))
        if not fwd:
            latErr = -latErr
        angOut = ang.update(angErr)
        latOut = slew(lat.update(latErr), prevLat, SPEC.lateral["slew"])
        if drift > 0 and dc > 0.1:
            # curvature to the carrot -> max speed before the wheels slip
            hr = math.radians(eff)
            a = -math.tan(hr) if abs(math.cos(hr)) > 1e-6 else 1e6
            c = math.tan(hr) * x - y
            xp = abs(a * cx + cy + c) / math.hypot(a, 1)          # dist from carrot to heading line
            side = math.copysign(1, math.sin(hr) * (cy - y) - math.cos(hr) * (cx - x))
            curv = side * (2 * xp) / (dc * dc)
            if abs(curv) > 1e-6:
                maxSlip = math.sqrt(drift * (1 / abs(curv)) * 9.8)
                latOut = max(-maxSlip, min(maxSlip, latOut))
        latOut, angOut = _apply_speed_limits(latOut, angOut, b["maxSpeed"], b["minSpeed"], fwd, close)
        prevLat = latOut
        if small.update(latErr, t) or large.update(latErr, t):
            break
        robot.step(latOut + angOut, latOut - angOut)
        t += LOOP_S * 1000
        trace.append((robot.x, robot.y))
    return t


def sim_turn(robot, b, trace, targetHeading, swingSide=None):
    """lemlib turnToHeading / swingToHeading. In-place turns drive the
    sides opposite; a swing holds one side still, so the robot pivots
    around that wheel and its centre travels an arc of trackWidth/2."""
    ang = PID(SPEC.angular)
    small = ExitCondition(SPEC.angular["smallErr"], SPEC.angular["smallTime"])
    large = ExitCondition(SPEC.angular["largeErr"], SPEC.angular["largeTime"])
    direction = b.get("direction", "AUTO")
    settled = 0
    t = 0
    while t < b["timeout"]:
        x, y, th = robot.pose()
        # A forced direction only matters for choosing which way round;
        # once nearly there, settle the shortest way so an overshoot
        # doesn't turn into another full revolution.
        if abs(angle_err(targetHeading, th)) < 10:
            direction = "AUTO"
        err = angle_err(targetHeading, th, direction)
        if abs(err) < b.get("earlyExitRange", 0):
            break
        if small.update(err, t) or large.update(err, t):
            break
        # This robot's angular exits are all 0 (never exit), so the real
        # turn runs to timeout while holding the heading. For the preview,
        # stop once it has plainly settled -- the shape is the same.
        settled = settled + 1 if abs(err) < 0.75 else 0
        if settled >= 15:
            break
        out = ang.update(err)
        out = max(-b["maxSpeed"], min(b["maxSpeed"], out))
        if abs(out) < b["minSpeed"]:
            out = math.copysign(b["minSpeed"], out)
        if swingSide == "LEFT":
            robot.step(0, -out)
        elif swingSide == "RIGHT":
            robot.step(out, 0)
        else:
            robot.step(out, -out)
        t += LOOP_S * 1000
        trace.append((robot.x, robot.y))
    return t


def simulate(blocks, start):
    """Runs the block list through the simulator from `start` = (x, y,
    theta). Returns, per block, (endPose, shape):
        ("trace", [(x, y), ...])          the path the robot centre takes
        ("turn", x, y, fromHdg, toHdg)    an in-place turn (drawn as an arc)
        ("swing", x, y, fromHdg, toHdg, trace)  a swing: arc + the centre's path
        None                              nothing on the map (wait, ...)
    Each block starts where the previous one ended, so the shapes chain
    into the full route."""
    robot = Robot(*start)
    out = []
    for b in blocks:
        k = b["kind"]
        shape = None
        trace = [(robot.x, robot.y)]
        x0, y0, th0 = robot.pose()
        if k == "setPose":
            robot = Robot(b["x"], b["y"], b["theta"])
        elif k == "moveToPoint":
            sim_move_to_point(robot, b, trace)
            shape = ("trace", trace)
        elif k == "moveToPose":
            sim_move_to_pose(robot, b, trace)
            shape = ("trace", trace)
        elif k in ("turnToHeading", "turnToPoint", "swingToHeading", "swingToPoint"):
            if k.endswith("Heading"):
                target = b["theta"]
            else:
                target = heading_to(x0, y0, b["x"], b["y"], b.get("forwards", True)) \
                    if (b["x"], b["y"]) != (x0, y0) else th0
            swingSide = b.get("side") if k.startswith("swing") else None
            sim_turn(robot, b, trace, target, swingSide)
            if swingSide:
                shape = ("swing", x0, y0, th0, robot.th, trace)
            else:
                shape = ("turn", x0, y0, th0, robot.th)
        out.append((robot.pose(), shape))
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
        self.drawMode = False          # mouse path editor on/off
        self.waypoints = []            # control points: (x, y, CURVE/STRAIGHT)
        self.drag = None               # ("wp", i) or ("block", i) while dragging
        self.robotPose = None
        self.robotPath = []
        self.planStatus = ("IDLE", -1, 0)
        self.pendingRun = None
        self.uploadStarted = 0.0
        self.uploadTries = 0
        self.statusHold = 0.0
        self.onBrainScreen = False
        self.fieldImage = None
        self._redrawPending = False    # see redrawMap(): draws are coalesced
        self._simCache = (None, None)  # (key, simulate() result)
        self._lastState = None         # last robot state drawn, to skip no-ops
        self.protocol("WM_DELETE_WINDOW", self.withdraw)  # hide, don't destroy

        self._buildUi()
        self._bindKeys()
        self.redrawMap()
        self.refreshList()
        self._lockMinSize()

    def _lockMinSize(self):
        """Stop the window being sized smaller than its controls need.
        Tk's packer hides whatever doesn't fit rather than scrolling or
        wrapping, and a button you can't see is a button that isn't there
        as far as anyone using it is concerned. Capped to the screen, so a
        small display still gets a usable window."""
        self.update_idletasks()
        w = min(self.winfo_reqwidth(), self.winfo_screenwidth())
        h = min(self.winfo_reqheight(), self.winfo_screenheight() - 60)
        self.minsize(w, h)

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
        self.map.bind("<Button-1>", self.onMapPress)
        self.map.bind("<B1-Motion>", self.onMapDrag)
        self.map.bind("<ButtonRelease-1>", self.onMapRelease)
        self.map.bind("<Button-3>", self.onMapRightClick)
        self.map.bind("<Shift-Button-1>", lambda e: self.removeWaypointAt(e.x, e.y))
        self.map.bind("<Button-2>", lambda e: self.removeWaypointAt(e.x, e.y))
        self.map.bind("<Motion>", self.onMapMotion)
        self.mapStatus = tk.Label(mapFrame, text="", anchor="w", fg="#999", bg="#14141a",
                                  font=("Consolas", 9))
        self.mapStatus.pack(fill="x")

        # mouse path editor -- drop control points on the map and turn the
        # route through them into blocks (path.jerryio style).
        dp = tk.LabelFrame(mapFrame, text="mouse path", fg="#ccc", bg="#14141a",
                           font=("Segoe UI", 9))
        dp.pack(fill="x", pady=(6, 0))
        row = tk.Frame(dp, bg="#14141a")
        row.pack(fill="x", padx=4, pady=(4, 0))
        self.drawBtn = tk.Button(row, text="Draw path  (D)", command=self.toggleDraw)
        self.drawBtn.pack(side="left")
        self.smoothVar = tk.BooleanVar(value=True)
        tk.Checkbutton(row, text="smooth", variable=self.smoothVar, bg="#14141a", fg="#ddd",
                       selectcolor="#161B22", activebackground="#14141a",
                       activeforeground="#ddd", command=self.redrawMap).pack(side="left", padx=6)
        tk.Label(row, text="spacing", fg="#888", bg="#14141a").pack(side="left")
        self.spacingVar = tk.StringVar(value="12")
        e = tk.Entry(row, textvariable=self.spacingVar, width=5, font=("Consolas", 10))
        e.pack(side="left", padx=(2, 6))
        e.bind("<Return>", lambda _e: self.redrawMap())
        e.bind("<FocusOut>", lambda _e: self.redrawMap())
        self.reverseVar = tk.BooleanVar(value=False)
        tk.Checkbutton(row, text="backwards", variable=self.reverseVar, bg="#14141a", fg="#ddd",
                       selectcolor="#161B22", activebackground="#14141a",
                       activeforeground="#ddd").pack(side="left")
        tk.Label(dp, text="left-click = curve point    right-click = straight point / flip"
             "    shift-click = remove", fg="#8a8a96", bg="#14141a",
             font=("Segoe UI", 8), anchor="w").pack(fill="x", padx=6)
        row2 = tk.Frame(dp, bg="#14141a")
        row2.pack(fill="x", padx=4, pady=(2, 4))
        for text, cmd in (("Apply", self.applyDrawnPath), ("Undo point", self.undoWaypoint),
                          ("Clear points", self.clearWaypoints),
                          ("Trace blocks", self.tracePointsFromBlocks),
                          ("Clear map", self.clearMap)):
            tk.Button(row2, text=text, command=cmd).pack(side="left", padx=3)
        self.drawInfo = tk.Label(row2, text="", fg="#777", bg="#14141a", font=("Segoe UI", 8))
        self.drawInfo.pack(side="left", padx=6)

        self._loadFieldImage()

        # Right column: blocks + parameters --------------------------------
        right = tk.Frame(top, bg="#14141a")
        right.pack(side="left", fill="both", expand=True, padx=(0, 8), pady=8)

        hdr = tk.Frame(right, bg="#14141a")
        hdr.pack(fill="x")
        self.brainStatus = tk.Label(hdr, text="brain: waiting...", anchor="w", fg="#00F0FF",
                                    bg="#14141a", font=("Segoe UI", 10, "bold"))
        self.brainStatus.pack(side="left")

        # Start pose: the numbers on one row and the buttons on the next.
        # Packed side by side they needed more width than the column had, and
        # pack answers that by dropping whatever is last -- which is how
        # "Reset robot" would quietly not be there at all.
        sp = tk.Frame(right, bg="#14141a")
        sp.pack(fill="x", pady=(6, 0))
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

        sp2 = tk.Frame(right, bg="#14141a")
        sp2.pack(fill="x", pady=(2, 2))
        self.pickBtn = tk.Button(sp2, text="Pick start on map", command=self.togglePickStart)
        self.pickBtn.pack(side="left")
        tk.Button(sp2, text="Use robot's pose", command=self.useRobotPose).pack(side="left", padx=6)
        tk.Button(sp2, text="Reset robot", command=self.resetRobot, fg="#FF6B81").pack(side="left")

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
        self.calibrateVar = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text="calibrate first", variable=self.calibrateVar, bg="#14141a",
                       fg="#ddd", selectcolor="#161B22", activebackground="#14141a",
                       activeforeground="#ddd").pack(side="left")
        tk.Button(bar, text="Save", command=self.savePlan).pack(side="left", padx=3)
        tk.Button(bar, text="Load", command=self.loadPlan).pack(side="left", padx=3)

        self.hint = tk.Label(right, text="Click the map to set the selected block's point, or "
                             "drag any point on it.  D = draw a path with the mouse.  "
                             "Delete = remove block.  Ctrl+S / Ctrl+O save / load.  Ctrl+E export.",
                             anchor="w", justify="left", wraplength=520, fg="#777", bg="#14141a",
                             font=("Segoe UI", 8))
        self.hint.pack(fill="x", pady=(6, 0))

    def _bindKeys(self):
        self.bind("<F5>", lambda e: self.runAll())
        self.bind("<F6>", lambda e: self.runSelected())
        self.bind("<Escape>", lambda e: self.stop())
        self.bind("<Control-s>", lambda e: self.savePlan())
        self.bind("<Control-o>", lambda e: self.loadPlan())
        self.bind("<Control-e>", lambda e: self.exportCode())
        self.listbox.bind("<Delete>", lambda e: self.deleteBlock())
        # ...and the draw-path keys, which must not fire while a number is
        # being typed into one of the entries (Tk sends the key to the
        # widget first, then here).
        self.bind("<d>", lambda e: None if self._typing() else self.toggleDraw())
        self.bind("<D>", lambda e: None if self._typing() else self.toggleDraw())
        self.bind("<Control-z>", lambda e: None if self._typing() else self.undoWaypoint())
        self.bind("<BackSpace>", lambda e: None if self._typing() else self.undoWaypoint())
        self.bind("<Return>", lambda e: self.applyDrawnPath()
                  if self.drawMode and not self._typing() else None)

    def _typing(self):
        """True when the keyboard focus is in a text entry / combobox."""
        w = self.focus_get()
        return isinstance(w, (tk.Entry, tk.Text)) or isinstance(w, ttk.Combobox) or \
            isinstance(w, ttk.Entry)

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
    def simulation(self):
        """simulate() for the current blocks, cached. The map is redrawn
        on every state the robot sends (20 a second), and re-running the
        whole motion simulation each time is what made it crawl and
        flicker -- the route only changes when the blocks or the start
        pose do."""
        key = (repr(self.blocks), tuple(self.start))
        if self._simCache[0] != key:
            self._simCache = (key, simulate(self.blocks, tuple(self.start)))
        return self._simCache[1]

    def redrawMap(self):
        """Ask for a redraw. Several calls in one go (editing a block
        refreshes the list, the params and the map) collapse into a single
        draw at idle, so the map doesn't flicker or fall behind."""
        if self._redrawPending:
            return
        self._redrawPending = True
        self.after_idle(self._drawMap)

    def _drawMap(self):
        self._redrawPending = False
        c = self.map
        c.delete("all")
        if self.fieldImage:
            c.create_image(0, 0, anchor="nw", image=self.fieldImage)
            # A field render is a busy thing to draw a route over, so it's
            # knocked back with a stippled dark wash (Tk canvases have no
            # alpha) -- the field still reads, the path sits clearly on top.
            c.create_rectangle(0, 0, MAP_PX, MAP_PX, fill="#0A0A12", stipple="gray25",
                               outline="")
        else:
            c.create_rectangle(0, 0, MAP_PX, MAP_PX, fill="#1b1b22", outline="")
        # tile grid (24 in) either way -- faint so it reads over the render
        for v in range(-72, 73, 24):
            px, _ = self.toMap(v, 0)
            _, py = self.toMap(0, v)
            col = "#5a5a66" if v == 0 else "#33333d"
            c.create_line(px, 0, px, MAP_PX, fill=col, dash=(2, 6))
            c.create_line(0, py, MAP_PX, py, fill=col, dash=(2, 6))

        # planned route: every block's movement, drawn from where the
        # previous block left the robot, so the whole stack reads as one
        # route. Unselected blocks in pink, the selected one in cyan.
        sim = self.simulation()
        self.drawRobotMarker(tuple(self.start), "#FF8C00", "S")
        for i, (b, (pose, shape)) in enumerate(zip(self.blocks, sim)):
            sel = (i == self.selected)
            col = "#00F0FF" if sel else "#FF6B81"
            w = 4 if sel else 2
            dash = (6, 3) if not b.get("forwards", True) else None
            if shape and shape[0] == "trace":
                self.drawTrace(shape[1], col, w, dash)
            elif shape and shape[0] in ("turn", "swing"):
                _, x, y, a0, a1 = shape[:5]
                if shape[0] == "swing":
                    self.drawTrace(shape[5], col, w, None, arrow=False)
                self.drawTurnArc(x, y, a0, a1, col, w, swing=(shape[0] == "swing"))
                if b["kind"] in ("turnToPoint", "swingToPoint"):
                    x0, y0 = self.toMap(x, y)
                    x1, y1 = self.toMap(b["x"], b["y"])
                    c.create_line(x0, y0, x1, y1, fill=col, width=1, dash=(2, 4))
                    c.create_oval(x1 - 4, y1 - 4, x1 + 4, y1 + 4, outline=col)
            if b["kind"] in MOTIONS or b["kind"] == "setPose":
                self.drawRobotMarker(pose, col, str(i + 1), small=True)

        # robot: driven path then live pose
        if len(self.robotPath) > 1:
            pts = [self.toMap(x, y) for x, y in self.robotPath]
            self.drawPolyline([v for p in pts for v in p], "#2EFF8C", 2)
        if self.robotPose:
            self.drawRobotMarker(self.robotPose, "#2EFF8C", "", robot=True)

        self.drawWaypoints()

    # -- mouse path editor --
    def spacing(self):
        try:
            return max(2.0, float(self.spacingVar.get()))
        except (ValueError, tk.TclError):
            return 12.0

    def pathPoints(self):
        """The drawn route as (x, y, stop) points -- see path_points()."""
        return path_points(self.waypoints, self.spacing(), self.smoothVar.get())

    def drawWaypoints(self):
        """Control points and the route through them, live while drawing."""
        if not self.waypoints:
            if self.drawMode:
                self.drawInfo.config(text="click the map to drop points")
            return
        c = self.map
        pts = self.pathPoints()
        if len(pts) > 1:
            flat = [v for x, y, _ in pts for v in self.toMap(x, y)]
            self.drawPolyline(flat, "#FFD166", 3,
                              dash=(7, 4) if self.reverseVar.get() else None)
            self.drawDirectionArrows([(x, y) for x, y, _ in pts], "#FFD166")
        for i, w in enumerate(self.waypoints):
            px, py = self.toMap(w[0], w[1])
            straight = len(w) > 2 and w[2] == STRAIGHT
            r = 8 if i == 0 else 6
            col = "#FF8C00" if i == 0 else ("#7CE0FF" if straight else "#FFD166")
            if straight:      # square = hard corner, circle = flows through
                c.create_rectangle(px - r, py - r, px + r, py + r, outline="#101018", width=4)
                c.create_rectangle(px - r, py - r, px + r, py + r, outline=col, width=2,
                                   fill="#14141a")
            else:
                c.create_oval(px - r - 1, py - r - 1, px + r + 1, py + r + 1,
                              outline="#101018", width=4)
                c.create_oval(px - r, py - r, px + r, py + r, outline=col, width=2,
                              fill="#14141a")
            c.create_text(px, py, text=str(i + 1) if i else "S", fill=col,
                          font=("Segoe UI", 7, "bold"))
        curves = sum(1 for w in self.waypoints if len(w) < 3 or w[2] == CURVE)
        dist = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(pts, pts[1:]))
        shape = ("turnToHeading + moveToPoint per leg" if not self.smoothVar.get()
                 else f"{len(pts) - 1} moveToPoint")
        self.drawInfo.config(
            text=f"{len(self.waypoints)} points ({curves} curve, "
                 f"{len(self.waypoints) - curves} straight)  ->  {shape}  ({dist:.0f} in)")

    def toggleDraw(self):
        self.drawMode = not self.drawMode
        self.drawBtn.config(relief="sunken" if self.drawMode else "raised")
        if self.drawMode and self.pickStart:
            self.togglePickStart()
        self.redrawMap()

    def undoWaypoint(self):
        if self.waypoints:
            self.waypoints.pop()
            self.redrawMap()

    def clearWaypoints(self):
        self.waypoints = []
        self.drawInfo.config(text="")
        self.redrawMap()

    def clearMap(self):
        """Wipe the lines off the map: the drawn control points and the
        path the robot actually drove. The movement blocks are the plan
        rather than scribble, so those are only cleared if asked for."""
        self.waypoints = []
        self.robotPath = []
        self.drawInfo.config(text="")
        if self.blocks and messagebox.askyesno(
                "Clear map", f"Also remove the {len(self.blocks)} movement "
                             "block{} and the route they draw?".format(
                                 "" if len(self.blocks) == 1 else "s"),
                default="no", parent=self):
            self.blocks = []
            self.selected = None
            self.showParams()
        self.refreshList()

    def toggleWaypointMode(self, i):
        """Flip a dropped point between curve and straight."""
        x, y = self.waypoints[i][0], self.waypoints[i][1]
        mode = self.waypoints[i][2] if len(self.waypoints[i]) > 2 else CURVE
        self.waypoints[i] = (x, y, STRAIGHT if mode == CURVE else CURVE)
        self.redrawMap()

    def removeWaypointAt(self, ex, ey):
        i = self._hitWaypoint(ex, ey)
        if i is not None:
            del self.waypoints[i]
            self.redrawMap()

    def tracePointsFromBlocks(self):
        """Load the route the blocks already describe back in as control
        points, so an existing plan can be re-shaped with the mouse."""
        # traced back as straight points, so the route is reproduced exactly
        # as the blocks describe it -- right-click any of them to round it off.
        pts = [(round(self.start[0], 1), round(self.start[1], 1), STRAIGHT)]
        for b in self.blocks:
            if b["kind"] in ("moveToPoint", "moveToPose", "setPose"):
                pt = (round(b["x"], 1), round(b["y"], 1), STRAIGHT)
                if math.hypot(pt[0] - pts[-1][0], pt[1] - pts[-1][1]) > 0.05:
                    pts.append(pt)
        self.waypoints = pts
        if not self.drawMode:
            self.toggleDraw()
        else:
            self.redrawMap()

    # A leg is treated as carrying straight on (no turn block) below this
    # much heading change -- a turnToHeading for a degree or two would cost
    # its whole timeout for nothing.
    TURN_DEG = 3.0

    def applyDrawnPath(self):
        """Turn the drawn route into blocks: first point = start pose
        (facing the next point), every point after it a moveToPoint.

        With "smooth" on, points along a curve keep moving through the
        corner (minSpeed + earlyExitRange) instead of stopping dead on
        each. With it off the route is turn-and-drive: wherever it turns
        left or right the robot is aimed down the next leg with a
        turnToHeading first, so the moveToPoint after it drives a
        straight line instead of steering onto the target as it goes."""
        pts = self.pathPoints()
        if len(pts) < 2:
            messagebox.showinfo("Draw path", "Drop at least two points on the map first.",
                                parent=self)
            return
        replace = True
        if self.blocks:
            ans = messagebox.askyesnocancel(
                "Apply path",
                "Replace the current blocks with the drawn path?\n\n"
                "Yes  -- replace them (the first point becomes the start pose)\n"
                "No   -- append the path to the end of the list",
                parent=self)
            if ans is None:
                return
            replace = ans
        fwd = not self.reverseVar.get()
        exitRange = round(self.spacing() / 2.0, 2)
        # With "smooth" off the route is a turn-and-drive one: aim down each
        # leg first, so the moveToPoint that follows is a straight line.
        turnFirst = not self.smoothVar.get()
        blocks = []
        if replace:
            x0, y0 = pts[0][0], pts[0][1]
            hdg = heading_to(x0, y0, pts[1][0], pts[1][1], fwd)
            self.start = [round(x0, 1), round(y0, 1), round(hdg, 1)]
            for v, val in zip(self.startVars, self.start):
                v.set(fmt(val))
            rest = pts[1:]
            cx, cy, ch = x0, y0, hdg
        else:
            rest = pts        # appended: drive to the first point too
            sim = self.simulation()
            cx, cy, ch = sim[-1][0] if sim else tuple(self.start)
        for i, (x, y, stop) in enumerate(rest):
            if turnFirst:
                # only where the route actually turns -- a leg carrying
                # straight on doesn't need (or want) a turn block
                want = heading_to(cx, cy, x, y, fwd)
                if abs((want - ch + 180) % 360 - 180) > self.TURN_DEG:
                    blocks.append(new_block("turnToHeading", theta=round(want, 2)))
                    blocks.append(new_block("waitUntilDone"))
                ch = want       # the drive that follows holds this heading
            # points along a curve are flowed through (minSpeed + early exit);
            # straight points and the last point are arrived on properly.
            stop = stop or i == len(rest) - 1
            blocks.append(new_block("moveToPoint", x=round(x, 2), y=round(y, 2), forwards=fwd,
                                    minSpeed=0 if stop else 40,
                                    earlyExitRange=0.0 if stop else exitRange))
            blocks.append(new_block("waitUntilDone"))
            cx, cy = x, y
        self.blocks = blocks if replace else self.blocks + blocks
        self.selected = len(self.blocks) - len(blocks)
        self.refreshList()
        self.showParams()

    # -- map hit-testing --
    HIT_PX = 9

    def _hitWaypoint(self, ex, ey):
        for i, w in enumerate(self.waypoints):
            px, py = self.toMap(w[0], w[1])
            if math.hypot(px - ex, py - ey) <= self.HIT_PX:
                return i
        return None

    def _hitBlockPoint(self, ex, ey):
        """A block's target point under the cursor -- the selected block
        wins, so overlapping points stay draggable."""
        order = list(range(len(self.blocks)))
        if self.selected is not None and self.selected < len(self.blocks):
            order.remove(self.selected)
            order.insert(0, self.selected)
        for i in order:
            b = self.blocks[i]
            if b["kind"] not in HAS_POINT:
                continue
            px, py = self.toMap(b["x"], b["y"])
            if math.hypot(px - ex, py - ey) <= self.HIT_PX:
                return i
        return None

    # Everything on the map is drawn as a dark halo with the bright line
    # on top of it, so a route stays readable wherever it crosses the field
    # render, the grid, or another route.
    HALO = "#0B0B12"

    def drawPolyline(self, flat, color, width, dash=None, arrow=None):
        if len(flat) < 4:
            return
        self.map.create_line(*flat, fill=self.HALO, width=width + 4, dash=dash,
                             capstyle="round", joinstyle="round")
        self.map.create_line(*flat, fill=color, width=width, dash=dash, arrow=arrow,
                             capstyle="round", joinstyle="round")

    def drawDirectionArrows(self, pts, color, every_px=70):
        """Arrowheads spaced along a route, so which way round it goes is
        readable without following it end to end."""
        px = [self.toMap(x, y) for x, y in pts]
        run = 0.0
        for (x0, y0), (x1, y1) in zip(px, px[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue
            run += seg
            if run < every_px:
                continue
            run = 0.0
            ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
            self.map.create_line(x1 - ux * 9, y1 - uy * 9, x1, y1, fill=self.HALO, width=6,
                                 arrow="last", arrowshape=(10, 12, 5))
            self.map.create_line(x1 - ux * 9, y1 - uy * 9, x1, y1, fill=color, width=2,
                                 arrow="last", arrowshape=(9, 11, 4))

    def drawTrace(self, trace, color, width, dash, arrow=True):
        """The simulated path of the robot centre, thinned so Tk isn't
        handed thousands of segments."""
        step = max(1, len(trace) // 120)
        pts = trace[::step]
        if pts[-1] != trace[-1]:
            pts.append(trace[-1])
        if len(pts) < 2 or all(math.hypot(px - pts[0][0], py - pts[0][1]) < 0.05 for px, py in pts):
            return
        flat = [v for x, y in pts for v in self.toMap(x, y)]
        self.drawPolyline(flat, color, width, dash, "last" if arrow else None)

    def drawTurnArc(self, x, y, a0, a1, color, width, swing=False):
        """Arc from heading a0 to a1 around (x, y) -- the short way round,
        like lemlib's AUTO direction -- with an arrowhead at the end. Swings
        pivot about one side of the drivetrain, so their arc is offset and
        drawn dashed to tell them apart."""
        px, py = self.toMap(x, y)
        r = 22
        delta = (a1 - a0 + 180) % 360 - 180   # signed, shortest
        if abs(delta) < 0.5:
            return
        # Tk arcs: start angle CCW from +x; compass heading h -> 90 - h
        start = 90 - a0
        extent = -delta
        if swing:
            off = 9
            side = 1 if delta > 0 else -1
            rad = math.radians(a0 + 90 * side)
            px, py = px + math.sin(rad) * off, py - math.cos(rad) * off
        self.map.create_arc(px - r, py - r, px + r, py + r, start=start, extent=extent,
                            style="arc", outline=self.HALO, width=width + 4)
        self.map.create_arc(px - r, py - r, px + r, py + r, start=start, extent=extent,
                            style="arc", outline=color, width=width,
                            dash=(3, 3) if swing else None)
        end = math.radians(a1)
        ex, ey = px + math.sin(end) * r, py - math.cos(end) * r
        tang = math.radians(a1 + (90 if delta > 0 else -90))
        self.map.create_line(ex - math.sin(tang) * 6, ey + math.cos(tang) * 6, ex, ey,
                             fill=color, width=width, arrow="last")

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
        self.map.create_oval(px - r - 1, py - r - 1, px + r + 1, py + r + 1,
                             outline=self.HALO, width=4)
        self.map.create_oval(px - r, py - r, px + r, py + r, outline=color, width=2,
                             fill="#14141a" if not robot else "")
        self.map.create_line(px, py, tx, ty, fill=self.HALO, width=5, arrow="last")
        self.map.create_line(px, py, tx, ty, fill=color, width=2, arrow="last")
        if label:
            self.map.create_text(px, py, text=label, fill=color, font=("Segoe UI", 8, "bold"))

    def onMapMotion(self, e):
        x, y = self.toField(e.x, e.y)
        if self.pickStart:
            hint = "   -- click to set START pose"
        elif self.drawMode:
            i = self._hitWaypoint(e.x, e.y)
            hint = ("   -- drag to move, right-click to flip curve/straight, "
                    "shift-click to remove") if i is not None else \
                   "   -- left-click = curve point, right-click = straight point"
        elif self._hitBlockPoint(e.x, e.y) is not None:
            hint = "   -- drag to move this block's point"
        else:
            hint = ""
        self.mapStatus.config(text=f"({x:6.1f}, {y:6.1f}) in" + hint)

    def onMapPress(self, e):
        x, y = self.toField(e.x, e.y)
        x, y = round(x, 1), round(y, 1)
        if self.pickStart:
            self.start[0], self.start[1] = x, y
            self.startVars[0].set(fmt(x))
            self.startVars[1].set(fmt(y))
            self.togglePickStart()
            self.redrawMap()
            return
        if self.drawMode:
            i = self._hitWaypoint(e.x, e.y)
            if i is None:                      # new curve point at the end
                self.waypoints.append((x, y, CURVE))
                i = len(self.waypoints) - 1
            self.drag = ("wp", i)
            self.redrawMap()
            return
        i = self._hitBlockPoint(e.x, e.y)      # grab a block's point and drag it
        if i is not None:
            if i != self.selected:
                self.selected = i
                self.refreshList()
                self.showParams()
            self.drag = ("block", i)
            return
        if self.selected is None or self.blocks[self.selected]["kind"] not in HAS_POINT:
            return
        b = self.blocks[self.selected]
        b["x"], b["y"] = x, y
        self.drag = ("block", self.selected)
        self.showParams()
        self.refreshList()

    def onMapDrag(self, e):
        if not self.drag:
            return
        x, y = self.toField(e.x, e.y)
        x, y = round(x, 1), round(y, 1)
        what, i = self.drag
        if what == "wp":
            if i < len(self.waypoints):
                w = self.waypoints[i]
                self.waypoints[i] = (x, y, w[2] if len(w) > 2 else CURVE)
                self.redrawMap()
        elif i < len(self.blocks):
            b = self.blocks[i]
            b["x"], b["y"] = x, y
            self.codeLabel.config(text=block_code(b))
            self.refreshList()          # redraws the map too

    def onMapRelease(self, _e):
        if self.drag and self.drag[0] == "block":
            self.showParams()
        self.drag = None

    def onMapRightClick(self, e):
        """Right button: drop a STRAIGHT point -- the legs either side of
        it stay dead straight and the robot stops on it. On a point that's
        already there, flip it between straight and curve instead."""
        if not self.drawMode:
            return
        i = self._hitWaypoint(e.x, e.y)
        if i is not None:
            self.toggleWaypointMode(i)
            return
        x, y = self.toField(e.x, e.y)
        self.waypoints.append((round(x, 1), round(y, 1), STRAIGHT))
        self.redrawMap()

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

    def resetRobot(self):
        """Forget everything the robot thinks it knows about where it is:
        its pose goes back to the start pose (chassis.setPose), the trail
        it drew on the brain and the driven path on this map are cleared,
        and the last run's status is dropped."""
        self.applyStartPose()
        self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")
        self.link._send("PLAN CLEAR")   # also resets the brain's step/status display
        self.robotPath = []
        self.robotPose = tuple(self.start)
        self.pendingRun = None
        self.redrawMap()

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
        sim = self.simulation()
        end = sim[-1][0] if sim else tuple(self.start)
        b = new_block(kind)
        if kind in HAS_POINT:
            b["x"], b["y"] = round(end[0], 1), round(end[1], 1)
        if kind in HAS_HEADING:
            b["theta"] = round(end[2], 1)
        at = len(self.blocks) if self.selected is None else self.selected + 1
        # ...but never between a movement and the waitUntilDone that
        # belongs to it.
        if at < len(self.blocks) and self.blocks[at]["kind"] == "waitUntilDone":
            at += 1
        self.blocks.insert(at, b)
        # lemlib runs motions asynchronously, so every movement gets a
        # waitUntilDone block after it -- delete it if you really do want
        # the next block to start while the robot is still moving.
        if kind in MOTIONS:
            self.blocks.insert(at + 1, new_block("waitUntilDone"))
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
    # Uploading is a burst of lines (CLEAR, one ADD per block) and over the
    # controller's radio a line can go missing or arrive late. So instead
    # of firing PLAN RUN blindly right after, wait until the brain reports
    # (in every RUI line) that it holds exactly our number of blocks, then
    # send the run. If the count doesn't settle, resend once, then give up
    # with a message instead of silently doing nothing.
    UPLOAD_TIMEOUT_S = 6.0
    UPLOAD_RETRIES = 2

    def uploadPlan(self):
        self.link._send("PLAN CLEAR")
        for b in self.blocks:
            self.link._send(block_to_plan_add(b))
        self.link._send("NOP")   # chaser: nudges a stuck last line through the radio
        self.uploadStarted = time.time()

    def startRun(self, which):
        """which: None = all, int = that block."""
        if not self.blocks:
            return
        self.pendingRun = "ALL" if which is None else which
        self.uploadTries = 1
        self.uploadPlan()
        self.brainStatus.config(text=f"uploading {len(self.blocks)} blocks to the robot...",
                                fg="#FF8C00")

    def runAll(self):
        self.startRun(None)

    def runSelected(self):
        if self.selected is not None:
            self.startRun(self.selected)

    def checkPendingRun(self, count):
        if self.pendingRun is None:
            return
        if count == len(self.blocks):
            cmd = "PLAN RUN" if self.pendingRun == "ALL" else f"PLAN RUN {self.pendingRun}"
            self.link._send(cmd)
            self.link._send("NOP")
            self.pendingRun = None
            self.brainStatus.config(text="run sent", fg="#2ECC71")
        elif time.time() - self.uploadStarted > self.UPLOAD_TIMEOUT_S:
            if self.uploadTries < self.UPLOAD_RETRIES:
                self.uploadTries += 1
                self.uploadPlan()
                self.brainStatus.config(
                    text=f"robot has {count}/{len(self.blocks)} blocks -- resending", fg="#FF8C00")
            else:
                self.pendingRun = None
                self.statusHold = time.time() + 8   # keep the message readable
                self.brainStatus.config(
                    text=f"upload failed: robot has {count}/{len(self.blocks)} blocks. "
                         "Is it running this firmware? (check --debug output)", fg="#FF6B81")

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
        self.checkPendingRun(count)
        # The robot sends a state 20 times a second whether or not anything
        # moved; redrawing an identical map just makes it shimmer.
        sig = (None if not self.robotPose else tuple(round(v, 2) for v in self.robotPose),
               len(self.robotPath), status, idx, count, self.onBrainScreen)
        unchanged = sig == self._lastState
        self._lastState = sig
        if self.pendingRun is not None or time.time() < self.statusHold:
            if not unchanged:
                self.redrawMap()
            return
        if self.onBrainScreen:
            txt = f"brain: PLANNER  {status}  ({count} block{'' if count == 1 else 's'} on robot)"
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
        if not unchanged:
            self.redrawMap()

    # -- files --
    def exportCode(self):
        self.applyStartPose()
        code = export_code(self.blocks, self.start, self.calibrateVar.get())
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
            json.dump({"start": self.start, "blocks": self.blocks,
                       "waypoints": [[p[0], p[1], p[2] if len(p) > 2 else CURVE]
                                     for p in self.waypoints]}, f, indent=2)

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
        self.waypoints = [(float(p[0]), float(p[1]),
                           p[2] if len(p) > 2 and p[2] in (CURVE, STRAIGHT) else CURVE)
                          for p in data.get("waypoints", [])]
        self.selected = 0 if self.blocks else None
        self.refreshList()
        self.showParams()
