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

DRIVING IT FROM HERE
    "Drive (WASD)" hands the drivetrain to the keyboard: W / S drive
    forwards and back, A / D turn, held together they mix (arcade). The
    power slider sets how hard, and the robot only moves while a key is
    actually down.

    The robot is told to move by "DRIVE left right" lines, which expire on
    the brain after 300 ms. The planner repeats the current command as a
    keepalive while you hold a key and sends a stop the moment you let go,
    lose the window's focus, or switch the mode off -- so a dropped radio
    line, a window that loses focus mid-key, or this program being killed
    outright all end with the robot stopped rather than driving on.

    This needs the firmware that understands DRIVE (src/driver_menu.cpp);
    an older brain will simply ignore it. The brain has to be on the
    driver menu (any screen of it) for the command to be applied.

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

import copy
import json
import math
import os
import re
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD_PNG = os.path.join(HERE, "field.png")
PLANS_DIR = os.path.join(HERE, "plans")
EXPORT_PATH = os.path.join(HERE, "exported_path.cpp")

FIELD_IN = 144.0            # field is 12 ft square
FIELD_BG = "#2b2b2b"        # map ground and window chrome -- path.jerryio's flat dark grey
PANEL_BG = "#2b2b2b"        # window background: the same grey as the map ground
INPUT_BG = "#1f1f1f"        # text entries and the block list, a step darker
PATH_GREEN = "#8bd13f"     # the route line
PATH_SEL = "#c6f27a"        # ...the selected block's leg
PREVIEW = "#4fc3f7"         # the motion being placed, before Apply
PURPLE = "#7a5cd6"          # path points and the robot footprint
POINT_FILL = "#4b3d8f"      # the inside of a path point
END_BOX = "#b9a6ff"         # the robot footprint at the end of a path
DEFAULT_ROBOT_IN = 15.0    # robot length / width until set in the window
MAP_PX = 500               # map is drawn this many px square
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
    # swingToHeading has no forwards in lemlib: here it only decides which side
    # is locked (a backward swing locks the other one) and is not exported
    "swingToHeading": ["theta", "side", "timeout", "forwards", "direction", "maxSpeed",
                       "minSpeed", "earlyExitRange"],
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


def path_points(waypoints, spacing, smooth=True, legs=None):
    """The route through the drawn control points, as (x, y, stop) --
    stop marking the points the robot should actually arrive on rather
    than flow through.

    A point is either CURVE or STRAIGHT (left / right click on the map).
    Runs of neighbouring curve points are splined together; any leg that
    touches a straight point stays a straight line, and the robot stops
    where a curve meets a straight leg, because the two don't share a
    tangent there.

    If `legs` is a list, it is filled with one flag per returned point after
    the first: True when the leg that ends at that point is a straight one
    (drawn with a right click), False when it is part of a curve."""
    pts = [(w[0], w[1]) for w in waypoints]
    modes = [w[2] if len(w) > 2 else CURVE for w in waypoints]
    if len(pts) < 2:
        return [(x, y, True) for x, y in pts]
    if not smooth:
        modes = [STRAIGHT] * len(pts)
    out = [(pts[0][0], pts[0][1], True)]
    straight = [] if legs is None else legs
    i, n = 0, len(pts)
    while i < n - 1:
        if modes[i] == CURVE and modes[i + 1] == CURVE:
            j = i + 1                       # end of this run of curve legs
            while j < n - 1 and modes[j + 1] == CURVE:
                j += 1
            if j == i + 1:
                # a "curve" through just two points is a straight line: one
                # leg, not a line chopped into spacing-sized pieces
                out.append((pts[j][0], pts[j][1], True))
                straight.append(True)
                i = j
                continue
            curve = resample(catmull_rom(pts[i:j + 1]), spacing)
            for x, y in curve[1:-1]:
                out.append((x, y, False))
                straight.append(False)
            # the run's last point: a corner unless the route ends there
            out.append((pts[j][0], pts[j][1], True))
            straight.append(False)
            i = j
        else:
            out.append((pts[i + 1][0], pts[i + 1][1], True))
            straight.append(True)
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


AUTONOMOUS_CPP = os.path.join(HERE, "..", "src", "autonomous.cpp")
AUTONOMOUS_HPP = os.path.join(HERE, "..", "include", "autonomous.hpp")


def auto_function_name(stem):
    """A C++ function name from a plan's file name: letters, digits and
    underscores, never starting with a digit."""
    name = re.sub(r"\W+", "_", stem).strip("_").lower() or "planned_auto"
    return "auto_" + name if name[0].isdigit() else name


def write_auto_function(name, blocks, start, cpp_path=AUTONOMOUS_CPP, hpp_path=AUTONOMOUS_HPP):
    """Write the plan into autonomous.cpp as `void name()` and declare it in
    autonomous.hpp. Saving the same plan again replaces its function -- it
    lives between BEGIN/END planner marker comments, so nothing else in the
    file is touched. A name already taken by hand-written code is never
    overwritten: the function gets a `_planned` suffix instead. It goes just
    above runAutonomous() (or at the end of the file). Returns the name
    actually used and whether an earlier version of it was replaced."""
    def read(path):
        with open(path, encoding="utf-8", newline="") as f:
            raw = f.read()
        crlf = "\r\n" in raw
        return raw.replace("\r\n", "\n"), crlf

    def write(path, text, crlf):
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write(text.replace("\n", "\r\n") if crlf else text)

    def block_re(n):
        return re.compile(rf"^// BEGIN planner: {re.escape(n)}\b.*?^// END planner: "
                          rf"{re.escape(n)}[^\n]*\n?", re.M | re.S)

    cpp, crlf = read(cpp_path)
    handwritten = re.compile(r"^[ \t]*(?:static\s+)?void\s+(\w+)\s*\(", re.M)
    others = block_re(name).sub("", cpp)          # the file without this plan's own copy
    taken = {m.group(1) for m in handwritten.finditer(others)}
    final = name
    while final in taken:
        final += "_planned"

    body = [f"// BEGIN planner: {final}  (tools/path_planner.py -- saving this plan again "
            "replaces everything up to END)",
            "// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.",
            f"void {final}() {{",
            f"    chassis.setPose({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f});  // start pose"]
    body += [f"    {block_code(b)}  // {i + 1}: {block_summary(b)}" for i, b in enumerate(blocks)]
    body += ["}", f"// END planner: {final}", ""]
    code = "\n".join(body)

    replaced = bool(block_re(final).search(cpp))
    if replaced:
        cpp = block_re(final).sub(lambda _m: code, cpp, count=1)
    else:
        lines = cpp.split("\n")
        at = next((i for i, ln in enumerate(lines) if re.match(r"\s*void\s+runAutonomous\s*\(", ln)),
                  None)
        if at is None:
            cpp = cpp.rstrip("\n") + "\n\n" + code
        else:
            while at > 0 and lines[at - 1].lstrip().startswith("//"):   # keep its comment with it
                at -= 1
            lines[at:at] = code.split("\n")      # code ends in "\n": one blank line after it
            cpp = "\n".join(lines)
    write(cpp_path, cpp, crlf)

    try:
        hpp, hcrlf = read(hpp_path)
    except OSError:
        return final, replaced
    if not re.search(rf"\bvoid\s+{re.escape(final)}\s*\(\s*\)\s*;", hpp):
        decl = f"void {final}();"
        lines = hpp.split("\n")
        at = next((i for i, ln in enumerate(lines)
                   if re.match(r"\s*void\s+runAutonomous\s*\(", ln)), None)
        if at is None:
            hpp = hpp.rstrip("\n") + "\n" + decl + "\n"
        else:
            lines.insert(at, decl)
            hpp = "\n".join(lines)
        write(hpp_path, hpp, hcrlf)
    return final, replaced


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


def swing_side(err, forwards=True):
    """Which side a swing locks for a turn of `err` degrees (positive =
    clockwise). Clockwise with the right side locked drives the left side
    forwards, so the robot swings forwards; locking the left instead drives
    the right side backwards -- a backward swing."""
    right = err > 0
    return "RIGHT" if right == bool(forwards) else "LEFT"


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


def sim_turn(robot, b, trace, targetHeading, swingSide=None, headings=None):
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
        if headings is not None:
            headings.append(robot.th)       # one per trace point: the footprint's rotation
    return t


def simulate(blocks, start):
    """Runs the block list through the simulator from `start` = (x, y,
    theta). Returns, per block, (endPose, shape):
        ("trace", [(x, y), ...])          the path the robot centre takes
        ("turn", x, y, fromHdg, toHdg)    an in-place turn (drawn as an arc)
        ("swing", x, y, fromHdg, toHdg, trace, headings)  a swing: arc + the centre's
                                          path, and the heading at each point of it
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
            heads = [th0]
            sim_turn(robot, b, trace, target, swingSide, heads)
            if swingSide:
                shape = ("swing", x0, y0, th0, robot.th, trace, heads)
            else:
                shape = ("turn", x0, y0, th0, robot.th)
        out.append((robot.pose(), shape))
    return out


# ─── The window ──────────────────────────────────────────────────────────────
class PlannerWindow(tk.Toplevel):
    def __init__(self, master, link):
        super().__init__(master)
        self.link = link
        self.planName = None           # file the plan was last saved to / loaded from
        self._retitle()
        self._applyTheme()
        self.blocks = []
        self.selected = None
        self.start = [0.0, 0.0, 0.0]   # start pose used to draw the route
        self.pickStart = False
        self.driveMode = False         # WASD driving on/off
        self.driveKeys = set()         # keys held right now
        self.driveLast = None          # last (l, r) sent, to avoid resending
        self.driveJob = None           # keepalive callback
        self.drawMode = False          # mouse path editor on/off
        self.waypoints = []            # control points: (x, y, CURVE/STRAIGHT[, heading])
        self.selWp = None              # the drawn point the heading buttons act on
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
        self._frameSim = None          # simulate() result pinned for the length of one paint
        self._lastState = None         # last robot state drawn, to skip no-ops
        self.startPlaced = False       # True once the robot has been put somewhere on the map
        self.mapPx, self.mapX0, self.mapY0 = MAP_PX, 0, 0   # the square map inside the canvas
        self._fieldSrc = None          # field.png at full size (Pillow only)
        self._relayoutJob = None
        self.sidebarShown = True
        self._livePending = False
        self.pathLegs = []             # per drawn leg: straight (True) or part of a curve
        self.aimPending = False        # a swing / turn just added in draw mode wants the next click
        self.pending = None            # (block, insert index): a motion being placed, not applied yet
        self.hoverSwing = None         # block index of the swing the mouse is over
        self._chips = []               # clickable shape options drawn beside a selected swing / turn
        self.undoStack = []            # plan snapshots taken before each applied path
        self.redoStack = []            # what Ctrl+Z took away: ("wp", point) / ("plan", snapshot)
        self.replaceOnApply = False    # True while re-shaping a traced route
        # Hiding the window must not leave the robot driving on.
        self.protocol("WM_DELETE_WINDOW", lambda: (self.stopDrive(), self.withdraw()))

        self._buildUi()
        self._bindKeys()
        self.redrawMap()
        self.refreshList()
        self._sizeWindow()

    def _sizeWindow(self):
        """Open at a size that fits the screen, taskbar and title bar included
        -- the side panel scrolls, so nothing needs the window to be tall --
        and let it shrink well below that."""
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w = min(1200, sw - 40)
        h = max(480, min(720, sh - 110))
        self.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 3)}")
        self.minsize(min(760, w), min(460, h))

    def _retitle(self):
        self.title("Path Planner -- V5" + (f"  --  {self.planName}" if self.planName else ""))

    def _applyTheme(self):
        """One dark look for every widget made after this, so the controls
        match the map instead of being default grey and white. Anything that
        sets its own colours (the run buttons, the map icons) still wins."""
        for pattern, value in (
                ("*Entry.background", INPUT_BG), ("*Entry.foreground", "#e6e6e6"),
                ("*Entry.insertBackground", "#e6e6e6"), ("*Entry.relief", "flat"),
                ("*Entry.highlightThickness", 1), ("*Entry.highlightBackground", "#3a3a3a"),
                ("*Entry.highlightColor", PATH_GREEN),
                ("*Button.background", "#3a3a3a"), ("*Button.foreground", "#e6e6e6"),
                ("*Button.activeBackground", "#4a4a4a"), ("*Button.activeForeground", "#ffffff"),
                ("*Button.relief", "flat"), ("*Button.borderWidth", 0),
                ("*Button.padX", 8), ("*Button.padY", 3), ("*Button.cursor", "hand2"),
                ("*Listbox.background", INPUT_BG), ("*Listbox.foreground", "#e6e6e6"),
                ("*Listbox.relief", "flat"), ("*Listbox.highlightThickness", 0),
                ("*Scale.background", PANEL_BG), ("*Scale.troughColor", INPUT_BG),
                ("*Scrollbar.background", "#3a3a3a"), ("*Scrollbar.troughColor", INPUT_BG),
                ("*Label.background", PANEL_BG), ("*Label.foreground", "#cfcfcf"),
                ("*LabelFrame.background", PANEL_BG), ("*LabelFrame.foreground", "#cfcfcf"),
                ("*Frame.background", PANEL_BG)):
            # scoped to this window's path, so the main remote window keeps its own look
            self.option_add(f"{self}{pattern}", value)

    # -- map icons --
    # Drawn as canvas items, not child widgets: a widget placed over a canvas
    # is repainted along with it, and on Windows that shows up as the buttons
    # flashing every time the map redraws. Clicks and hovers are matched by
    # position in onMapPress / onMapMotion.
    ICON = 40
    ICON_GAP = 6
    ICON_MARGIN = 56        # dark margin kept clear each side of the map, for the icons
    ICON_BG, ICON_HOVER, ICON_ON = "#383838", "#4a4a4a", "#5a5a5a"
    SIDE_W = 470            # width of the side panel

    def _buildMapButtons(self):
        self.icons = {}
        self._hoverIcon = None

        def icon(name, glyph, cmd, tip):
            rect = self.map.create_rectangle(0, 0, 1, 1, fill=self.ICON_BG, outline="",
                                             tags=("ui",))
            text = self.map.create_text(0, 0, text=glyph, fill="#e6e6e6",
                                        font=("Segoe UI Symbol", 15), tags=("ui",))
            self.icons[name] = dict(rect=rect, text=text, cmd=cmd, tip=tip, box=None, on=False)
        icon("panel", "☰", self.toggleSidebar, "Hide / show the side panel  (H)")
        icon("draw", "✎", self.toggleDraw, "Draw a path  (D)")
        icon("apply", "✓", self.applyDrawnPath, "Apply the drawn path to the blocks  (Enter)")
        icon("clear", "✕", self.clearWaypoints, "Clear the points being drawn")
        icon("redo", "↷", self.redoPath, "Redo  (Ctrl+Y)")
        icon("undo", "↶", self.undoPath, "Undo  (Ctrl+Z)")

    def _placeIcons(self):
        """Lay the icons out in the margins either side of the square map:
        panel toggle top-left, draw tools down the right, undo / redo at the
        bottom right -- the arrangement path.jerryio uses."""
        w = self.map.winfo_width() or 600
        h = self.map.winfo_height() or MAP_PX
        s, g, m = self.ICON, self.ICON_GAP, 8
        right = w - m - s
        spots = {"panel": (m, m), "draw": (right, m), "apply": (right, m + s + g),
                 "clear": (right, m + 2 * (s + g)), "redo": (right, h - m - 2 * s - g),
                 "undo": (right, h - m - s)}
        for name, (x, y) in spots.items():
            ic = self.icons[name]
            ic["box"] = (x, y, x + s, y + s)
            self.map.coords(ic["rect"], x, y, x + s, y + s)
            self.map.coords(ic["text"], x + s / 2, y + s / 2)
        self.map.tag_raise("ui")

    def _iconAt(self, ex, ey):
        for name, ic in self.icons.items():
            x0, y0, x1, y1 = ic["box"] or (0, 0, -1, -1)
            if x0 <= ex <= x1 and y0 <= ey <= y1:
                return name
        return None

    def _paintIcon(self, name):
        ic = self.icons[name]
        fill = self.ICON_ON if ic["on"] else (self.ICON_HOVER if name == self._hoverIcon
                                              else self.ICON_BG)
        self.map.itemconfigure(ic["rect"], fill=fill)

    def _setIconActive(self, name, on):
        self.icons[name]["on"] = on
        self._paintIcon(name)

    def _hoverOver(self, name):
        if name == self._hoverIcon:
            return
        old, self._hoverIcon = self._hoverIcon, name
        for n in (old, name):
            if n:
                self._paintIcon(n)

    # -- layout --
    def _buildUi(self):
        self.configure(bg=PANEL_BG)
        top = tk.Frame(self, bg=PANEL_BG)
        top.pack(side="top", fill="both", expand=True)

        # The side panel is packed first, so when the window is tight it is the
        # map that gives way, never a button; the map then takes what's left.
        self.side = tk.Frame(top, bg=PANEL_BG, width=self.SIDE_W)
        self.side.pack(side="right", fill="y", padx=(0, 8), pady=8)
        self.side.pack_propagate(False)
        self.mapFrame = tk.Frame(top, bg=PANEL_BG)
        self.mapFrame.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        self._buildMapArea(self.mapFrame)
        self._buildSidePanel(self.side)
        self._loadFieldImage()

    def _buildMapArea(self, mapFrame):
        # The canvas asks for little and takes whatever the window gives it;
        # the square map is fitted inside it (see _relayout).
        self.map = tk.Canvas(mapFrame, width=320, height=320, bg=FIELD_BG, highlightthickness=0)
        self.map.pack(fill="both", expand=True)
        self._buildMapButtons()
        self.map.bind("<Configure>", self._onMapConfigure)
        self.map.bind("<Button-1>", self.onMapPress)
        self.map.bind("<B1-Motion>", self.onMapDrag)
        self.map.bind("<ButtonRelease-1>", self.onMapRelease)
        self.map.bind("<Button-3>", self.onMapRightClick)
        self.map.bind("<Shift-Button-1>", lambda e: self.removeWaypointAt(e.x, e.y))
        self.map.bind("<Button-2>", lambda e: self.removeWaypointAt(e.x, e.y))
        self.map.bind("<Motion>", self.onMapMotion)
        self.map.bind("<Leave>", self._onMapLeave)
        self.mapStatus = tk.Label(mapFrame, text="", anchor="w", fg="#999", bg=PANEL_BG,
                                  font=("Consolas", 9))
        self.mapStatus.pack(fill="x", pady=(4, 0))

        # Mouse-path tools under the map: everything the icons do, with labels,
        # plus the options that don't have an icon.
        row = tk.Frame(mapFrame, bg=PANEL_BG)
        row.pack(fill="x", pady=(6, 0))
        self.drawBtnRow = tk.Button(row, text="Draw  (D)", command=self.toggleDraw)
        self.drawBtnRow.pack(side="left")
        for text, cmd in (("Apply  (Enter)", self.applyAction), ("Undo", self.undoPath),
                          ("Redo", self.redoPath), ("Clear points", self.clearWaypoints)):
            tk.Button(row, text=text, command=cmd).pack(side="left", padx=(6, 0))
        row2 = tk.Frame(mapFrame, bg=PANEL_BG)
        row2.pack(fill="x", pady=(6, 0))
        self.smoothVar = tk.BooleanVar(value=True)
        tk.Checkbutton(row2, text="smooth", variable=self.smoothVar, bg=PANEL_BG, fg="#ddd",
                       selectcolor=INPUT_BG, activebackground=PANEL_BG,
                       activeforeground="#ddd", command=self.redrawMap).pack(side="left")
        tk.Label(row2, text="spacing", fg="#888", bg=PANEL_BG).pack(side="left", padx=(8, 0))
        self.spacingVar = tk.StringVar(value="12")
        e = tk.Entry(row2, textvariable=self.spacingVar, width=5, font=("Consolas", 10))
        e.pack(side="left", padx=(2, 8))
        e.bind("<Return>", lambda _e: self.redrawMap())
        e.bind("<FocusOut>", lambda _e: self.redrawMap())
        self.reverseVar = tk.BooleanVar(value=False)
        tk.Checkbutton(row2, text="backwards", variable=self.reverseVar, bg=PANEL_BG, fg="#ddd",
                       selectcolor=INPUT_BG, activebackground=PANEL_BG,
                       activeforeground="#ddd").pack(side="left")
        tk.Button(row2, text="Trace blocks", command=self.tracePointsFromBlocks) \
            .pack(side="left", padx=(12, 0))
        tk.Button(row2, text="Clear map", command=self.clearMap).pack(side="left", padx=(6, 0))
        self.drawInfo = tk.Label(mapFrame, text="", fg="#777", bg=PANEL_BG, font=("Segoe UI", 8),
                                 anchor="w")
        self.drawInfo.pack(fill="x", pady=(4, 0))

    def _buildSidePanel(self, side):
        # header: brain status and the hide button -- always on screen
        hdr = tk.Frame(side, bg=PANEL_BG)
        hdr.pack(side="top", fill="x", pady=(0, 6))
        tk.Button(hdr, text="Hide panel  (H) ▸", command=self.toggleSidebar) \
            .pack(side="right")
        self.brainStatus = tk.Label(hdr, text="brain: waiting...", anchor="w", fg="#00F0FF",
                                    bg=PANEL_BG, font=("Segoe UI", 10, "bold"))
        self.brainStatus.pack(side="left", fill="x", expand=True)

        # footer: run / export / save -- packed before the scrolling part so it
        # always keeps its space, however short the window is
        foot = tk.Frame(side, bg=PANEL_BG)
        foot.pack(side="bottom", fill="x", pady=(8, 0))
        f1 = tk.Frame(foot, bg=PANEL_BG)
        f1.pack(fill="x")
        for text, cmd, color in (("Run all  (F5)", self.runAll, "#2ECC71"),
                                 ("Run selected  (F6)", self.runSelected, "#00F0FF"),
                                 ("Stop  (Esc)", self.stop, "#FF6B81")):
            tk.Button(f1, text=text, command=cmd, fg="black", bg=color,
                      activebackground=color, activeforeground="black") \
                .pack(side="left", fill="x", expand=True, padx=(0, 4))
        f2 = tk.Frame(foot, bg=PANEL_BG)
        f2.pack(fill="x", pady=(6, 0))
        tk.Button(f2, text="Save → autonomous.cpp  (Ctrl+S)", command=self.savePlan) \
            .pack(side="left", fill="x", expand=True, padx=(0, 4))
        tk.Button(f2, text="Load", command=self.loadPlan).pack(side="left", padx=(0, 4))
        tk.Button(f2, text="Export code", command=self.exportCode).pack(side="left")
        f3 = tk.Frame(foot, bg=PANEL_BG)
        f3.pack(fill="x", pady=(4, 0))
        self.calibrateVar = tk.BooleanVar(value=True)
        tk.Checkbutton(f3, text="calibrate first (export)", variable=self.calibrateVar,
                       bg=PANEL_BG, fg="#ddd", selectcolor=INPUT_BG, activebackground=PANEL_BG,
                       activeforeground="#ddd").pack(side="left")
        self.setStartVar = tk.BooleanVar(value=True)
        tk.Checkbutton(f3, text="Run all starts from the start pose", variable=self.setStartVar,
                       bg=PANEL_BG, fg="#ddd", selectcolor=INPUT_BG, activebackground=PANEL_BG,
                       activeforeground="#ddd").pack(side="left", padx=(10, 0))

        # the scrolling middle
        body = tk.Frame(side, bg=PANEL_BG)
        body.pack(side="top", fill="both", expand=True)
        sc = tk.Canvas(body, bg=PANEL_BG, highlightthickness=0)
        vsb = tk.Scrollbar(body, command=sc.yview)
        sc.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        sc.pack(side="left", fill="both", expand=True)
        inner = tk.Frame(sc, bg=PANEL_BG)
        win = sc.create_window(0, 0, window=inner, anchor="nw")
        inner.bind("<Configure>", lambda e: sc.configure(scrollregion=sc.bbox("all")))
        sc.bind("<Configure>", lambda e: sc.itemconfigure(win, width=e.width))
        self._sideCanvas = sc

        def wheel(e):
            sc.yview_scroll(-1 if e.delta > 0 else 1, "units")
        saved = {}

        def enter(_e):
            saved["old"] = self.bind_all("<MouseWheel>")
            self.bind_all("<MouseWheel>", wheel)

        def leave(_e):
            self.unbind_all("<MouseWheel>")
            if saved.get("old"):
                self.bind_all("<MouseWheel>", saved["old"])
        side.bind("<Enter>", enter)
        side.bind("<Leave>", leave)

        def section(title):
            f = tk.LabelFrame(inner, text=f" {title} ", fg="#9a9a9a", bg=PANEL_BG,
                              font=("Segoe UI", 9), bd=1, relief="groove")
            f.pack(fill="x", padx=(0, 6), pady=(0, 8))
            return f

        def row(parent, pady=(3, 3)):
            r = tk.Frame(parent, bg=PANEL_BG)
            r.pack(fill="x", padx=6, pady=pady)
            return r

        def labelled(parent, label, var, width=7, first=False):
            tk.Label(parent, text=label, fg="#8a8a8a", bg=PANEL_BG) \
                .pack(side="left", padx=(0 if first else 8, 2))
            e = tk.Entry(parent, textvariable=var, width=width, font=("Consolas", 10))
            e.pack(side="left")
            return e

        # robot: where it starts, and how big it is ------------------------
        rb = section("robot")
        r1 = row(rb)
        self.startVars = []
        for i, lab in enumerate(("x", "y", "hdg")):
            v = tk.StringVar(value="0")
            e = labelled(r1, lab, v, 6, first=(i == 0))
            e.bind("<Return>", lambda _e: self.applyStartPose())
            self.startVars.append(v)
        tk.Button(r1, text="Send to robot", command=self.sendStartPose).pack(side="right")
        r2 = row(rb)
        self.pickBtn = tk.Button(r2, text="Pick start", command=self.togglePickStart)
        self.pickBtn.pack(side="left")
        tk.Button(r2, text="Use robot's pose", command=self.useRobotPose) \
            .pack(side="left", padx=6)
        tk.Button(r2, text="Reset robot", command=self.resetRobot, fg="#FF6B81") \
            .pack(side="left")
        # Footprint: drawn as a bounding box at the start, the end of the route,
        # the selected block and the live robot. length runs along the heading
        # (front to back), width across it.
        r3 = row(rb, pady=(3, 6))
        tk.Label(r3, text="size (in)", fg="#ccc", bg=PANEL_BG).pack(side="left")
        self.robotLenVar = tk.StringVar(value=fmt(DEFAULT_ROBOT_IN))
        self.robotWidVar = tk.StringVar(value=fmt(DEFAULT_ROBOT_IN))
        for lab, var in (("length", self.robotLenVar), ("width", self.robotWidVar)):
            e = labelled(r3, lab, var, 6)
            e.bind("<Return>", lambda _e: self.redrawMap())
            e.bind("<FocusOut>", lambda _e: self.redrawMap())

        # selected route point, like path.jerryio's "Control" panel -----------
        pt = section("selected point")
        r4 = row(pt, pady=(3, 6))
        self.ptVars = []
        for i, lab in enumerate(("x", "y", "heading")):
            v = tk.StringVar()
            e = labelled(r4, lab, v, 7 if lab == "heading" else 6, first=(i == 0))
            e.bind("<Return>", lambda _e: self.applyPointPanel())
            e.bind("<FocusOut>", lambda _e: self.applyPointPanel())
            self.ptVars.append(v)
        for text, delta in (("↻", self.HEADING_STEP), ("↺", -self.HEADING_STEP)):
            tk.Button(r4, text=text, width=3, font=("Segoe UI Symbol", 11),
                      command=lambda d=delta: self.rotatePoint(d)).pack(side="right", padx=(4, 0))

        # blocks -------------------------------------------------------------
        bl = section("blocks")
        lf = tk.Frame(bl, bg=PANEL_BG)
        lf.pack(fill="x", padx=6, pady=6)
        self.listbox = tk.Listbox(lf, font=("Consolas", 10), bg=INPUT_BG, fg="#ddd",
                                  selectbackground="#3d4f2a", selectforeground="#e8ffc8",
                                  activestyle="none", height=9, exportselection=False)
        self.listbox.pack(side="left", fill="both", expand=True)
        self.listbox.bind("<<ListboxSelect>>", self.onSelect)

        def listWheel(e):
            # over the blocks list the wheel scrolls only the list -- "break"
            # keeps it from also reaching the sidebar's own wheel binding
            self.listbox.yview_scroll(-1 if e.delta > 0 else 1, "units")
            return "break"
        self.listbox.bind("<MouseWheel>", listWheel)
        sb = tk.Scrollbar(lf, command=self.listbox.yview)
        sb.pack(side="left", fill="y")
        self.listbox.config(yscrollcommand=sb.set)
        ops = tk.Frame(lf, bg=PANEL_BG)
        ops.pack(side="left", fill="y", padx=(6, 0))
        for text, cmd in (("Up", lambda: self.moveBlock(-1)), ("Down", lambda: self.moveBlock(1)),
                          ("Duplicate", self.duplicateBlock), ("Delete", self.deleteBlock),
                          ("Clear all", self.clearBlocks)):
            tk.Button(ops, text=text, width=9, command=cmd).pack(pady=2)

        # parameters of the selected block --------------------------------
        self.paramFrame = tk.LabelFrame(inner, text=" parameters ", fg="#9a9a9a", bg=PANEL_BG,
                                        font=("Segoe UI", 9), bd=1, relief="groove")
        self.paramFrame.pack(fill="x", padx=(0, 6), pady=(0, 4))
        self.paramWidgets = {}
        self.codeLabel = tk.Label(inner, text="", anchor="w", justify="left", fg="#2ECC71",
                                  bg=PANEL_BG, font=("Consolas", 9), wraplength=430)
        self.codeLabel.pack(fill="x", pady=(0, 8))

        # add a block ----------------------------------------------------------
        mv = section("add block")
        grid = tk.Frame(mv, bg=PANEL_BG)
        grid.pack(fill="x", padx=6, pady=6)
        for i, kind in enumerate(KINDS):
            tk.Button(grid, text=kind, command=lambda k=kind: self.pickMotion(k)) \
                .grid(row=i // 3, column=i % 3, padx=2, pady=2, sticky="ew")
        for c in range(3):
            grid.columnconfigure(c, weight=1, uniform="kinds")

        # driving ----------------------------------------------------------------
        dr = section("drive")
        drow = row(dr, pady=(4, 6))
        self.driveBtn = tk.Button(drow, text="Drive (WASD)", command=self.toggleDrive)
        self.driveBtn.pack(side="left")
        tk.Label(drow, text="power", fg="#888", bg=PANEL_BG).pack(side="left", padx=(8, 2))
        self.powerVar = tk.IntVar(value=70)
        tk.Scale(drow, from_=10, to=127, orient="horizontal", variable=self.powerVar,
                 length=110, bg=PANEL_BG, fg="#ddd", highlightthickness=0,
                 troughcolor=INPUT_BG).pack(side="left")
        self.driveStatus = tk.Label(drow, text="off", fg="#777", bg=PANEL_BG,
                                    font=("Consolas", 9))
        self.driveStatus.pack(side="left", padx=8)

        self.hint = tk.Label(inner, text="Drag the robot to place it, any route point to move "
                             "it, and the knob on a point's line to turn it.  D = draw a path "
                             "(click to add points, Enter to apply).  H = hide this panel.  "
                             "Ctrl+Z / Ctrl+Y undo / redo.  Delete = remove block.  Ctrl+S saves "
                             "the plan and writes it into autonomous.cpp.  Ctrl+O load.  "
                             "Ctrl+E export.", anchor="w", justify="left", wraplength=430,
                             fg="#777", bg=PANEL_BG, font=("Segoe UI", 8))
        self.hint.pack(fill="x", pady=(0, 8))

    def toggleSidebar(self):
        """Hide the whole right-hand panel so only the map shows (and bring it
        back). The map grows to fill the window when the panel goes."""
        self.sidebarShown = not self.sidebarShown
        if self.sidebarShown:
            self.side.pack(side="right", fill="y", padx=(0, 8), pady=8, before=self.mapFrame)
        else:
            self.side.pack_forget()
        self._setIconActive("panel", not self.sidebarShown)

    def _bindKeys(self):
        self.bind("<F5>", lambda e: self.runAll())
        self.bind("<F6>", lambda e: self.runSelected())
        self.bind("<Escape>", lambda e: self.cancelPending() if self.pending
                  else (self.stopDrive(), self.stop()))
        self.bind("<Control-s>", lambda e: self.savePlan())
        self.bind("<Control-o>", lambda e: self.loadPlan())
        self.bind("<Control-e>", lambda e: self.exportCode())
        self.listbox.bind("<Delete>", lambda e: self.deleteBlock())
        self.bind("<Delete>", lambda e: None if self._typing() or self.drawMode
                  else self.deleteBlock())
        # ...and the draw-path keys, which must not fire while a number is
        # being typed into one of the entries (Tk sends the key to the
        # widget first, then here).
        # W/A/S/D drive the robot while drive mode is on; "d" only toggles
        # the path editor when it isn't (it would otherwise do both).
        self.bind("<KeyPress>", self.onDriveKeyPress, add="+")
        self.bind("<KeyRelease>", self.onDriveKeyRelease, add="+")
        self.bind("<FocusOut>", lambda e: self.stopDrive() if self.driveMode else None)
        self.bind("<d>", lambda e: None if self._typing() or self.driveMode else self.toggleDraw())
        self.bind("<D>", lambda e: None if self._typing() or self.driveMode else self.toggleDraw())
        self.bind("<h>", lambda e: None if self._typing() else self.toggleSidebar())
        self.bind("<H>", lambda e: None if self._typing() else self.toggleSidebar())
        self.bind("<Control-z>", lambda e: None if self._typing() else self.undoPath())
        self.bind("<Control-y>", lambda e: None if self._typing() else self.redoPath())
        self.bind("<Control-Z>", lambda e: None if self._typing() else self.redoPath())
        self.bind("<BackSpace>", lambda e: None if self._typing() else self.undoWaypoint())
        self.bind("<Return>", lambda e: self.applyAction()
                  if (self.drawMode or self.pending) and not self._typing() else None)

    def _typing(self):
        """True when the keyboard focus is in a text entry / combobox."""
        w = self.focus_get()
        return isinstance(w, (tk.Entry, tk.Text)) or isinstance(w, ttk.Combobox) or \
            isinstance(w, ttk.Entry)

    def _loadFieldImage(self):
        """tools/field.png, scaled to the map. With Pillow it is resized to
        whatever size the map has (see _relayout); without it Tk can only
        subsample it, so the map stays at its fixed size."""
        if not os.path.exists(FIELD_PNG):
            return
        try:
            from PIL import Image
            self._fieldSrc = Image.open(FIELD_PNG).convert("RGB")
            self._scaleField()
        except ImportError:
            try:
                img = tk.PhotoImage(file=FIELD_PNG)
                f = max(1, round(img.width() / MAP_PX))
                self.fieldImage = img.subsample(f, f)
            except tk.TclError:
                self.fieldImage = None
        except Exception:
            self._fieldSrc = None
            self.fieldImage = None

    def _scaleField(self):
        if self._fieldSrc is None:
            return
        from PIL import Image, ImageTk
        self.fieldImage = ImageTk.PhotoImage(
            self._fieldSrc.resize((self.mapPx, self.mapPx), Image.LANCZOS))

    def _onMapConfigure(self, _e):
        """The canvas changed size (window resized, panel hidden / shown):
        refit the map once things settle, not on every step of a drag."""
        if self._relayoutJob:
            self.after_cancel(self._relayoutJob)
        self._relayoutJob = self.after(60, self._relayout)

    def _relayout(self):
        """Fit the square map in the middle of the canvas, as large as the
        window allows while keeping a margin each side for the icons."""
        self._relayoutJob = None
        w, h = self.map.winfo_width(), self.map.winfo_height()
        if w < 60 or h < 60:
            return
        side = MAP_PX if self._fieldSrc is None and self.fieldImage else \
            int(max(220, min(h, w - 2 * self.ICON_MARGIN)))
        x0, y0 = (w - side) // 2, (h - side) // 2
        changed = (side, x0, y0) != (self.mapPx, self.mapX0, self.mapY0)
        self.mapPx, self.mapX0, self.mapY0 = side, x0, y0
        self._placeIcons()
        if changed:
            self._scaleField()
            self.map.delete("field")
            self.redrawMap()

    # -- coordinates --
    @property
    def ppi(self):
        """Map pixels per field inch."""
        return self.mapPx / FIELD_IN

    def toMap(self, x, y):
        return (self.mapX0 + self.mapPx / 2 + x * self.ppi,
                self.mapY0 + self.mapPx / 2 - y * self.ppi)

    def toField(self, px, py):
        return ((px - self.mapX0 - self.mapPx / 2) / self.ppi,
                (self.mapY0 + self.mapPx / 2 - py) / self.ppi)

    # -- map drawing --
    def simulation(self):
        """simulate() for the current blocks, cached. The map is redrawn
        on every state the robot sends (20 a second), and re-running the
        whole motion simulation each time is what made it crawl and
        flicker -- the route only changes when the blocks or the start
        pose do."""
        if self._frameSim is not None:      # mid-paint: nothing changes until it ends
            return self._frameSim
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
        """Repaint the route layer. The field render is created once and left
        alone -- only the items drawn on top of it ("dyn") are replaced, all
        inside this one callback, so the map never shows a half-drawn frame
        and the big image isn't rebuilt every time something moves."""
        self._redrawPending = False
        c = self.map
        if not c.find_withtag("field"):
            # Clean like path.jerryio: just the field on a flat dark ground --
            # no grid, no wash over the render.
            if self.fieldImage:
                c.create_image(self.mapX0, self.mapY0, anchor="nw", image=self.fieldImage,
                               tags="field")
            else:
                c.create_rectangle(self.mapX0, self.mapY0, self.mapX0 + self.mapPx,
                                   self.mapY0 + self.mapPx, fill="#222222", outline="",
                                   tags="field")
            c.tag_lower("field")
        c.delete("dyn")
        c.delete("live")
        # The simulation is looked up once per paint: pointHeading() asks for
        # it for every point, and the cache key is a repr of every block.
        self._frameSim = self.simulation()
        try:
            self._paintMap()
        finally:
            self._frameSim = None
            self._tagNew("dyn")
        self._paintLive()
        self._tagNew("live")
        c.tag_raise("live")
        c.tag_raise("ui")

    def _tagNew(self, tag):
        """Tag the items just drawn: the ones not yet in any layer."""
        c = self.map
        for item in c.find_withtag("!field&&!ui&&!dyn&&!live"):
            c.addtag_withtag(tag, item)

    def redrawLive(self):
        """Repaint only the live robot and its trail. The robot reports 20
        times a second; redoing the whole route for each would be most of
        the flicker, so those updates touch just this small layer, at most
        about 25 times a second."""
        if self._livePending or self._redrawPending:
            return          # a repaint is already on its way, and includes this
        self._livePending = True
        self.after(40, self._drawLive)

    def _drawLive(self):
        self._livePending = False
        if self._redrawPending:
            return
        c = self.map
        c.delete("live")
        self._paintLive()
        self._tagNew("live")
        c.tag_raise("live")
        c.tag_raise("ui")

    # A step this long between two trail samples is not driving: the robot's
    # position was reset (setPose, "Send to robot") or the odometry glitched.
    # Joining the two would draw a straight line clean across the field.
    TRAIL_JUMP_IN = 18.0

    def trailRuns(self):
        """The robot's trail as separate polylines, split wherever it jumps or
        has a point that can't be on the field, and ending at the live pose
        when that is just a short step on."""
        limit = FIELD_IN / 2 + 12
        runs, run = [], []
        for pt in self.robotPath:
            x, y = pt
            if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > limit or abs(y) > limit:
                if len(run) > 1:
                    runs.append(run)
                run = []
                continue
            if run and math.hypot(x - run[-1][0], y - run[-1][1]) > self.TRAIL_JUMP_IN:
                if len(run) > 1:
                    runs.append(run)
                run = []
            run.append((x, y))
        if run and self.robotPose and math.hypot(self.robotPose[0] - run[-1][0],
                                                 self.robotPose[1] - run[-1][1])                 <= self.TRAIL_JUMP_IN:
            run.append((self.robotPose[0], self.robotPose[1]))   # the thinned trail stops short
        if len(run) > 1:
            runs.append(run)
        return runs

    def _paintLive(self):
        # robot: driven path then live pose
        for run in self.trailRuns():
            flat = [v for x, y in run for v in self.toMap(x, y)]
            self.drawPolyline(flat, "#2EFF8C", 2)
        if self.robotPose:
            self.drawRobotMarker(self.robotPose, "#2EFF8C", "", robot=True)

    def _paintMap(self):
        c = self.map
        # planned route: every block's movement, drawn from where the
        # previous block left the robot, so the whole stack reads as one
        # route. All in the path green, the selected block's leg brighter.
        sim = self.simulation()
        self.drawStartRobot()
        # footprint boxes only at the ends of the route and the selected
        # block -- one per block would bury a drawn path
        lastMove = max((i for i, b in enumerate(self.blocks)
                        if b["kind"] in MOTIONS or b["kind"] == "setPose"), default=None)
        for i, (b, (pose, shape)) in enumerate(zip(self.blocks, sim)):
            sel = (i == self.selected)
            col = PATH_SEL if sel else PATH_GREEN
            w = 4 if sel else 3
            dash = (6, 3) if not b.get("forwards", True) else None
            if shape and shape[0] == "trace":
                self.drawTrace(shape[1], col, w, dash, arrow=False)
            elif shape and shape[0] == "swing":     # just the line the robot's centre takes
                self.drawTrace(shape[5], col, w, None, arrow=False)
            elif shape and shape[0] == "turn":
                # no arc for an applied turn: they pile up along a route. Only the
                # selected turnToPoint shows where it aims.
                _, x, y, a0, a1 = shape[:5]
                if sel and b["kind"] == "turnToPoint":
                    x0, y0 = self.toMap(x, y)
                    x1, y1 = self.toMap(b["x"], b["y"])
                    c.create_line(x0, y0, x1, y1, fill=col, width=1, dash=(2, 4))
                    c.create_oval(x1 - 4, y1 - 4, x1 + 4, y1 + 4, outline=col)
            # a marker only where it means something: the selected block and
            # the end of the route (the flag), not one per block
            swing = shape and shape[0] == "swing"
            if swing and (sel or i == self.hoverSwing) and i != lastMove:
                self.drawSwingEnd(shape, PATH_SEL)      # where the swing leaves the robot
            elif sel and not swing and (b["kind"] in MOTIONS or b["kind"] == "setPose"):
                self.drawRobotBox(pose, PURPLE, width=1, dash=(4, 3))
            if i == lastMove:
                self.drawRobotBox(pose, END_BOX, width=2)

        # The route's points, always visible like path.jerryio's: a disc on
        # every block's target, drawn after all the lines so none hides one.
        # The selected block's is larger; the last one carries the flag.
        for i, b in enumerate(self.blocks):
            if b["kind"] in ("moveToPoint", "moveToPose"):
                r = 11 if i == self.selected else 8
                self.drawPoint(b["x"], b["y"], PURPLE, r=r)
                self.drawHeadingLine(i, r)
            elif b["kind"].startswith("swing"):     # the point the swing ends on
                r = 11 if i == self.selected else 8
                self.drawPoint(*self.pointPos(i), PURPLE, r=r)
                self.drawHeadingLine(i, r)
        ends = [b for b in self.blocks if b["kind"] in ("moveToPoint", "moveToPose")]
        if ends:
            self.drawFlag(ends[-1]["x"], ends[-1]["y"])
        self._chips = []
        if self.pending:
            self._paintPending()
        else:
            sel = self.selected
            if sel is not None and sel < len(self.blocks) \
                    and self.blocks[sel]["kind"] == "moveToPose":
                self.drawLeadHandle(self.blocks[sel], self.poseBefore(sel), PATH_SEL)
        if not self.pending and self.aimVisible():
            i = self.selected
            b = self.blocks[i]
            if b["kind"] in self.AIM_KINDS:
                self.drawAimLine(i, self.poseBefore(i))
            else:                       # turn / swing to a point: show its target
                self.drawPoint(b["x"], b["y"], PATH_SEL, r=8)
            self.drawShapeChips(b, self.poseBefore(i))

        self.drawWaypoints()

    # -- mouse path editor --
    def spacing(self):
        try:
            return max(2.0, float(self.spacingVar.get()))
        except (ValueError, tk.TclError):
            return 12.0

    def pathPoints(self):
        """The drawn route as (x, y, stop) points -- see path_points()."""
        self.pathLegs = []
        return path_points(self.effectiveWaypoints(), self.spacing(), self.smoothVar.get(),
                           legs=self.pathLegs)

    def drawWaypoints(self):
        """Control points and the route through them, live while drawing."""
        if not self.waypoints:
            if self.drawMode:
                self.drawInfo.config(text="click the map to drop points" + (
                    "  (continues from the end of the route)" if self.anchor() else ""))
            return
        c = self.map
        pts = self.pathPoints()
        if len(pts) > 1:
            flat = [v for x, y, _ in pts for v in self.toMap(x, y)]
            self.drawPolyline(flat, PATH_GREEN, 3,
                              dash=(7, 4) if self.reverseVar.get() else None)
        for i, w in enumerate(self.waypoints):
            px, py = self.toMap(w[0], w[1])
            straight = len(w) > 2 and w[2] == STRAIGHT
            r = 11
            # translucent purple points like path.jerryio; a hard (straight)
            # corner is a square instead of a disc
            if straight:
                c.create_rectangle(px - r, py - r, px + r, py + r, fill=POINT_FILL,
                                   outline=PURPLE, width=1)
            else:
                self.drawPoint(w[0], w[1], PURPLE, r)
            c.create_text(px, py, text=str(i + 1), fill="#f2f2f2", font=("Segoe UI", 7, "bold"))
        self.drawWaypointHeadings()
        lx, ly = self.waypoints[-1][0], self.waypoints[-1][1]
        if len(pts) > 1:
            # where the robot would end up, facing along the last leg
            hdg = heading_to(pts[-2][0], pts[-2][1], pts[-1][0], pts[-1][1],
                             not self.reverseVar.get())
            self.drawRobotBox((pts[-1][0], pts[-1][1], hdg), END_BOX, width=2)
        self.drawFlag(lx, ly)
        curves = sum(1 for w in self.waypoints if len(w) < 3 or w[2] == CURVE)
        dist = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(pts, pts[1:]))
        shape = ("turnToHeading + moveToPoint per leg" if not self.smoothVar.get()
                 else f"{len(pts) - 1} moveToPoint")
        self.drawInfo.config(
            text=f"{len(self.waypoints)} points ({curves} curve, "
                 f"{len(self.waypoints) - curves} straight)  ->  {shape}  ({dist:.0f} in)")

    def wpTravelHeading(self, i):
        """The way the robot faces leaving drawn point i along the route
        (the last point: arriving at it), when no heading was chosen."""
        fwd = not self.reverseVar.get()
        wp = self.waypoints
        if i + 1 < len(wp):
            return heading_to(wp[i][0], wp[i][1], wp[i + 1][0], wp[i + 1][1], fwd)
        if i > 0:
            return heading_to(wp[i - 1][0], wp[i - 1][1], wp[i][0], wp[i][1], fwd)
        end = self.anchor()
        return end[2] if end is not None else 0.0

    def wpHeading(self, i):
        """(heading, set) of drawn point i: the one chosen with the heading
        buttons, else the direction of travel."""
        w = self.waypoints[i]
        if len(w) > 3 and w[3] is not None:
            return w[3], True
        return self.wpTravelHeading(i), False

    def setWpHeading(self, i, theta):
        w = self.waypoints[i]
        self.waypoints[i] = (w[0], w[1], w[2] if len(w) > 2 else CURVE,
                             round(theta % 360.0, 1))

    def curWp(self):
        """Index of the drawn point the heading controls act on, or None."""
        i = self.selWp
        return i if self.drawMode and i is not None and i < len(self.waypoints) else None

    def drawWaypointHeadings(self):
        """A heading line out of every drawn point, as path.jerryio does. One
        chosen with the heading buttons is solid with an arrowhead; the rest
        are dashed and only show the direction of travel."""
        for i, w in enumerate(self.waypoints):
            h, isSet = self.wpHeading(i)
            px, py = self.toMap(w[0], w[1])
            rad = math.radians(h)
            kx, ky = px + math.sin(rad) * self.HEAD_LEN, py - math.cos(rad) * self.HEAD_LEN
            sel = i == self.selWp
            self.map.create_line(px, py, kx, ky, fill="#d6c8ff" if isSet or sel else "#b3a6e6",
                                 width=2, arrow="last")
            if sel:
                self.map.create_oval(px - 13, py - 13, px + 13, py + 13, outline="#d6c8ff",
                                     width=2)

    def routeEnd(self):
        """(x, y, heading) the existing blocks leave the robot at, or None
        with no blocks. The position is the last block's *target* point,
        not where the simulated robot happened to stop, so a path drawn from
        here joins the line already on the map with no gap."""
        if not self.blocks:
            return None
        sim = self.simulation()
        if not sim:
            return None
        x, y, th = sim[-1][0]
        for b in reversed(self.blocks):
            if b["kind"].startswith("swing"):
                break           # a swing moves the robot: the route ends where it does
            if b["kind"] in ("moveToPoint", "moveToPose", "setPose"):
                return (b["x"], b["y"], th)
        return (x, y, th)

    def anchor(self):
        """The pose a drawn path continues from: where the blocks already in
        the plan leave the robot. None when there are no blocks (the first
        point drawn is then the start pose) or while re-shaping a traced
        route. It is looked up fresh every time, never stored, so editing
        the blocks by hand moves it and the drawing can't drift off the end
        of the route."""
        if self.replaceOnApply:
            return None
        end = self.routeEnd()
        if end is None and self.startPlaced:
            end = tuple(self.start)         # the robot as placed on the map
        elif end is None and self.robotPose:
            # nothing planned yet: the first path starts from the robot
            # itself, wherever it currently is
            end = tuple(self.robotPose)
        return end

    def effectiveWaypoints(self):
        """Control points of the drawing, led by the anchor if there is one."""
        end = self.anchor()
        # The leg out of the anchor is whatever the first point drawn is: a
        # left click curves out of the route's end, a right click runs
        # straight to it. (A straight anchor would make the first leg
        # straight whatever you clicked.)
        first = self.waypoints[0] if self.waypoints else None
        mode = first[2] if first is not None and len(first) > 2 else STRAIGHT
        lead = [(end[0], end[1], mode)] if end else []
        return lead + list(self.waypoints)

    def toggleDraw(self):
        if self.pending:
            self.cancelPending()
        self.drawMode = not self.drawMode
        self.aimPending = False
        self.drawBtnRow.config(relief="sunken" if self.drawMode else "raised")
        self._setIconActive("draw", self.drawMode)       # lit while drawing
        if self.drawMode and self.pickStart:
            self.togglePickStart()
        self.redrawMap()

    def undoWaypoint(self):
        if self.waypoints:
            self.redoStack.append(("wp", self.waypoints.pop()))
            if self.selWp is not None and self.selWp >= len(self.waypoints):
                self.selWp = None
            self.syncPointPanel()
            self.redrawMap()

    def pushUndo(self):
        """Snapshot the plan before a path is applied, for undoPath()."""
        self.undoStack.append(self.snapshot())
        del self.undoStack[:-50]
        self.redoStack = []             # a new edit forks history

    def undoPath(self):
        """Ctrl+Z: drop the control point being drawn, or, with none in
        progress, take back the path that was applied last."""
        if self.waypoints:
            self.undoWaypoint()
            return
        if not self.undoStack:
            return
        self.redoStack.append(("plan", self.snapshot()))
        self.restore(self.undoStack.pop())

    def snapshot(self):
        return (copy.deepcopy(self.blocks), list(self.start), self.selected)

    def restore(self, snap):
        self.blocks, start, self.selected = copy.deepcopy(snap[0]), snap[1], snap[2]
        self.start = list(start)
        for v, val in zip(self.startVars, self.start):
            v.set(fmt(val))
        self.replaceOnApply = False
        self.refreshList()
        self.showParams()

    def redoPath(self):
        """Ctrl+Y / Ctrl+Shift+Z: put back what Ctrl+Z last took away."""
        if not self.redoStack:
            return
        kind, item = self.redoStack.pop()
        if kind == "wp":
            self.waypoints.append(item)
            self.redrawMap()
        else:
            self.undoStack.append(self.snapshot())
            self.restore(item)

    def clearWaypoints(self):
        self.waypoints = []
        self.replaceOnApply = False
        self.drawInfo.config(text="")
        self.redrawMap()

    def clearMap(self):
        """Wipe the lines off the map: the drawn control points and the
        path the robot actually drove. The movement blocks are the plan
        rather than scribble, so those are only cleared if asked for."""
        self.waypoints = []
        self.replaceOnApply = False
        self.robotPath = []
        self.drawInfo.config(text="")
        if self.blocks and messagebox.askyesno(
                "Clear map", f"Also remove the {len(self.blocks)} movement "
                             "block{} and the route they draw?".format(
                                 "" if len(self.blocks) == 1 else "s"),
                default="no", parent=self):
            self.pushUndo()
            self.blocks = []
            self.selected = None
            self.showParams()
        self.refreshList()

    def toggleWaypointMode(self, i):
        """Flip a dropped point between curve and straight."""
        x, y = self.waypoints[i][0], self.waypoints[i][1]
        mode = self.waypoints[i][2] if len(self.waypoints[i]) > 2 else CURVE
        self.waypoints[i] = (x, y, STRAIGHT if mode == CURVE else CURVE) + tuple(
            self.waypoints[i][3:])
        self.redrawMap()

    def removeWaypointAt(self, ex, ey):
        i = self._hitWaypoint(ex, ey)
        if i is not None:
            del self.waypoints[i]
            self.selWp = None
            self.syncPointPanel()
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
        self.replaceOnApply = True      # the traced route replaces the blocks
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
        end = self.anchor()
        # With blocks already in the plan the drawing is led by their end
        # point (see anchor()), so the new path is appended and joins on
        # exactly; with none, the first point drawn becomes the start pose.
        replace = end is None
        fromRobot = end is not None and not self.blocks   # first path: starts at the robot
        if len(pts) < 2:
            messagebox.showinfo("Draw path", "Drop at least one point on the map first."
                                if end else "Drop at least two points on the map first.",
                                parent=self)
            return
        self.pushUndo()
        fwd = not self.reverseVar.get()
        exitRange = round(self.spacing() / 2.0, 2)
        # With "smooth" off the route is a turn-and-drive one: aim down each
        # leg first, so the moveToPoint that follows is a straight line.
        turnFirst = not self.smoothVar.get()
        blocks = []
        if replace:
            x0, y0 = pts[0][0], pts[0][1]
            hdg = heading_to(x0, y0, pts[1][0], pts[1][1], fwd)
            if len(self.waypoints[0]) > 3 and self.waypoints[0][3] is not None:
                hdg = self.waypoints[0][3]         # the start heading was chosen
            self.start = [round(x0, 1), round(y0, 1), round(hdg, 1)]
            for v, val in zip(self.startVars, self.start):
                v.set(fmt(val))
            rest = pts[1:]
            cx, cy, ch = x0, y0, hdg
        else:
            if fromRobot:       # the start pose is where the robot is now, heading included
                self.start = [round(v, 1) for v in end]
                for v, val in zip(self.startVars, self.start):
                    v.set(fmt(val))
            cx, cy, ch = end
            rest = pts[1:]      # pts[0] is the anchor: the robot is already there
        # headings chosen on drawn points: each lands on the nearest point of
        # the route, which then turns to it (and so stops there)
        chosen = {}
        for k, w in enumerate(self.waypoints):
            if len(w) > 3 and w[3] is not None and not (replace and k == 0):
                n = min(range(len(rest)),
                        key=lambda m: math.hypot(rest[m][0] - w[0], rest[m][1] - w[1]),
                        default=None)
                if n is not None:
                    chosen[n] = w[3]
        legs = list(self.pathLegs)      # filled by pathPoints() above, one per point after the first
        legs = legs[len(legs) - len(rest):] if len(legs) >= len(rest) else [True] * len(rest)
        for i, (x, y, stop) in enumerate(rest):
            want = heading_to(cx, cy, x, y, fwd)
            if turnFirst or legs[i]:
                # A straight leg (right click) is aimed down first, so the
                # moveToPoint after it drives a straight line. Without that
                # the robot steers onto the point as it goes -- a curve --
                # whenever it isn't already facing it. Only where the route
                # actually turns: a leg carrying straight on needs no turn.
                if abs((want - ch + 180) % 360 - 180) > self.TURN_DEG:
                    blocks.append(new_block("turnToHeading", theta=round(want, 2)))
                    blocks.append(new_block("waitUntilDone"))
            ch = want           # arriving along this leg, the robot faces its way
            # points along a curve are flowed through (minSpeed + early exit);
            # straight points and the last point are arrived on properly.
            stop = stop or i == len(rest) - 1 or i in chosen
            blocks.append(new_block("moveToPoint", x=round(x, 2), y=round(y, 2), forwards=fwd,
                                    minSpeed=0 if stop else 40,
                                    earlyExitRange=0.0 if stop else exitRange))
            blocks.append(new_block("waitUntilDone"))
            if i in chosen:
                if abs((chosen[i] - ch + 180) % 360 - 180) > self.TURN_DEG:
                    blocks.append(new_block("turnToHeading", theta=round(chosen[i] % 360.0, 2)))
                    blocks.append(new_block("waitUntilDone"))
                ch = chosen[i]
            cx, cy = x, y
        self.blocks = blocks if replace else self.blocks + blocks
        # Select the last movement of the path just added: a block added next
        # goes in after it (past its waitUntilDone), i.e. on the end of this
        # path -- not after the path's first block, which is where the swing
        # used to end up.
        newEnd = max((k for k in range(len(self.blocks) - len(blocks), len(self.blocks))
                      if self.blocks[k]["kind"] in ("moveToPoint", "moveToPose")),
                     default=len(self.blocks) - 1)
        self.selected = newEnd
        # The drawing is spent; the next one continues from the new end of
        # the route (and Ctrl+Z takes this path back).
        self.waypoints = []
        self.selWp = None
        self.replaceOnApply = False
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
        # one clean line, no dark halo -- the field render is plain enough
        if len(flat) < 4:
            return
        self.map.create_line(*flat, fill=color, width=width, dash=dash, arrow=arrow,
                             capstyle="round", joinstyle="round")

    def drawPoint(self, x, y, color, r=11):
        """A path point as path.jerryio draws it: a purple disc with a thin
        rim. Solid rather than see-through -- stippled fills are slow to
        paint and were part of the flicker."""
        px, py = self.toMap(x, y)
        self.map.create_oval(px - r, py - r, px + r, py + r, fill=POINT_FILL, outline=color,
                             width=1)

    HEAD_LEN = 26      # px from a point's centre to the knob on its heading line

    def headingKnob(self, i):
        """Map position (px, py) of block i's heading knob, and the point."""
        px, py = self.toMap(*self.pointPos(i))
        rad = math.radians(self.pointHeading(i)[0])
        return (px + math.sin(rad) * self.HEAD_LEN, py - math.cos(rad) * self.HEAD_LEN), (px, py)

    def pointPos(self, i):
        """Field (x, y) of block i's point. A swing has no target of its own
        (or aims at something else), so its point is where it leaves the robot."""
        b = self.blocks[i]
        if b["kind"].startswith("swing"):
            sim = self.simulation()
            if i < len(sim):
                return sim[i][0][0], sim[i][0][1]
        return b["x"], b["y"]

    def drawHeadingLine(self, i, r):
        """The line out of a point showing the way the robot faces there. A
        heading that has been set is solid; one that is only the direction
        of travel is dashed and dim. The selected point gets a knob to drag."""
        (kx, ky), (px, py) = self.headingKnob(i)
        isSet = self.pointHeading(i)[1]
        col = "#d6c8ff" if (isSet or i == self.selected) else "#b3a6e6"
        self.map.create_line(px, py, kx, ky, fill=col, width=2, arrow="last")
        if i == self.selected:
            self.map.create_oval(kx - 5, ky - 5, kx + 5, ky + 5, fill="#d6c8ff", outline=PURPLE,
                                 width=2)

    def _hitHeadingKnob(self, ex, ey):
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in (
                "moveToPoint", "moveToPose", "swingToHeading", "swingToPoint"):
            return None
        (kx, ky), _ = self.headingKnob(i)
        return i if math.hypot(kx - ex, ky - ey) <= self.HIT_PX else None

    def startKnob(self):
        """Map position of the knob on the start robot's heading line."""
        px, py = self.toMap(self.start[0], self.start[1])
        rad = math.radians(self.start[2])
        return px + math.sin(rad) * self.HEAD_LEN, py - math.cos(rad) * self.HEAD_LEN

    def drawStartRobot(self):
        """The robot where the plan starts: its footprint box, a disc on its
        centre and a heading line with a knob. Drag the robot to put it
        somewhere else, drag the knob to turn it."""
        px, py = self.toMap(self.start[0], self.start[1])
        kx, ky = self.startKnob()
        self.drawRobotBox(tuple(self.start), PURPLE, width=2)
        self.drawPoint(self.start[0], self.start[1], PURPLE, r=9)
        self.map.create_line(px, py, kx, ky, fill="#d6c8ff", width=2)
        self.map.create_oval(kx - 5, ky - 5, kx + 5, ky + 5, fill="#d6c8ff", outline=PURPLE,
                             width=2)

    def _hitStart(self, ex, ey):
        """True on the start robot's body (inside its footprint)."""
        px, py = self.toMap(self.start[0], self.start[1])
        length, wid = self.robotSize()
        reach = max(self.HIT_PX, min(length, wid) / 2 * self.ppi)
        return math.hypot(px - ex, py - ey) <= reach

    def _hitStartKnob(self, ex, ey):
        kx, ky = self.startKnob()
        return math.hypot(kx - ex, ky - ey) <= self.HIT_PX

    def finishStartDrag(self):
        """The robot has been put down: that is now where the first path
        starts, and (idle brain only) where the robot itself is told it is."""
        self.startPlaced = True
        if self.link and self.planStatus[0] != "RUNNING":
            self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")

    def drawFlag(self, x, y):
        """Chequered finish flag standing on the last point of the route."""
        px, py = self.toMap(x, y)
        top = py - 24
        self.map.create_line(px, py - 4, px, top, fill="#e8e8e8", width=2)
        s = 5
        for row in range(2):
            for col in range(3):
                dark = (row + col) % 2 == 0
                x0, y0 = px + 1 + col * s, top + row * s
                self.map.create_rectangle(x0, y0, x0 + s, y0 + s, outline="",
                                          fill="#111111" if dark else "#f4f4f4")

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

    def robotSize(self):
        """(length, width) in inches from the window, falling back to the
        default for anything that isn't a positive number."""
        def num(var):
            try:
                v = float(var.get())
                return v if v > 0 else DEFAULT_ROBOT_IN
            except (ValueError, tk.TclError):
                return DEFAULT_ROBOT_IN
        return num(self.robotLenVar), num(self.robotWidVar)

    def drawRobotBox(self, pose, color, width=2, dash=None):
        """The robot's footprint (length along the heading, width across it)
        centred on `pose`, rotated to its heading, with the front edge
        marked so it's clear which way it faces."""
        x, y, th = pose
        px, py = self.toMap(x, y)
        ppi = self.ppi
        length, wid = self.robotSize()
        rad = math.radians(th)
        fwd = (math.sin(rad), -math.cos(rad))     # screen direction of the heading
        right = (math.cos(rad), math.sin(rad))
        corners = []
        for a, f in ((-1, 1), (1, 1), (1, -1), (-1, -1)):     # front-left first
            ca, cf = a * wid / 2 * ppi, f * length / 2 * ppi
            corners += [px + right[0] * ca + fwd[0] * cf, py + right[1] * ca + fwd[1] * cf]
        self.map.create_polygon(*corners, outline=color, fill="", width=width, dash=dash)
        self.map.create_line(*corners[0:4], fill=color, width=width + 2)   # front edge

    def drawSwingEnd(self, shape, color):
        """One footprint box where a swing leaves the robot, turned to the
        heading it ends on."""
        trace, heads = shape[5], shape[6]
        if len(trace) >= 2:
            x, y = trace[-1]
            self.drawRobotBox((x, y, heads[-1]), color, width=2)

    def _swingAt(self, ex, ey):
        """Index of the swing whose path (or end footprint) is under the mouse."""
        best, bestD = None, 14.0
        for i, (b, (pose, shape)) in enumerate(zip(self.blocks, self.simulation())):
            if not shape or shape[0] != "swing":
                continue
            for x, y in shape[5][::2] + [shape[5][-1]]:
                px, py = self.toMap(x, y)
                d = math.hypot(px - ex, py - ey)
                if d < bestD:
                    best, bestD = i, d
        return best

    def drawRobotMarker(self, pose, color, label, small=False, robot=False, box=False):
        x, y, th = pose
        px, py = self.toMap(x, y)
        r = 9 if not small else 6
        rad = math.radians(th)
        tx, ty = px + math.sin(rad) * (r + 10), py - math.cos(rad) * (r + 10)
        if robot or box:
            self.drawRobotBox(pose, color, width=2 if robot else 1, dash=None if robot else (4, 3))
        if robot:
            self.map.create_oval(px - r, py - r, px + r, py + r, outline=color, width=2)
        else:
            self.drawPoint(x, y, color, r=r + 2)
        self.map.create_line(px, py, tx, ty, fill=color, width=2, arrow="last")
        if label:
            self.map.create_text(px, py, text=label, fill=color, font=("Segoe UI", 8, "bold"))

    def _onMapLeave(self, _e):
        self._hoverOver(None)
        if self.hoverSwing is not None:
            self.hoverSwing = None
            self.redrawMap()

    def onMapMotion(self, e):
        icon = self._iconAt(e.x, e.y)
        self._hoverOver(icon)
        if icon:
            self.mapStatus.config(text=self.icons[icon]["tip"])
            if str(self.map.cget("cursor")) != "hand2":
                self.map.config(cursor="hand2")
            return
        x, y = self.toField(e.x, e.y)
        hs = None if (self.pending or self.drag) else self._swingAt(e.x, e.y)
        if hs != self.hoverSwing:           # hovering a swing shows the body's path through it
            self.hoverSwing = hs
            self.redrawMap()
        if self.pickStart:
            hint = "   -- click to set START pose"
        elif self.pending:
            hint = "   -- tap to place, drag to adjust, Enter = Apply, Esc = cancel"
        elif self.drawMode:
            i = self._hitWaypoint(e.x, e.y)
            hint = ("   -- drag to move, right-click to flip curve/straight, "
                    "shift-click to remove") if i is not None else \
                   "   -- left-click = curve point, right-click = straight point"
        elif self._hitAimKnob(e.x, e.y) is not None:
            hint = "   -- drag to aim this turn"
        elif self._hitHeadingKnob(e.x, e.y) is not None:
            hint = "   -- drag to turn this point's heading"
        elif self._hitBlockPoint(e.x, e.y) is not None:
            hint = "   -- drag to move this block's point"
        elif self._hitStartKnob(e.x, e.y):
            hint = "   -- drag to turn the robot"
        elif self._hitStart(e.x, e.y):
            hint = "   -- drag to put the robot where it is"
        elif self.aimable() and (not self.drawMode or self.aimPending):
            kind = self.blocks[self.selected]["kind"]
            what = "swing" if kind.startswith("swing") else "turn"
            hint = (f"   -- click or drag to aim this {what}" if kind in self.AIM_KINDS
                    else f"   -- click to set where this {what} aims; drag to adjust")
        else:
            hint = ""
        # the cursor says what a click will do: a crosshair when it drops a
        # point, a hand over anything that can be picked up
        grab = (self._chipAt(e.x, e.y) is not None or self._hitHeadingKnob(e.x, e.y) is not None
                or self._hitStartKnob(e.x, e.y)
                or self._hitAimKnob(e.x, e.y) is not None
                or self._hitBlockPoint(e.x, e.y) is not None or self._hitStart(e.x, e.y)
                or (self.drawMode and self._hitWaypoint(e.x, e.y) is not None))
        cursor = "hand2" if grab else ("crosshair" if self.drawMode or self.pickStart or self.pending
                                       or (self.aimable() and self.aimPending) else "arrow")
        if str(self.map.cget("cursor")) != cursor:
            self.map.config(cursor=cursor)
        # inches (what the code uses and Jerry.io exports); the cm are what
        # path.jerryio's own cursor readout and point panel show on screen
        self.mapStatus.config(text=f"({x:6.1f}, {y:6.1f}) in  = ({x * 2.54:7.1f}, "
                                   f"{y * 2.54:7.1f}) cm" + hint)

    def onMapPress(self, e):
        icon = self._iconAt(e.x, e.y)
        if icon:                                # a click on one of the map's icons
            self.icons[icon]["cmd"]()
            return
        chip = self._chipAt(e.x, e.y)
        if chip:                                # a shape option beside the selected swing
            self._applyChip(chip)
            return
        x, y = self.toField(e.x, e.y)
        x, y = round(x, 1), round(y, 1)
        if self.pickStart:
            self.start[0], self.start[1] = x, y
            self.startVars[0].set(fmt(x))
            self.startVars[1].set(fmt(y))
            self.startPlaced = True
            self.togglePickStart()
            self.redrawMap()
            return
        if self.pending:                        # placing a motion: the tap sets where it goes
            knob = self._hitPendingKnob(e.x, e.y)
            if knob == "lead":
                self.drag = ("pendlead", 0)
            elif knob == "hdg":
                self.drag = ("pend", 0)         # a drag turns the finish heading
            else:
                self._placePending(e.x, e.y, True)
                self.drag = ("pend", 0)
            return
        k = self._hitLeadKnob(e.x, e.y)         # the arc's lead point on the selected moveToPose
        if k is not None:
            self.pushUndo()
            self.drag = ("lead", k)
            return
        k = self._hitHeadingKnob(e.x, e.y)     # the knob on the selected point's heading line
        if k is not None:
            self.pushUndo()
            self.drag = ("hdg", k)
            return
        k = self._hitAimKnob(e.x, e.y)          # the knob on a selected turn / swing's aim line
        if k is not None:
            self.pushUndo()
            self.drag = ("aim", k)
            return
        if self._hitStartKnob(e.x, e.y):       # the knob on the start robot's heading line
            self.pushUndo()
            self.drag = ("starthdg", 0)
            return
        if self.drawMode and self.aimPending and self.aimable():
            self.pushUndo()
            kind = self.blocks[self.selected]["kind"]
            if kind in self.AIM_KINDS:
                self._aimAt(self.selected, e.x, e.y)
                self.drag = ("aim", self.selected)
            else:
                self.blocks[self.selected]["x"], self.blocks[self.selected]["y"] = x, y
                self.autoSide(self.selected)
                self.drag = ("block", self.selected)
            self.showParams()
            self.refreshList()
            return
        if self.drawMode:
            i = self._hitWaypoint(e.x, e.y)
            j = self._hitBlockPoint(e.x, e.y) if i is None else None
            if j is not None:                  # a point of the route: drag it, as in jerry
                self.selWp = None
                self.pushUndo()
                self.selected = j
                self.refreshList()
                self.showParams()
                self.drag = ("block", j)
                return
            if i is None and self._hitStart(e.x, e.y) and not self.waypoints:
                self.pushUndo()                # pick the robot up (nothing drawn yet to click on)
                self.drag = ("start", 0)
                return
            if i is None:                      # new curve point at the end
                self.redoStack = []
                self.waypoints.append((x, y, CURVE))
                i = len(self.waypoints) - 1
            self.selWp = i                     # its heading is set with the heading buttons
            self.drag = ("wp", i)
            self.syncPointPanel()
            self.redrawMap()
            return
        i = self._hitBlockPoint(e.x, e.y)      # grab a block's point and drag it
        if i is not None:
            self.pushUndo()                    # so Ctrl+Z puts a dragged point back
            if i != self.selected:
                self.selected = i
                self.refreshList()
                self.showParams()
            self.drag = ("block", i)
            return
        if self._hitStart(e.x, e.y):           # point at the robot and drag it where it is
            self.pushUndo()
            self.drag = ("start", 0)
            return
        if self.aimable() and self.blocks[self.selected]["kind"] in self.AIM_KINDS:
            # a turn / swing to a heading: the click (and drag) aims it
            self.pushUndo()
            self._aimAt(self.selected, e.x, e.y)
            self.drag = ("aim", self.selected)
            self.showParams()
            self.refreshList()
            return
        if self.selected is None or self.blocks[self.selected]["kind"] not in HAS_POINT:
            return
        self.pushUndo()
        b = self.blocks[self.selected]
        b["x"], b["y"] = x, y
        self.autoSide(self.selected)
        self.drag = ("block", self.selected)
        self.showParams()
        self.refreshList()

    def onMapDrag(self, e):
        if not self.drag:
            return
        x, y = self.toField(e.x, e.y)
        x, y = round(x, 1), round(y, 1)
        what, i = self.drag
        if what == "pend":
            if self.pending:
                self._placePending(e.x, e.y, False)
        elif what == "pendlead":
            if self.pending:
                self.setLead(self.pending[0], self._poseAt(self.pending[1]), e.x, e.y)
                self.syncPendingParams()
                self.redrawMap()
        elif what == "lead" and i < len(self.blocks):
            self.setLead(self.blocks[i], self.poseBefore(i), e.x, e.y)
            self.codeLabel.config(text=block_code(self.blocks[i]))
            self.syncList()
        elif what == "wp":
            if i < len(self.waypoints):
                w = self.waypoints[i]
                self.waypoints[i] = (x, y, w[2] if len(w) > 2 else CURVE) + tuple(w[3:])
                self.syncPointPanel()
                self.redrawMap()
        elif what == "start":
            self.start[0], self.start[1] = x, y
            for v, val in zip(self.startVars, self.start):
                v.set(fmt(val))
            self.redrawMap()
        elif what == "starthdg":
            px, py = self.toMap(self.start[0], self.start[1])
            if math.hypot(e.x - px, e.y - py) > 3:
                self.start[2] = round(math.degrees(math.atan2(e.x - px, py - e.y)) % 360.0, 1)
                self.startVars[2].set(fmt(self.start[2]))
                self.redrawMap()
        elif what == "aim" and i < len(self.blocks):
            self._aimAt(i, e.x, e.y)
            self.codeLabel.config(text=block_code(self.blocks[i]))
            self.syncList()
        elif what == "hdg" and i < len(self.blocks):
            # turn the heading to point at the mouse
            _, (px, py) = self.headingKnob(i)
            if math.hypot(e.x - px, e.y - py) > 3:
                self.setPointHeading(i, math.degrees(math.atan2(e.x - px, py - e.y)))
                self.codeLabel.config(text=block_code(self.blocks[i]))
                self.syncPointPanel()
                self.syncList()
        elif i < len(self.blocks):
            b = self.blocks[i]
            b["x"], b["y"] = x, y
            self.autoSide(i)
            self.codeLabel.config(text=block_code(b))
            self.syncPointPanel()
            self.syncList()             # redraws the map too

    def onMapRelease(self, _e):
        if self.drag and self.drag[0] in ("start", "starthdg"):
            self.finishStartDrag()
        if self.drag and self.drag[0] in ("block", "hdg", "aim", "lead"):
            self.showParams()
        if self.drag and self.drag[0] in ("aim", "block"):
            self.aimPending = False         # aimed: the next click drops a point again
        self.drag = None

    def onMapRightClick(self, e):
        """Right button: drop a STRAIGHT point -- the legs either side of
        it stay dead straight and the robot stops on it. On a point that's
        already there, flip it between straight and curve instead."""
        if not self.drawMode:
            return
        i = self._hitWaypoint(e.x, e.y)
        if i is not None:
            self.selWp = i
            self.syncPointPanel()
            self.toggleWaypointMode(i)
            return
        x, y = self.toField(e.x, e.y)
        self.redoStack = []
        self.waypoints.append((round(x, 1), round(y, 1), STRAIGHT))
        self.selWp = len(self.waypoints) - 1
        self.syncPointPanel()
        self.redrawMap()

    # -- driving (WASD) --
    # The brain drops a DRIVE command it hasn't heard from in 300 ms, so
    # the current one is repeated this often while a key is held. Well
    # inside that, and slow enough not to crowd the radio link (the vexcom
    # link paces its sends 60 ms apart).
    DRIVE_KEEPALIVE_MS = 120

    def toggleDrive(self):
        self.driveMode = not self.driveMode
        self.driveBtn.config(relief="sunken" if self.driveMode else "raised")
        if self.driveMode:
            if self.drawMode:      # W/A/S/D would fight the drawing keys
                self.toggleDraw()
            self.focus_set()
            self.driveStatus.config(text="ready -- hold W A S D", fg="#2ECC71")
            self._driveTick()
        else:
            self.stopDrive()

    def onDriveKeyPress(self, e):
        if not self.driveMode or self._typing():
            return
        k = e.keysym.lower()
        if k in ("w", "a", "s", "d"):
            self.driveKeys.add(k)
            self._sendDrive()

    def onDriveKeyRelease(self, e):
        if not self.driveMode:
            return
        self.driveKeys.discard(e.keysym.lower())
        self._sendDrive()

    def driveMix(self):
        """W/S forwards and back, A/D turn, mixed arcade style when held
        together. Turning is given a bit less than full power -- at full
        it spins faster than anyone can steer by keyboard."""
        p = self.powerVar.get()
        fwd = ("w" in self.driveKeys) - ("s" in self.driveKeys)
        turn = ("d" in self.driveKeys) - ("a" in self.driveKeys)
        l = fwd * p + turn * p * 0.7
        r = fwd * p - turn * p * 0.7
        cap = max(1.0, abs(l), abs(r), float(p))
        if cap > 127:               # keep the mix's shape when it clips
            l, r = l * 127 / cap, r * 127 / cap
        return int(round(max(-127, min(127, l)))), int(round(max(-127, min(127, r))))

    def _sendDrive(self, force=False):
        l, r = self.driveMix() if self.driveMode else (0, 0)
        if (l, r) != self.driveLast or force:
            self.link._send(f"DRIVE {l} {r}")
            self.driveLast = (l, r)
        if not self.driveMode:
            return
        if (l, r) == (0, 0):
            self.driveStatus.config(text="ready -- hold W A S D", fg="#2ECC71")
        else:
            self.driveStatus.config(text=f"L {l:+4d}  R {r:+4d}", fg="#00F0FF")

    def _driveTick(self):
        """Keepalive: resends the held command so the brain's watchdog
        doesn't time it out mid-drive."""
        if not self.driveMode:
            return
        if self.driveKeys:
            self._sendDrive(force=True)
        self.driveJob = self.after(self.DRIVE_KEEPALIVE_MS, self._driveTick)

    def stopDrive(self, _e=None):
        """Stop, and mean it: the keys are dropped, the keepalive is
        cancelled and a zero command goes out. If it doesn't arrive, the
        brain's 300 ms watchdog stops the robot anyway."""
        if self.driveJob is not None:
            self.after_cancel(self.driveJob)
            self.driveJob = None
        self.driveKeys.clear()
        if self.driveMode or self.driveLast not in (None, (0, 0)):
            self.link._send("DRIVE 0 0")
            self.driveLast = (0, 0)
        self.driveMode = False
        self.driveBtn.config(relief="raised")
        self.driveStatus.config(text="off", fg="#777")

    # -- start pose --
    def togglePickStart(self):
        self.pickStart = not self.pickStart
        self.pickBtn.config(relief="sunken" if self.pickStart else "raised")

    def applyStartPose(self):
        try:
            self.start = [float(v.get()) for v in self.startVars]
        except ValueError:
            return
        self.startPlaced = True
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
            self.startPlaced = True
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

    def syncList(self):
        """refreshList() for the middle of a drag: the block list has the same
        number of rows, so only the rows whose text changed are rewritten.
        Rebuilding the whole listbox on every mouse move made it flash."""
        if self.listbox.size() != len(self.blocks):
            return self.refreshList()
        for k, b in enumerate(self.blocks):
            text = f"{k + 1:2d}. {block_summary(b)}"
            if self.listbox.get(k) != text:
                self.listbox.delete(k)
                self.listbox.insert(k, text)
        want = (self.selected,) if self.selected is not None and self.selected < len(self.blocks) \
            else ()
        if tuple(self.listbox.curselection()) != want:
            self.listbox.selection_clear(0, "end")
            for k in want:
                self.listbox.selection_set(k)
        self.redrawMap()

    def onSelect(self, _e=None):
        self.pending = None
        self.aimPending = False
        sel = self.listbox.curselection()
        self.selected = sel[0] if sel else None
        self.showParams()
        self.redrawMap()

    # -- placing a motion: pick it, tap the map, see its path, then Apply --
    def pickMotion(self, kind):
        """A movement button. A motion is previewed before it becomes a
        block: tap the map and the path the robot would take shows as one
        dashed line; Enter / Apply adds it, Esc drops it. Everything else
        (wait, setPose, ...) is added straight away, as is anything while
        the path editor is open."""
        self.pending = None
        if kind not in MOTIONS or self.drawMode:
            self.addBlock(kind)
            return
        at = len(self.blocks) if self.selected is None else self.selected + 1
        if at < len(self.blocks) and self.blocks[at]["kind"] == "waitUntilDone":
            at += 1                 # never between a movement and its waitUntilDone
        b = new_block(kind)
        x, y, th = self._poseAt(at)
        if kind in HAS_POINT:
            b["x"], b["y"] = round(x, 1), round(y, 1)
        if kind in HAS_HEADING:
            b["theta"] = round(th, 1)
        self.pending = (b, at)
        self.showParams()           # its parameters are editable straight away
        self.mapStatus.config(text=f"{kind}: tap the map to place it"
                                   + (" (drag to turn it)" if kind == "moveToPose" else "")
                                   + "  --  Enter = Apply, Esc = cancel")
        self.redrawMap()

    def _poseAt(self, at):
        """Where the robot is as the block at index `at` would start."""
        sim = self.simulation()
        return sim[at - 1][0] if 0 < at <= len(sim) else tuple(self.start)

    def _placePending(self, ex, ey, first):
        """A tap (first) or drag on the map while a motion is being placed."""
        b, at = self.pending
        k = b["kind"]
        x0, y0, th0 = self._poseAt(at)
        if k in self.AIM_KINDS:                     # aimed by direction
            px, py = self.toMap(x0, y0)
            if math.hypot(ex - px, ey - py) < 4:
                return
            b["theta"] = round(math.degrees(math.atan2(ex - px, py - ey)) % 360.0, 1)
        elif first or k != "moveToPose":            # a target point
            fx, fy = self.toField(ex, ey)
            b["x"], b["y"] = round(fx, 1), round(fy, 1)
            if k == "moveToPose":
                b["theta"] = round(heading_to(x0, y0, b["x"], b["y"]), 1)
        else:                                       # moveToPose: drag turns the finish heading
            px, py = self.toMap(b["x"], b["y"])
            if math.hypot(ex - px, ey - py) >= 4:
                b["theta"] = round(math.degrees(math.atan2(ex - px, py - ey)) % 360.0, 1)
        self._pendingSide()
        self.syncPendingParams()
        self.redrawMap()

    def _pendingSide(self):
        """Lock the side a pending swing turns towards (the other one if it
        goes backwards)."""
        b, at = self.pending
        if not b["kind"].startswith("swing"):
            return
        x0, y0, th0 = self._poseAt(at)
        target = b["theta"] if b["kind"] == "swingToHeading" else \
            heading_to(x0, y0, b["x"], b["y"], b.get("forwards", True))
        err = angle_err(target, th0, b.get("direction", "AUTO"))
        if abs(err) > 0.5:
            b["side"] = swing_side(err, b.get("forwards", True))

    def _paintPending(self):
        """The motion being placed: just its path (dashed, in its own colour)
        and a ring on where it goes. No handles, nothing else."""
        b, at = self.pending
        pose = self._poseAt(at)
        shape = simulate([b], pose)[0][1]
        c = self.map
        if shape and shape[0] == "trace":
            self.drawTrace(shape[1], PREVIEW, 3, (6, 3), arrow=False)
        elif shape and shape[0] == "swing":         # a line and one box where it ends
            self.drawTrace(shape[5], PREVIEW, 3, (6, 3), arrow=False)
            self.drawSwingEnd(shape, PREVIEW)
        elif shape and shape[0] == "turn":
            _, x, y, a0, a1 = shape[:5]
            self.drawTurnArc(x, y, a0, a1, PREVIEW, 3, swing=False)
        if b["kind"] in HAS_POINT:
            px, py = self.toMap(b["x"], b["y"])
            c.create_oval(px - 7, py - 7, px + 7, py + 7, outline=PREVIEW, width=2)
            if b["kind"] == "moveToPose":           # facing there, and the arc's lead point
                rad = math.radians(b["theta"])
                c.create_line(px, py, px + math.sin(rad) * 24, py - math.cos(rad) * 24,
                              fill=PREVIEW, width=2, arrow="last")
                self.drawLeadHandle(b, pose, PREVIEW)
            elif b["kind"] in self.AIM_POINT_KINDS:
                x0, y0 = self.toMap(pose[0], pose[1])
                c.create_line(x0, y0, px, py, fill=PREVIEW, width=1, dash=(2, 4))
        self.drawShapeChips(b, pose)        # the same settings chips an added swing has

    # -- the arc of a moveToPose --
    # Its curve is lemlib's boomerang: the robot chases a "carrot" that sits
    # behind the target along its finishing heading, lead * distance away.
    # So two points shape the arc, like the handles of a bezier: the
    # heading knob out the front of the target (the way it ends up
    # facing) and the lead point behind it (how wide the curve swings).
    def leadHandle(self, b, pose0):
        """Map position of the lead point of moveToPose block b."""
        tth = b["theta"] if b.get("forwards", True) else b["theta"] + 180
        dist = math.hypot(b["x"] - pose0[0], b["y"] - pose0[1])
        r, reach = math.radians(tth), b.get("lead", 0.6) * dist
        return self.toMap(b["x"] - reach * math.sin(r), b["y"] - reach * math.cos(r))

    def setLead(self, b, pose0, ex, ey):
        """Slide the lead point to the mouse, along the heading line."""
        dist = math.hypot(b["x"] - pose0[0], b["y"] - pose0[1])
        if dist < 1.0:
            return
        tth = b["theta"] if b.get("forwards", True) else b["theta"] + 180
        mx, my = self.toField(ex, ey)
        r = math.radians(tth)
        back = -((mx - b["x"]) * math.sin(r) + (my - b["y"]) * math.cos(r))
        b["lead"] = round(max(0.0, min(1.0, back / dist)), 2)

    def drawLeadHandle(self, b, pose0, color):
        hx, hy = self.leadHandle(b, pose0)
        tx, ty = self.toMap(b["x"], b["y"])
        self.map.create_line(tx, ty, hx, hy, fill=color, width=1, dash=(2, 3))
        self.map.create_oval(hx - 5, hy - 5, hx + 5, hy + 5, fill=color, outline="")

    def _hitLeadKnob(self, ex, ey):
        """Index of the selected moveToPose whose lead point is under the mouse."""
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] != "moveToPose":
            return None
        hx, hy = self.leadHandle(self.blocks[i], self.poseBefore(i))
        return i if math.hypot(hx - ex, hy - ey) <= self.HIT_PX else None

    def _hitPendingKnob(self, ex, ey):
        """"lead" or "hdg" when the mouse is on a handle of the moveToPose
        being placed, else None."""
        b, at = self.pending
        if b["kind"] != "moveToPose":
            return None
        hx, hy = self.leadHandle(b, self._poseAt(at))
        if math.hypot(hx - ex, hy - ey) <= self.HIT_PX:
            return "lead"
        tx, ty = self.toMap(b["x"], b["y"])
        rad = math.radians(b["theta"])
        if math.hypot(tx + math.sin(rad) * 24 - ex, ty - math.cos(rad) * 24 - ey) <= self.HIT_PX:
            return "hdg"
        return None

    def applyAction(self):
        """Enter / the Apply button: add the motion being placed, or else
        turn the drawn path into blocks."""
        if self.pending:
            self.applyPending()
        elif self.drawMode:
            self.applyDrawnPath()

    def applyPending(self):
        b, at = self.pending
        self.pending = None
        self.pushUndo()
        self.blocks.insert(at, b)
        # lemlib motions are asynchronous: each gets its waitUntilDone, as addBlock does
        self.blocks.insert(at + 1, new_block("waitUntilDone"))
        self.selected = at
        self.refreshList()
        self.showParams()
        self.mapStatus.config(text=f"{b['kind']} added")

    def cancelPending(self):
        self.pending = None
        self.mapStatus.config(text="")
        self.showParams()           # back to the selected block's
        self.redrawMap()

    def addBlock(self, kind):
        # New blocks start where the route currently ends, so the map shows
        # something sensible before you've typed anything.
        aims = kind in self.AIM_KINDS + self.AIM_POINT_KINDS
        if self.drawMode and aims and self.waypoints and len(self.pathPoints()) >= 2:
            # The points drawn so far are not blocks yet, so the swing would
            # have nothing to hang off and land on the robot's pose. Apply
            # them first: the swing then goes on the end of this path.
            self.applyDrawnPath()
        self.aimPending = self.drawMode and aims
        sim = self.simulation()
        end = sim[-1][0] if sim else tuple(self.start)
        self.pushUndo()
        b = new_block(kind)
        if kind in HAS_POINT:
            b["x"], b["y"] = round(end[0], 1), round(end[1], 1)
        if kind in HAS_HEADING:
            b["theta"] = round(end[2], 1)
        at = len(self.blocks) if self.selected is None else self.selected + 1
        if self.drawMode and aims:
            at = len(self.blocks)   # drawing continues from the route's end, so does this
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
        if kind in self.AIM_KINDS + self.AIM_POINT_KINDS:
            what = "swing" if kind.startswith("swing") else "turn"
            self.mapStatus.config(text=f"Click on the map to aim this {what}"
                                       + (", drag to adjust" if kind in self.AIM_KINDS else "")
                                       + ("  (then keep drawing)" if self.drawMode else ""))

    # Every edit below takes an undo snapshot first, so Ctrl+Z covers block
    # edits as well as drawn paths.
    def moveBlock(self, delta):
        i = self.selected
        if i is None:
            return
        j = i + delta
        if 0 <= j < len(self.blocks):
            self.pushUndo()
            self.blocks[i], self.blocks[j] = self.blocks[j], self.blocks[i]
            self.selected = j
            self.refreshList()

    def duplicateBlock(self):
        if self.selected is None:
            return
        self.pushUndo()
        self.blocks.insert(self.selected + 1, dict(self.blocks[self.selected]))
        self.selected += 1
        self.refreshList()

    def deleteBlock(self):
        if self.selected is None or self.selected >= len(self.blocks):
            return
        self.pushUndo()
        del self.blocks[self.selected]
        self.selected = min(self.selected, len(self.blocks) - 1) if self.blocks else None
        self.refreshList()
        self.showParams()

    def clearBlocks(self):
        if self.blocks and not messagebox.askyesno(
                "Clear", "Remove every block?\n(Ctrl+Z brings them back.)", parent=self):
            return
        self.pushUndo()
        self.blocks = []
        self.selected = None
        self.refreshList()
        self.showParams()

    # -- aiming swings and turns on the map --
    AIM_KINDS = ("turnToHeading", "swingToHeading")          # aimed by direction
    AIM_POINT_KINDS = ("turnToPoint", "swingToPoint")        # aimed by a target point

    def aimable(self):
        """True when the selected block is a turn or swing, i.e. something a
        click or drag on the map aims. (While drawing a path a click normally
        drops a point; a swing just added in draw mode is "pending" and takes
        the next click -- see aimPending.)"""
        i = self.selected
        return (i is not None and i < len(self.blocks)
                and self.blocks[i]["kind"] in self.AIM_KINDS + self.AIM_POINT_KINDS)

    def aimVisible(self):
        """Whether the selected turn / swing shows its aim line and shape
        options. While drawing, only until you carry on drawing: once more
        points follow it the swing is no longer at the end of the path, and
        its controls left hanging at the old pose looked like it still was."""
        return self.aimable() and (not self.drawMode or self.aimPending or not self.waypoints)

    def poseBefore(self, i):
        """Where the robot is as block i starts. The simulated robot stops a
        little short of (or past) the point it drove to, so when it is
        essentially at the target of the movement just before, that target is
        used: the aim line then sits on the path's last point."""
        sim = self.simulation()
        if not 0 < i <= len(sim):
            return tuple(self.start)
        x, y, th = sim[i - 1][0]
        for j in range(i - 1, -1, -1):
            b = self.blocks[j]
            if b["kind"].startswith("swing"):
                break           # the robot is where the swing left it, not at an older target
            if b["kind"] in self.ROUTE_POINTS:
                if math.hypot(b["x"] - x, b["y"] - y) <= 6.0:
                    x, y = b["x"], b["y"]
                break
        return (x, y, th)

    def autoSide(self, i):
        """Pick which side a swing pivots on from the way it turns: a swing
        forwards turning clockwise locks the right side, counter-clockwise
        the left. Set when a swing is aimed on the map; the side box in the
        parameters still overrides it (e.g. for a swing in reverse)."""
        b = self.blocks[i]
        if not b["kind"].startswith("swing"):
            return
        x0, y0, th0 = self.poseBefore(i)
        if b["kind"] == "swingToHeading":
            target = b["theta"]
        elif (b["x"], b["y"]) != (x0, y0):
            target = heading_to(x0, y0, b["x"], b["y"], b.get("forwards", True))
        else:
            return
        err = angle_err(target, th0, b.get("direction", "AUTO"))
        if abs(err) > 0.5:
            b["side"] = swing_side(err, b.get("forwards", True))

    def _aimAt(self, i, ex, ey):
        """Point turnToHeading / swingToHeading block i at map position
        (ex, ey), as seen from where the robot is when the block starts."""
        x0, y0, _ = self.poseBefore(i)
        px, py = self.toMap(x0, y0)
        if math.hypot(ex - px, ey - py) < 4:
            return
        self.blocks[i]["theta"] = round(math.degrees(math.atan2(ex - px, py - ey)) % 360.0, 1)
        self.autoSide(i)

    def _aimKnob(self, i, pose0=None):
        """Map position of the knob on the aim line of heading block i, and
        of the robot it comes out of."""
        x0, y0, _ = pose0 or self.poseBefore(i)
        px, py = self.toMap(x0, y0)
        rad = math.radians(self.blocks[i]["theta"])
        reach = self.HEAD_LEN * 1.7
        return (px + math.sin(rad) * reach, py - math.cos(rad) * reach), (px, py)

    def _hitAimKnob(self, ex, ey):
        i = self.selected
        if not self.aimVisible() or self.blocks[i]["kind"] not in self.AIM_KINDS:
            return None
        (kx, ky), _ = self._aimKnob(i)
        return i if math.hypot(kx - ex, ky - ey) <= self.HIT_PX else None

    def drawShapeChips(self, b, pose0):
        """Small option chips beside the selected swing / turn: which side it
        pivots on, which way round it turns, and for a point, forwards or in
        reverse. lemlib's swing has no radius setting -- the arc comes from
        the pivot side (the robot turns about that wheel) and the direction --
        so these are the ways to change its curve. Clicking one sets the
        block's parameter and the arc redraws."""
        params = KIND_PARAMS[b["kind"]]
        rows = []
        if "side" in params:
            rows.append(("pivot", [("L", "side", "LEFT"), ("R", "side", "RIGHT")]))
        if "direction" in params:
            rows.append(("turn", [("auto", "direction", "AUTO"), ("cw", "direction", "CW"),
                                  ("ccw", "direction", "CCW")]))
        if "forwards" in params:
            rows.append(("drive", [("fwd", "forwards", True), ("back", "forwards", False)]))
        px, py = self.toMap(pose0[0], pose0[1])
        c = self.map
        h, gap = 20, 4
        widest = max(6 + sum(18 + 7 * len(t) + gap for t, _, _ in chips) for _, chips in rows)
        x0 = px + 40
        room = c.winfo_width() - self.ICON_MARGIN - 8      # keep clear of the icon column
        if x0 + widest > room:                             # no room on the right: go left
            x0 = max(46, px - 34 - widest)
        y = max(6, py - 12 - len(rows) * (h + gap))
        for label, chips in rows:
            c.create_text(x0, y + h / 2, text=label, anchor="e", fill="#cfcfcf",
                          font=("Segoe UI", 8))
            x = x0 + 6
            for text, name, value in chips:
                w = 18 + 7 * len(text)
                on = b.get(name) == value
                c.create_rectangle(x, y, x + w, y + h, fill=PATH_GREEN if on else "#383838",
                                   outline="#101010")
                c.create_text(x + w / 2, y + h / 2, text=text, fill="#101010" if on else "#e6e6e6",
                              font=("Segoe UI", 8, "bold"))
                self._chips.append(((x, y, x + w, y + h), name, value))
                x += w + gap
            y += h + gap

    def _chipAt(self, ex, ey):
        if not (self.pending or self.aimVisible()):
            return None
        for (x0, y0, x1, y1), name, value in self._chips:
            if x0 <= ex <= x1 and y0 <= ey <= y1:
                return name, value
        return None

    def _applyChip(self, chip):
        name, value = chip
        if self.pending:                    # a motion still being placed
            b = self.pending[0]
            if b.get(name) != value:
                b[name] = value
                if name == "forwards":
                    self._pendingSide()
                self.syncPendingParams()
                self.redrawMap()
            return
        b = self.blocks[self.selected]
        if b.get(name) == value:
            return
        self.pushUndo()
        b[name] = value
        if name == "forwards" and b["kind"].startswith("swing"):
            self.autoSide(self.selected)        # a backward swing locks the other side
        self.refreshList()
        self.showParams()

    def drawAimLine(self, i, pose0):
        """The aim line of the selected turn / swing: out of the robot's
        position as the block starts, along the heading it turns to, ending
        in a knob you can drag."""
        (kx, ky), (px, py) = self._aimKnob(i, pose0)
        self.map.create_line(px, py, kx, ky, fill=PATH_SEL, width=2, dash=(5, 3))
        self.map.create_oval(kx - 6, ky - 6, kx + 6, ky + 6, fill=PATH_SEL, outline=PURPLE,
                             width=2)

    # -- point headings (path.jerryio's per-point heading line) --
    ROUTE_POINTS = ("moveToPoint", "moveToPose", "setPose")
    HEADING_STEP = 15.0      # what the rotate buttons turn by, degrees

    def turnAfter(self, i):
        """Index of the turnToHeading that belongs to point block i (it
        follows the block, past its waitUntilDone), or None."""
        j = i + 1
        if j < len(self.blocks) and self.blocks[j]["kind"] == "waitUntilDone":
            j += 1
        if j < len(self.blocks) and self.blocks[j]["kind"] == "turnToHeading":
            return j
        return None

    def pointHeading(self, i):
        """(heading, set) of route point i. moveToPose / setPose carry their
        own theta; a moveToPoint is 'set' when a turnToHeading follows it,
        and otherwise shows the way the robot is facing as it arrives."""
        b = self.blocks[i]
        if b["kind"] in HAS_HEADING and not b["kind"].startswith("swing"):
            return b["theta"], True
        j = self.turnAfter(i)
        if j is not None:
            return self.blocks[j]["theta"], True
        sim = self.simulation()
        return (sim[i][0][2] if i < len(sim) else 0.0), False

    def setPointHeading(self, i, theta):
        """Give route point i a heading. moveToPose / setPose just take the
        theta; a moveToPoint gets a turnToHeading (and waitUntilDone) right
        after it, which turns the robot in place to that heading there."""
        theta = round(theta % 360.0, 1)
        b = self.blocks[i]
        if b["kind"] in HAS_HEADING and not b["kind"].startswith("swing"):
            b["theta"] = theta
            return
        j = self.turnAfter(i)
        if j is not None:
            self.blocks[j]["theta"] = theta
            return
        at = i + 1
        if at >= len(self.blocks) or self.blocks[at]["kind"] != "waitUntilDone":
            self.blocks.insert(at, new_block("waitUntilDone"))
        self.blocks.insert(at + 1, new_block("turnToHeading", theta=theta))
        self.blocks.insert(at + 2, new_block("waitUntilDone"))

    def rotatePoint(self, delta):
        """The rotate buttons: turn the selected point's heading by delta."""
        w = self.curWp()
        if w is not None:
            self.setWpHeading(w, self.wpHeading(w)[0] + delta)
            self.syncPointPanel()
            self.redrawMap()
            return
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in self.ROUTE_POINTS:
            return
        self.pushUndo()
        self.setPointHeading(i, self.pointHeading(i)[0] + delta)
        self.refreshList()
        self.showParams()

    def syncPointPanel(self):
        """Fill the point panel from the selected route point (blank if the
        selection isn't one)."""
        w = self.curWp()
        if w is not None:
            wp = self.waypoints[w]
            for v, val in zip(self.ptVars, (wp[0], wp[1], self.wpHeading(w)[0])):
                v.set(fmt(round(val, 2)))
            return
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in self.ROUTE_POINTS:
            for v in self.ptVars:
                v.set("")
            return
        b = self.blocks[i]
        for v, val in zip(self.ptVars, (b["x"], b["y"], self.pointHeading(i)[0])):
            v.set(fmt(round(val, 2)))

    def applyPointPanel(self):
        w = self.curWp()
        if w is not None:
            try:
                x, y, h = (float(v.get()) for v in self.ptVars)
            except ValueError:
                return self.syncPointPanel()
            wp = self.waypoints[w]
            if (wp[0], wp[1], round(h % 360.0, 1)) == (
                    x, y, round(self.wpHeading(w)[0] % 360.0, 1)):
                return                   # nothing changed
            hdg = tuple(wp[3:])
            self.waypoints[w] = (x, y, wp[2] if len(wp) > 2 else CURVE) + hdg
            if round(h % 360.0, 1) != round(self.wpHeading(w)[0] % 360.0, 1):
                self.setWpHeading(w, h)
            self.redrawMap()
            return
        i = self.selected
        if i is None or i >= len(self.blocks) or self.blocks[i]["kind"] not in self.ROUTE_POINTS:
            return
        try:
            x, y, h = (float(v.get()) for v in self.ptVars)
        except ValueError:
            return self.syncPointPanel()
        b = self.blocks[i]
        if (b["x"], b["y"], round(h % 360.0, 1)) == (x, y, round(self.pointHeading(i)[0] % 360.0, 1)):
            return                       # nothing changed (FocusOut fires on every tab through)
        self.pushUndo()
        b["x"], b["y"] = x, y
        self.setPointHeading(i, h)
        self.refreshList()
        self.showParams()

    # -- parameter panel --
    def showParams(self):
        self.syncPointPanel()
        for w in self.paramFrame.winfo_children():
            w.destroy()
        self.paramWidgets = {}
        if self.pending:        # a motion still being placed: tune it before Apply
            b = self.pending[0]
            self.paramFrame.config(text=f" new {b['kind']}  (not applied yet) ")
        elif self.selected is None:
            tk.Label(self.paramFrame, text="select a block (or add one)", fg="#777",
                     bg=PANEL_BG).grid(row=0, column=0, padx=6, pady=4)
            self.codeLabel.config(text="")
            return
        else:
            b = self.blocks[self.selected]
            self.paramFrame.config(text=f"{self.selected + 1}. {b['kind']}")
        col = 0
        for name in KIND_PARAMS[b["kind"]]:
            default, kind = PARAM_SPEC[name]
            tk.Label(self.paramFrame, text=name, fg="#aaa", bg=PANEL_BG,
                     font=("Segoe UI", 8)).grid(row=(col // 5) * 2, column=col % 5, sticky="w",
                                                padx=6)
            if kind == "bool":
                var = tk.BooleanVar(value=bool(b[name]))
                w = tk.Checkbutton(self.paramFrame, variable=var, bg=PANEL_BG,
                                   text="forwards" if var.get() else "backwards",
                                   fg="#ddd", selectcolor=INPUT_BG,
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

    def syncPendingParams(self):
        """Put the pending block's values into the parameter boxes in place
        (tapping or dragging on the map moves them) without rebuilding the
        panel, which would steal the focus from a box being typed in."""
        if not self.pending:
            return
        b = self.pending[0]
        for name, var in self.paramWidgets.items():
            if isinstance(var, tk.BooleanVar):
                var.set(bool(b[name]))
            elif PARAM_SPEC[name][1] in ("num", "int"):
                var.set(fmt(b[name]))
            else:
                var.set(str(b[name]))
        for w in self.paramFrame.winfo_children():
            if isinstance(w, tk.Checkbutton) and "forwards" in b:
                w.config(text="forwards" if b["forwards"] else "backwards")
        self.codeLabel.config(text=block_code(b))

    def paramChanged(self, name):
        if self.pending:
            return self.pendingParamChanged(name)
        if self.selected is None:
            return
        b = self.blocks[self.selected]
        var = self.paramWidgets[name]
        _, kind = PARAM_SPEC[name]
        try:
            if kind == "bool":
                new = bool(var.get())
            elif isinstance(kind, list):
                new = var.get()
            elif kind == "int":
                new = int(float(var.get()))
            else:
                new = float(var.get())
        except (ValueError, tk.TclError):
            return
        if new == b[name]:
            return          # FocusOut fires on every tab through a field: nothing changed
        self.pushUndo()
        b[name] = new
        self.codeLabel.config(text=block_code(b))
        # flipping a swing between forwards and backwards locks the other side
        if name == "forwards" and b["kind"].startswith("swing"):
            self.autoSide(self.selected)
            self.paramWidgets["side"].set(b["side"])
        # forwards checkbox text follows its value
        if kind == "bool":
            for w in self.paramFrame.winfo_children():
                if isinstance(w, tk.Checkbutton):
                    w.config(text="forwards" if b[name] else "backwards")
        self.refreshList()

    def pendingParamChanged(self, name):
        """A parameter box edited while a motion is still being placed: it
        goes into the pending block and the preview redraws. No undo step --
        nothing is in the plan yet."""
        b = self.pending[0]
        var = self.paramWidgets[name]
        _, kind = PARAM_SPEC[name]
        try:
            if kind == "bool":
                new = bool(var.get())
            elif isinstance(kind, list):
                new = var.get()
            elif kind == "int":
                new = int(float(var.get()))
            else:
                new = float(var.get())
        except (ValueError, tk.TclError):
            return
        if new == b[name]:
            return
        b[name] = new
        if name == "forwards":
            self._pendingSide()
        self.syncPendingParams()
        self.redrawMap()

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
        if self.driveMode:      # the motion is about to take the drivetrain
            self.stopDrive()
        self.pendingRun = "ALL" if which is None else which
        self.uploadTries = 1
        if which is None and self.setStartVar.get():
            # The route is in field coordinates, so it only matches the map if
            # the robot starts where the map says it does. Without this the
            # run starts from whatever pose the robot's odometry has -- the
            # drawn start pose is never sent -- and every point is driven to
            # from the wrong place.
            self.link._send(f"POSE {self.start[0]:.2f} {self.start[1]:.2f} {self.start[2]:.2f}")
        self.uploadPlan()
        self.brainStatus.config(text=f"uploading {len(self.blocks)} blocks to the robot...",
                                fg="#FF8C00")

    def startOffset(self):
        """(inches, degrees) between the robot's live pose and the plan's start
        pose, when that is enough to matter; None if it is close or unknown."""
        if not self.robotPose:
            return None
        d = math.hypot(self.robotPose[0] - self.start[0], self.robotPose[1] - self.start[1])
        a = abs((self.robotPose[2] - self.start[2] + 180) % 360 - 180)
        return (d, a) if d > 2.0 or a > 5.0 else None

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
        if self.onBrainScreen:
            # the brain empties its path on a new run or a pose reset; keeping
            # the old one would leave last run's line lying under the new one
            self.robotPath = state.get("path") or []
        elif state.get("path"):
            self.robotPath = state["path"]
        status, idx, count = state.get("plan") or ("IDLE", -1, 0)
        self.planStatus = (status, idx, count)
        self.checkPendingRun(count)
        # The robot sends a state 20 times a second whether or not anything
        # moved; redrawing an identical map just makes it shimmer.
        sig = (None if not self.robotPose else
               (round(self.robotPose[0], 1), round(self.robotPose[1], 1),
                round(self.robotPose[2] * 2) / 2),
               len(self.robotPath), status, idx, count, self.onBrainScreen)
        unchanged = sig == self._lastState
        self._lastState = sig
        if self.pendingRun is not None or time.time() < self.statusHold:
            if not unchanged:
                self.redrawLive()
            return
        if self.onBrainScreen:
            txt = f"brain: PLANNER  {status}  ({count} block{'' if count == 1 else 's'} on robot)"
            if idx >= 0:
                txt += f"  step {idx + 1}/{count}"
            off = self.startOffset()
            if off and status != "RUNNING":
                txt += f"   [robot is {off[0]:.1f} in / {off[1]:.0f} deg off the start pose]"
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
            # a drawing that has no blocks or placed robot behind it starts at
            # the live robot, so that one does need the whole route repainted
            if self.waypoints and not self.blocks and not self.startPlaced:
                self.redrawMap()
            else:
                self.redrawLive()

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
                      bg=INPUT_BG, fg="#2ECC71")
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
                       "waypoints": [[p[0], p[1], p[2] if len(p) > 2 else CURVE] + list(p[3:4])
                                     for p in self.waypoints]}, f, indent=2)
        self.planName = os.path.basename(path)
        self._retitle()
        # ...and the plan as a function in src/autonomous.cpp, named after the file
        if not self.blocks:
            messagebox.showinfo("Save", "Plan saved. It has no blocks, so no function was "
                                "written to autonomous.cpp.", parent=self)
            return
        try:
            name, replaced = write_auto_function(
                auto_function_name(os.path.splitext(os.path.basename(path))[0]),
                self.blocks, self.start)
        except OSError as err:
            messagebox.showerror("Save", f"Plan saved, but autonomous.cpp couldn't be "
                                 f"updated:\n{err}", parent=self)
            return
        messagebox.showinfo(
            "Save", f"Plan saved.\n\n{'Updated' if replaced else 'Added'} void {name}() in "
                    "src/autonomous.cpp (declared in include/autonomous.hpp).\n\n"
                    f"Call {name}() from runAutonomous() to run it.", parent=self)

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
        # saved control points belong to a route with no blocks behind it; with
        # blocks loaded a drawing is anchored on their end instead
        self.waypoints = [] if self.blocks else \
            [(float(p[0]), float(p[1]),
              p[2] if len(p) > 2 and p[2] in (CURVE, STRAIGHT) else CURVE) + tuple(p[3:4])
             for p in data.get("waypoints", [])]
        self.undoStack = []
        self.redoStack = []
        self.replaceOnApply = False
        self.startPlaced = True          # a loaded plan's start pose is where the robot goes
        self.planName = os.path.basename(path)
        self._retitle()
        self.selected = 0 if self.blocks else None
        self.refreshList()
        self.showParams()
