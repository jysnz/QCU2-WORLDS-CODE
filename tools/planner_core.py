"""The planner's logic, with no window: blocks and their parameters, the simulator, the
time estimate and its calibration, the plan checks, flipping a plan to the other side, and
the code that turns a plan into C++ (src/autonomous.cpp) or into robot commands.

tools/path_planner.py is the window built on this; tools/tests/test_planner.py tests it
without needing a screen. Run:  python -m unittest discover tools/tests
"""

import copy
import json
import math
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD_PNG = os.path.join(HERE, "field.png")
PLANS_DIR = os.path.join(HERE, "plans")
EXPORT_PATH = os.path.join(HERE, "exported_path.cpp")

FIELD_IN = 144.0            # field is 12 ft square
# tools/field.png is not exactly centred: its pins sit 0.55 in left of and 0.33 in above where they
# really are (measured from the pin centres). The render is drawn shifted by this much, (right, down)
# in inches, so the rings and goals land on it.
FIELD_IMG_SHIFT_IN = (0.55, 0.33)
FIELD_BG = "#12141a"        # map ground and window chrome (deep blue-black)
PANEL_BG = "#1a1d24"        # the side panel and the cards in it
INPUT_BG = "#0f1116"        # text entries and the block list, a step darker
CARD_BORDER = "#2b303b"     # hairline round every card and entry
BTN_BG = "#272c37"          # plain buttons...
BTN_HOVER = "#323948"       # ...hovered
TEXT = "#e8eaf0"            # main text
MUTED = "#8b92a1"           # captions and hints
ACCENT = "#7c9cff"          # the one accent colour: tabs, focus rings, card titles
ACCENT_HI = "#9db4ff"
PATH_GREEN = "#8bd13f"     # the route line
PATH_SEL = "#c6f27a"        # ...the selected block's leg
PREVIEW = "#4fc3f7"         # the motion being placed, before Apply
EXIT_COL = "#ff9f1c"        # earlyExitRange ring: the motion ends once the robot is inside it
EXIT_OFF = "#8a8a8a"        # ...the same ring when minSpeed is 0 and lemlib ignores it
MERGE_GOLD = "#FFD54A"      # points picked for merging, and the Merge button while it's on
FN_BG = "#1a1d24"           # the robot functions section (same card as the rest)
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
         "moveToPoint", "moveToPose", "wait", "setPose", "waitUntilDone",
         "liftToLevel", "claw", "homeLift", "goToPin", "shootToGoal", "autoGrabPin", "shootAtGoal"]
MOTIONS = set(KINDS[:6])   # the ones that actually drive (async on the robot)
# goToPin drives too, but it is one blocking call (open claws, lift, drive, close claws), so it
# is placed on the map like a motion and never gets a waitUntilDone after it.
PLACED_ON_MAP = MOTIONS | {"goToPin", "shootAtGoal"}
# "shootAtGoal" stays in the plan as ONE block with its own parameters. Wherever the robot has to
# run it (the map, the upload, the export) it is expanded by shoot_blocks() into ordinary blocks:
# the way round the goals, the last drive in, a turn to face the goal and shootToGoal. So the brain
# and the exported code need to know nothing about it.
PIN_COLOR = "#ff9f43"      # the disc and ring on a pin you have chosen
PIN_HINT = "#d9b45a"       # the faint rings on every pin while you are choosing one
PIN_PICK_PX = 16           # a tap this close (map px) to a pin selects it
PIN_RING_PX = 15           # radius of the ring drawn around the chosen pin
# Where the pins sit on the field (inches, origin at the centre, +y up), read off tools/field.png.
# Left out on purpose: the plain yellow dots in a dark ring ((-24, 48), (-48, 24), (0, 0),
# (48, -24), (24, -48)).
DEFAULT_PINS = [(48, 48), (24, 24), (-24, -24), (-48, -48),     # yellow, on the white diagonal lines
                (-48, 48), (-24, 24), (24, -24), (48, -48),     # the four coloured crosses
                (-24, 70), (24, 70), (-24, -70), (24, -70),     # on the walls
                (-70, 24), (-70, -24), (70, 24), (70, -24)]
# The pins can be moved / added / deleted in the planner (Edit pins, P); the list is kept in
# tools/pins.json so the changes stay between runs. PINS is edited in place, never rebound.
PINS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pins.json")


def _load_pins():
    try:
        with open(PINS_FILE, encoding="utf-8") as f:
            return [(float(x), float(y)) for x, y in json.load(f)["pins"]]
    except (OSError, ValueError, KeyError, TypeError):
        return list(DEFAULT_PINS)


def save_pins():
    try:
        with open(PINS_FILE, "w", encoding="utf-8") as f:
            json.dump({"pins": [list(p) for p in PINS]}, f, indent=1)
    except OSError:
        pass


PINS = _load_pins()

# The goals (inches, origin at the centre, +y up), read off tools/field.png: the black ones with a
# yellow pin in them, and the red and blue ones. The robot routes round them (see avoid_route).
GOALS = [(-24, 48, "black"), (-48, 24, "black"), (0, 0, "black"), (48, -24, "black"),
         (24, -48, "black"), (24, 48, "blue"), (48, 24, "blue"), (-48, -24, "red"),
         (-24, -48, "red")]
GOAL_R = 2.7                # radius of a goal, inches (as drawn on tools/field.png)
GOAL_PRESS_IN = 3.0         # a shot drives this far past touching the goal, so the front is pressed on it
GOAL_PAD = 2.5              # extra room kept round (goal + half the robot), inches
GOAL_COLORS = {"black": "#9aa0ad", "red": "#ff6b6b", "blue": "#5aa9ff"}
SIDE_DIR = {"TOP": (0, 1), "BOTTOM": (0, -1), "LEFT": (-1, 0), "RIGHT": (1, 0)}
GOAL_SIDES = list(SIDE_DIR)
FIELD_LIMIT = 66.0          # waypoints stay inside +-66 in


def goal_clearance(robot_len, robot_wid):
    """How far the robot's centre must stay from a goal's centre where it can turn on the spot
    (a waypoint, a stopping point): its footprint then sweeps a circle of half the diagonal."""
    return GOAL_R + math.hypot(robot_len, robot_wid) / 2.0 + GOAL_PAD


def _seg_dist(a, b, p):
    """Distance from point p to segment a-b, and how far along (0..1) the closest point is."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1]), t


def _swept_gap(a, b, p, length, width):
    """How far point p is from the area a robot of `length` (along its heading) x `width` sweeps
    driving a -> b facing along the leg: its footprint slid from a to b. 0 or less = inside."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return math.hypot(p[0] - a[0], p[1] - a[1]) - math.hypot(length, width) / 2.0
    ux, uy = dx / L, dy / L
    u = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy        # along the leg, from a
    v = -(p[0] - a[0]) * uy + (p[1] - a[1]) * ux       # across it
    du = max(0.0, abs(u - L / 2.0) - (L / 2.0 + length / 2.0))   # past the front / back of the sweep
    dv = max(0.0, abs(v) - width / 2.0)                          # beside its sides
    if du == 0.0 and dv == 0.0:
        return -min(width / 2.0 - abs(v), L / 2.0 + length / 2.0 - abs(u - L / 2.0))
    return math.hypot(du, dv)


def avoid_route(a, b, size, depth=4, goals=None):
    """Waypoints (not including a or b) that take the robot (size = (length, width), length along
    its heading) from a to b without its footprint coming within GOAL_R + GOAL_PAD of any goal.
    A leg is checked with the footprint turned along it, so a long robot passing a goal sideways
    needs only half its width of room, and each goal in the way is passed on whichever side gives
    the shorter way round. An empty list means the straight way is already clear. A goal the leg
    starts or ends right beside is left alone: being there is the point of the leg."""
    length, width = size
    turn = goal_clearance(length, width)          # where it spins: a waypoint, an end of a leg
    need = GOAL_R + GOAL_PAD
    if goals is None:
        goals = [g for g in GOALS
                 if math.hypot(g[0] - a[0], g[1] - a[1]) >= turn * 0.85
                 and math.hypot(g[0] - b[0], g[1] - b[1]) >= turn * 0.85]
    best_g, best_t = None, 1e9
    for g in goals:
        if _swept_gap(a, b, g, length, width) < need - 0.01:
            t = _seg_dist(a, b, g)[1]
            if t < best_t:
                best_g, best_t = g, t
    if best_g is None or depth <= 0:
        return []
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / L, dx / L                 # unit normal of the line a -> b
    best, bestLen = None, None
    for sign in (1, -1):
        r = turn * 1.15                      # the waypoint is a place it turns: give it room
        w = (max(-FIELD_LIMIT, min(FIELD_LIMIT, best_g[0] + sign * nx * r)),
             max(-FIELD_LIMIT, min(FIELD_LIMIT, best_g[1] + sign * ny * r)))
        path = avoid_route(a, w, size, depth - 1, goals) + [w] + avoid_route(w, b, size, depth - 1, goals)
        pts = [a] + path + [b]
        length_ = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))
        # a way that still clips a goal is worse than any that doesn't
        bad = sum(1 for p, q in zip(pts, pts[1:]) for g in goals
                  if _swept_gap(p, q, g, length, width) < need - 0.5)
        score = length_ + 1000.0 * bad
        if bestLen is None or score < bestLen:
            best, bestLen = path, score
    return best


def shoot_points(goal, side, standoff, clr):
    """(pre, stand): `pre` is out on the chosen side of the goal, clear of it; `stand` is where the
    robot shoots from, `standoff` inches from the goal's centre on that side, facing it."""
    dx, dy = SIDE_DIR[side]
    pre_d = max(standoff + 8.0, clr + 2.0)
    clamp = lambda v: max(-FIELD_LIMIT, min(FIELD_LIMIT, v))
    return ((clamp(goal[0] + dx * pre_d), clamp(goal[1] + dy * pre_d)),
            (clamp(goal[0] + dx * standoff), clamp(goal[1] + dy * standoff)))
# Robot functions: not movements of the chassis, but things the robot does at the
# spot it has reached. They get their own section in the window and a badge on
# the map at the pose where they happen.
FUNCTIONS = ["liftToLevel", "claw", "homeLift", "shootToGoal", "autoGrabPin"]
FUNCTION_COLORS = {k: "#d8d8d8" for k in FUNCTIONS}   # neutral: no colour per function


def _lift_level_names():
    """The lift's level names, read from src/lift_calibration.cpp so the planner
    always offers the levels the robot really has."""
    try:
        with open(os.path.join(HERE, "..", "src", "lift_calibration.cpp"), encoding="utf-8") as f:
            m = re.search(r"kLevelNames\[[^\]]*\]\s*=\s*\{([^}]*)\}", f.read())
        names = re.findall(r'"([^"]+)"', m.group(1)) if m else []
        if names:
            return names
    except OSError:
        pass
    return ["GROUND", "LOW", "MID", "HIGH", "MAX", "PIN"]


LIFT_LEVELS = _lift_level_names()
CLAW_WHICH = ["CLAW"]     # the robot has one claw (older plans may still say BOTH / CLAW 1 / CLAW 2)
CLAW_STATES = ["ON", "OFF"]                   # ON = extended, OFF = retracted

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
    "decelRange": (0.0, "num"), "decelSpeed": (40, "num"), "decelSteps": (1, "int"),
    "reach": (DEFAULT_ROBOT_IN / 2, "num"),     # goToPin: centre -> claw tip, inches
    "wallPin": (False, "bool"),                 # goToPin: face the pin, then drive straight through (no camera)
    "ms": (500, "int"),
    "level": (LIFT_LEVELS[0], LIFT_LEVELS), "position": ("LIFT", ["LIFT", "DROP"]),
    "waitLift": ("YES", ["YES", "NO"]),
    "goalSide": ("BOTTOM", GOAL_SIDES),           # shootAtGoal: which side of the goal to shoot from
    "isPin": ("NO", ["NO", "YES"]),                 # shootAtGoal: YES = a pin: lift to PIN, claws off
    "drive": ("SMOOTH", ["SMOOTH", "STRAIGHT"]),    # shootAtGoal: flow round the goals / aim each leg
    "standoff": (10.0, "num"),                    # (old plans: robot centre to goal centre; now computed from push)
    "push": (GOAL_PRESS_IN, "num"),               # shootAtGoal: inches the front is driven past touching the goal
    "shootLevel": ("CURRENT", ["CURRENT"] + LIFT_LEVELS),
    "grab": ("ON", ["ON", "OFF"]),                 # autoGrabPin: the IR sensor closes the claws
    "clawWhich": ("CLAW", CLAW_WHICH), "clawState": ("ON", CLAW_STATES),
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
    # decelRange > 0: slow to decelSpeed over the last decelRange inches. decelSteps 1 = one
    # step down (moveToPointDecel); 2+ = a ramp of that many stages (moveToPointSmooth)
    "moveToPoint": ["x", "y", "timeout", "forwards", "maxSpeed", "minSpeed", "earlyExitRange",
                    "decelRange", "decelSpeed", "decelSteps"],
    "moveToPose": ["x", "y", "theta", "timeout", "forwards", "horizontalDrift", "lead",
                   "maxSpeed", "minSpeed", "earlyExitRange"],
    "wait": ["ms"],
    "setPose": ["x", "y", "theta"],
    "waitUntilDone": [],
    "liftToLevel": ["level", "position", "waitLift", "timeout"],
    "claw": ["clawWhich", "clawState"],
    "homeLift": [],
    # fetch the pin at (x, y): open claws, lift to `level` while driving, close claws as soon as the
    # IR sensor sees the pin (or on arrival if it never does).
    # The claw tip -- `reach` inches ahead of the robot's centre -- ends on the pin, and the last
    # decelRange inches are driven at decelSpeed.
    "goToPin": ["x", "y", "level", "timeout", "forwards", "maxSpeed", "reach", "decelRange",
                "decelSpeed", "wallPin"],
    # drop at whatever level the lift is at when it runs (GROUND: just opens the claws)
    "shootToGoal": ["timeout"],
    # the IR sensor closes the claws by itself whenever it sees a pin (pinAutoGrabEnable)
    "autoGrabPin": ["grab"],
    # tap a goal, pick the side it is shot from: drives round the other goals, in to `standoff`
    # inches from this one, faces it, raises the lift to shootLevel and drops
    "shootAtGoal": ["x", "y", "goalSide", "push", "drive", "isPin", "shootLevel", "timeout"],
}
def bool_label(name, value):
    """The text of a yes/no parameter's checkbox: it follows the value."""
    if name == "wallPin":
        return "wall pin: face it, straight through" if value else "no wall"
    return "forwards" if value else "backwards"


HAS_POINT = {"turnToPoint", "swingToPoint", "moveToPoint", "moveToPose", "setPose", "goToPin"}
HAS_HEADING = {"turnToHeading", "swingToHeading", "moveToPose", "setPose"}


def new_block(kind, **overrides):
    b = {"kind": kind}
    for name in KIND_PARAMS[kind]:
        b[name] = PARAM_SPEC[name][0]
    if kind == "goToPin":       # a pin is fetched at the PIN lift level, with time to drive there
        b["timeout"] = 3000
        b["decelRange"], b["decelSpeed"] = 10.0, 40
        if "PIN" in LIFT_LEVELS:
            b["level"] = "PIN"
    b.update(overrides)
    return b


def fmt(v):
    return f"{v:.2f}" if isinstance(v, float) else str(v)


def level_index(b):
    """Index of the block's lift level in the robot's table (0 if it's gone)."""
    return LIFT_LEVELS.index(b["level"]) if b.get("level") in LIFT_LEVELS else 0


def block_code(b):
    """The C++ line(s) this block becomes -- same shape as the brain's
    formatStepCode(), so the export matches the on-screen preview."""
    k = b["kind"]
    if k == "wait":
        return f"pros::delay({int(b['ms'])});"
    if k == "waitUntilDone":
        # NOT chassis.waitUntilDone(): lemlib only ends that wait when a motion
        # *finishes*, so one reached before any motion has run never returns
        # and the rest of the routine is skipped.
        return "while (chassis.isInMotion()) pros::delay(10);"
    if k == "setPose":
        return f"chassis.setPose({b['x']:.2f}, {b['y']:.2f}, {b['theta']:.2f});"
    if k == "liftToLevel":
        wait = "true" if b["waitLift"] == "YES" else "false"
        if b["position"] == "DROP":     # to the drop angle, then the claws open; GROUND: just opens
            return f"liftDropAtLevel({level_index(b)}, {int(b['timeout'])});"
        return f"liftGoToLevel({level_index(b)}, {wait}, {int(b['timeout'])});"
    if k == "claw":
        if b["clawState"] == "ON":      # shut the claws; clawsClose() also lifts the lift slightly
            return "clawsClose();"
        return "claw.retract();"
    if k == "homeLift":
        return "homeLift();"
    if k == "shootAtGoal":      # (shown while it is placed; Apply turns it into the blocks below)
        return (f"// shoot at goal ({b['x']:g}, {b['y']:g}) from its {b['goalSide']}: drive round the "
                f"other goals, push {b['push']:g} in, face it, "
                + ("lift to PIN, claws off" if b.get("isPin") == "YES"
                   else f"{b['shootLevel']}, shootToGoal"))
    if k == "shootToGoal":
        return f"shootToGoal({int(b['timeout'])});"
    if k == "autoGrabPin":
        return f"pinAutoGrabEnable({'true' if b['grab'] == 'ON' else 'false'});"
    fwd = "true" if b.get("forwards", True) else "false"
    if k == "goToPin":
        return (f"goToPin({b['x']:.2f}, {b['y']:.2f}, {level_index(b)}, {int(b['timeout'])}, "
                f"{b['maxSpeed']:.0f}, {fwd}, {b.get('reach', 0.0):.2f}, "
                f"{b.get('decelRange', 0.0):.2f}, {b.get('decelSpeed', 40):.0f}"
                f"{', true' if b.get('wallPin') else ''});")
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
        if b.get("decelRange", 0) > 0 and b.get("decelSteps", 1) >= 2:
            return (f"moveToPointSmooth({b['x']:.2f}, {b['y']:.2f}, {t}, {o}, "
                    f"{b['decelRange']:.2f}, {b['decelSpeed']:.0f}, true, "
                    f"{min(8, int(b['decelSteps']))});")
        if b.get("decelRange", 0) > 0:
            return (f"moveToPointDecel({b['x']:.2f}, {b['y']:.2f}, {t}, {o}, "
                    f"{b['decelRange']:.2f}, {b['decelSpeed']:.0f});")
        return f"chassis.moveToPoint({b['x']:.2f}, {b['y']:.2f}, {t}, {o});"
    if k == "moveToPose":
        return f"chassis.moveToPose({b['x']:.2f}, {b['y']:.2f}, {b['theta']:.2f}, {t}, {o});"
    return f"// unknown block {k}"


DIR_ENUM = {"AUTO": "AUTO", "CW": "CW_CLOCKWISE", "CCW": "CCW_COUNTERCLOCKWISE"}


# One small symbol per kind of block, so the list can be read at a glance: arrows drive, circular
# arrows turn, the rest are what the robot does where it stops.
KIND_GLYPH = {"moveToPoint": "→", "moveToPose": "↝", "turnToHeading": "↻", "turnToPoint": "↻",
              "swingToHeading": "↷", "swingToPoint": "↷", "wait": "⏱", "setPose": "◎",
              "waitUntilDone": "·", "liftToLevel": "⇧", "claw": "◐", "homeLift": "⌂",
              "goToPin": "●", "shootToGoal": "✦", "autoGrabPin": "◉", "shootAtGoal": "✦"}
ROW_COLORS = {"drive": "#e8eaf0", "func": "#9db4ff", "quiet": "#6b7280"}


def row_color(kind):
    if kind == "waitUntilDone":
        return ROW_COLORS["quiet"]
    return ROW_COLORS["drive"] if kind in MOTIONS or kind in ("wait", "setPose") \
        else ROW_COLORS["func"]


def block_summary(b):
    k = b["kind"]
    if k == "wait":
        return f"wait {int(b['ms'])} ms"
    if k == "waitUntilDone":
        return "waitUntilDone"
    if k == "setPose":
        return f"setPose ({b['x']:.1f}, {b['y']:.1f}) hdg {b['theta']:.1f}"
    if k == "liftToLevel":
        if b["position"] == "DROP":
            return f"liftToLevel {b['level']} DROP" \
                + (" (open claws)" if level_index(b) == 0 else " (+ open claws)")
        return f"liftToLevel {b['level']} LIFT" + (" (wait)" if b["waitLift"] == "YES" else "")
    if k == "claw":
        return f"claw {b['clawState']}"
    if k == "homeLift":
        return "homeLift"
    if k == "shootToGoal":
        return "shootToGoal (drop at current level)"
    if k == "autoGrabPin":
        return f"autoGrabPin {b['grab']}"
    if k == "shootAtGoal":
        return (f"shootAtGoal ({b['x']:g}, {b['y']:g}) {b['goalSide']} {b['drive']}"
                + (" PIN" if b.get("isPin") == "YES" else ""))
    if k == "goToPin":
        return (f"goToPin ({b['x']:.1f}, {b['y']:.1f}) lift {b['level']}"
                + ("" if b.get("forwards", True) else " BWD") + (" WALL" if b.get("wallPin") else "")
                + f" {int(b['timeout'])}ms")
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
    if b["kind"] == "liftToLevel":      # dirIdx = level, sideIdx = 0 wait / 1 don't
        # ...and the last field (drift) is 1 for the DROP angle, 0 for the LIFT angle
        return "PLAN ADD {} 0 0 0 {} 1 {} {} 127 0 0 0.6 {}".format(
            k, t, level_index(b), 0 if b["waitLift"] == "YES" else 1,
            1 if b["position"] == "DROP" else 0)
    if b["kind"] == "claw":             # dirIdx = which claw, sideIdx = 0 extend / 1 retract
        return "PLAN ADD {} 0 0 0 0 1 {} {} 127 0 0 0.6 0".format(
            k, (CLAW_WHICH.index(b["clawWhich"]) if b["clawWhich"] in CLAW_WHICH else 0), CLAW_STATES.index(b["clawState"]))
    if b["kind"] == "autoGrabPin":      # sideIdx = 0 on / 1 off
        return "PLAN ADD {} 0 0 0 0 1 0 {} 127 0 0 0.6 0".format(k, 0 if b["grab"] == "ON" else 1)
    if b["kind"] == "shootToGoal":      # only the timeout matters
        return "PLAN ADD {} 0 0 0 {} 1 0 0 127 0 0 0.6 0".format(k, t)
    if b["kind"] == "goToPin":          # dirIdx = lift level; the rest as a moveToPoint
        # exit = reach (centre -> claw tip), lead = decelSpeed, drift = decelRange
        # (the side slot carries wallPin: 1 = square to the nearest wall first)
        return "PLAN ADD {} {:.2f} {:.2f} 0 {} {} {} {} {:.1f} 0 {:.2f} {:.1f} {:.2f}".format(
            k, b["x"], b["y"], t, 1 if b.get("forwards", True) else 0, level_index(b),
            1 if b.get("wallPin") else 0,
            b.get("maxSpeed", 127), b.get("reach", 0.0), b.get("decelSpeed", 40),
            b.get("decelRange", 0.0))
    lead, drift = b.get("lead", 0.6), b.get("horizontalDrift", 0.0)
    if b["kind"] == "moveToPoint":      # the brain reads the slow-down from these two fields
        lead, drift = b.get("decelSpeed", 40), b.get("decelRange", 0.0)
    side_field = SIDES.index(b.get("side", "LEFT"))
    if b["kind"] == "moveToPoint" and b.get("decelSteps", 1) >= 2:
        side_field = min(8, int(b["decelSteps"]))   # 2+ = ramp stages (brain: moveToPointSmooth)
    return ("PLAN ADD {} {:.2f} {:.2f} {:.2f} {} {} {} {} {:.1f} {:.1f} {:.2f} {:.2f} {:.2f}"
            .format(k, b.get("x", 0.0), b.get("y", 0.0), b.get("theta", 0.0), t,
                    1 if b.get("forwards", True) else 0,
                    DIRECTIONS.index(b.get("direction", "AUTO")),
                    side_field,
                    b.get("maxSpeed", 127), b.get("minSpeed", 0), b.get("earlyExitRange", 0.0),
                    lead, drift))


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


# Globals declared in motors.hpp: an auto function may not reuse these names.
RESERVED_NAMES = {"controller", "chassis", "lift", "claw", "claw2", "imu", "pinsensor", "liftlimitswitch", "liftbumper",
                  "left_motor_group", "right_motor_group", "verticalrotation", "horizontalrotation"}


def auto_function_name(stem):
    """A C++ function name from a plan's file name: letters, digits and
    underscores, never starting with a digit."""
    name = re.sub(r"\W+", "_", stem).strip("_").lower() or "planned_auto"
    if name in RESERVED_NAMES:      # would clash with a global of the same name (e.g. `controller`)
        name += "_auto"
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
    if any(b["kind"] in ("liftToLevel", "shootToGoal", "autoGrabPin", "goToPin") for b in blocks) and \
            '#include "lift_calibration.hpp"' not in cpp:
        cpp = '#include "lift_calibration.hpp"\n' + cpp      # liftGoToLevel() lives there
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


def list_planner_functions(cpp_path=AUTONOMOUS_CPP):
    """Names of the functions the planner wrote into autonomous.cpp (the ones
    between BEGIN/END planner markers), in file order."""
    try:
        with open(cpp_path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return []
    return re.findall(r"^// BEGIN planner: (\w+)", text, re.M)


_DIR_FROM_ENUM = {v: k for k, v in DIR_ENUM.items()}
# positional (non-option) arguments of each call, in order
_CALL_ARGS = {
    "turnToHeading": ["theta", "timeout"],
    "turnToPoint": ["x", "y", "timeout"],
    "swingToHeading": ["theta", "side", "timeout"],
    "swingToPoint": ["x", "y", "side", "timeout"],
    "moveToPoint": ["x", "y", "timeout"],
    "moveToPointDecel": ["x", "y", "timeout", "decelRange", "decelSpeed"],
    "moveToPointSmooth": ["x", "y", "timeout", "decelRange", "decelSpeed", "_async", "decelSteps"],
    "moveToPose": ["x", "y", "theta", "timeout"],
}


def _typed(name, text):
    """A call argument as the value type the block parameter wants."""
    text = text.strip()
    if name == "side":
        return text.split("::")[-1]
    return int(float(text)) if PARAM_SPEC[name][1] == "int" else float(text)


def parse_auto_line(line):
    """Turn one generated line of C++ back into a block (inverse of block_code).
    Returns the block, or None if the line isn't one the planner writes."""
    code, _, comment = line.partition("//")
    code = code.strip()
    if not code:
        return None
    m = re.fullmatch(r"clawsClose\((?:(true|false)(?:,\s*(true|false))?)?\);", code)
    if m:   # (older plans may pass which claws -- there is only one now)
        return new_block("claw", clawWhich="CLAW", clawState="ON")
    if re.fullmatch(r"(claw2?\.(extend|retract)\(\);\s*)+", code):
        calls = re.findall(r"(claw2?)\.(extend|retract)\(\)", code)
        return new_block("claw", clawWhich="CLAW",
                         clawState="ON" if calls[0][1] == "extend" else "OFF")
    if re.fullmatch(r"while\s*\(\s*chassis\.isInMotion\(\)\s*\)\s*pros::delay\(10\);", code):
        return new_block("waitUntilDone")
    m = re.fullmatch(r"(?:chassis\.|pros::)?(\w+)\((.*)\);", code)
    if not m:
        return None
    fn, args = m.group(1), m.group(2)
    if fn == "homeLift":
        return new_block("homeLift")
    if fn == "waitUntilDone":
        return new_block("waitUntilDone")
    if fn == "delay":
        return new_block("wait", ms=int(float(args)))
    opts = re.search(r"\{(.*)\}", args)
    pos = [a.strip() for a in (args[:opts.start()] + args[opts.end():] if opts else args).split(",")
           if a.strip()]
    if fn == "setPose":
        x, y, th = (float(a) for a in pos)
        return new_block("setPose", x=x, y=y, theta=th)
    if fn in ("liftGoToLevel", "liftDropAtLevel"):
        level = LIFT_LEVELS[min(int(pos[0]), len(LIFT_LEVELS) - 1)]
        if fn == "liftDropAtLevel":
            return new_block("liftToLevel", level=level, position="DROP", timeout=int(pos[1]))
        return new_block("liftToLevel", level=level, position="LIFT",
                         waitLift="YES" if pos[1] == "true" else "NO", timeout=int(pos[2]))
    if fn == "shootToGoal":
        return new_block("shootToGoal", timeout=int(float(pos[0])) if pos else 2000)
    if fn == "pinAutoGrabEnable":
        return new_block("autoGrabPin", grab="ON" if args.strip() == "true" else "OFF")
    if fn == "goToPin":     # goToPin(x, y, level, timeout, maxSpeed, forwards[, reach, decelRange, decelSpeed])
        vals = dict(x=float(pos[0]), y=float(pos[1]),
                    level=LIFT_LEVELS[min(int(pos[2]), len(LIFT_LEVELS) - 1)],
                    timeout=int(pos[3]), maxSpeed=float(pos[4]), forwards=pos[5] == "true")
        if len(pos) >= 9:   # older exports stop at forwards: reach 0, no slow-down
            vals.update(reach=float(pos[6]), decelRange=float(pos[7]), decelSpeed=float(pos[8]))
            vals["wallPin"] = len(pos) >= 10 and pos[9] == "true"
        else:
            vals.update(reach=0.0, decelRange=0.0)
        return new_block("goToPin", **vals)
    if fn not in _CALL_ARGS:
        return None
    kind ="moveToPoint" if fn in ("moveToPointDecel", "moveToPointSmooth") else fn
    vals = {}
    for name, text in zip(_CALL_ARGS[fn], pos):
        if not name.startswith("_"):
            vals[name] = _typed(name, text)
    if opts:
        for key, text in re.findall(r"\.(\w+)\s*=\s*([^,]+)", opts.group(1)):
            if key == "forwards":
                vals[key] = text.strip() == "true"
            elif key == "direction":
                vals[key] = _DIR_FROM_ENUM.get(text.strip().split("::")[-1], "AUTO")
            elif key in PARAM_SPEC:
                vals[key] = _typed(key, text)
    # a swing's backwards flag isn't exported to C++; the trailing comment keeps it
    if kind.startswith("swing") and re.search(r"\bBWD\b", comment):
        vals["forwards"] = False
    return new_block(kind, **{k: v for k, v in vals.items() if k in KIND_PARAMS[kind]})


def read_auto_function(name, cpp_path=AUTONOMOUS_CPP):
    """Read a planner-written function back out of autonomous.cpp.
    Returns (start, blocks, skipped): the start pose, the blocks, and the lines
    that couldn't be understood (e.g. hand-edited ones) and were left out."""
    with open(cpp_path, encoding="utf-8") as f:
        text = f.read().replace("\r\n", "\n")
    m = re.search(rf"^// BEGIN planner: {re.escape(name)}\b.*?^// END planner: "
                  rf"{re.escape(name)}", text, re.M | re.S)
    if not m:
        raise KeyError(name)
    start, blocks, skipped = [0.0, 0.0, 0.0], [], []
    for line in m.group(0).split("\n"):
        s = line.strip()
        if not s or s.startswith("//") or re.match(r"void\s+\w+\s*\(\s*\)\s*\{", s) or s == "}":
            continue
        if "// start pose" in s:
            nums = re.search(r"setPose\(([^)]*)\)", s)
            if nums:
                start = [float(v) for v in nums.group(1).split(",")]
            continue
        try:
            b = parse_auto_line(s)
        except (ValueError, IndexError):
            b = None
        if b is None:
            skipped.append(s)
        else:
            blocks.append(b)
    return start, blocks, skipped


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


def sim_move_to_point(robot, b, trace, log=None):
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
            if not close:
                # lemlib 0.5.x: once within 7.5 in the speed limit is REPLACED by
                # max(|last forward power|, 60) -- so a decelSpeed below 60 stops holding there
                closeCap = max(abs(prevLat), 60)
            close = True
        if b["minSpeed"] != 0 and d < b.get("earlyExitRange", 0):
            break       # lemlib: earlyExitRange only counts while minSpeed != 0
        angErr = angle_err(heading_to(x, y, tx, ty), eff)
        latErr = d * math.cos(math.radians(angErr))
        if not fwd:
            latErr = -latErr
        angOut = 0.0 if close else ang.update(angErr)
        latOut = slew(lat.update(latErr), prevLat, SPEC.lateral["slew"])
        top = b["maxSpeed"]
        if close and not b.get("decelRange", 0) > 0:
            top = closeCap      # (a decel move's last stretch is the brain's own approach: it holds)
        elif b.get("decelRange", 0) > 0 and d < b["decelRange"]:
            slow, steps = b.get("decelSpeed", 40), int(b.get("decelSteps", 1))
            if steps >= 2:      # ramp: the cap falls stage by stage (moveToPointSmooth)
                steps = min(8, steps)
                i = min(steps, int((b["decelRange"] - d) / b["decelRange"] * steps) + 1)
                top = min(top, slow + (b["maxSpeed"] - slow) * (steps - i) / steps)
            else:
                top = min(top, slow)                    # slowing down near the target
        latOut, angOut = _apply_speed_limits(latOut, angOut, top, b["minSpeed"], fwd, close)
        prevLat = latOut
        if small.update(latErr, t) or large.update(latErr, t):
            break
        robot.step(latOut + angOut, latOut - angOut)
        if log is not None:     # (ms, forward power, turn power, speed in/s, error)
            log.append((t, latOut, angOut, (robot.vl + robot.vr) / 2, latErr))
        t += LOOP_S * 1000
        trace.append((robot.x, robot.y))
    return t


def sim_move_to_pose(robot, b, trace, log=None):
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
        if b["minSpeed"] != 0 and d < b.get("earlyExitRange", 0):
            break       # lemlib: earlyExitRange only counts while minSpeed != 0
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
        if log is not None:     # (ms, forward power, turn power, speed in/s, error)
            log.append((t, latOut, angOut, (robot.vl + robot.vr) / 2, latErr))
        t += LOOP_S * 1000
        trace.append((robot.x, robot.y))
    return t


def sim_turn(robot, b, trace, targetHeading, swingSide=None, headings=None, log=None):
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
        if log is not None:
            log.append((t, 0.0, out, (robot.vl + robot.vr) / 2, err))
        t += LOOP_S * 1000
        trace.append((robot.x, robot.y))
        if headings is not None:
            headings.append(robot.th)       # one per trace point: the footprint's rotation
    return t


def simulate(blocks, start, logs=None, times=None):
    """Runs the block list through the simulator from `start` = (x, y,
    theta). Returns, per block, (endPose, shape):
        ("trace", [(x, y), ...])          the path the robot centre takes
        ("turn", x, y, fromHdg, toHdg)    an in-place turn (drawn as an arc)
        ("swing", x, y, fromHdg, toHdg, trace, headings)  a swing: arc + the centre's
                                          path, and the heading at each point of it
        None                              nothing on the map (wait, ...)
    Each block starts where the previous one ended, so the shapes chain
    into the full route. With a list as `logs`, one entry per block is appended: the motion's
    per-loop samples (ms, forward power, turn power, speed in/s, error), [] for non-motions."""
    robot = Robot(*start)
    out = []
    for b in blocks:
        k = b["kind"]
        shape = None
        dur = 0
        log = [] if logs is not None else None
        trace = [(robot.x, robot.y)]
        x0, y0, th0 = robot.pose()
        if k == "setPose":
            robot = Robot(b["x"], b["y"], b["theta"])
        elif k == "moveToPoint":
            dur = sim_move_to_point(robot, b, trace, log)
            shape = ("trace", trace)
        elif k == "goToPin":        # face the pin, then a moveToPoint that stops `reach` short of it
            tx, ty, reach = b["x"], b["y"], b.get("reach", 0.0)
            dist = math.hypot(tx - x0, ty - y0)
            fwd = b.get("forwards", True)
            if fwd and reach > 0 and dist <= reach:
                pass                # the claw is already on the pin
            else:
                if fwd and reach > 0:
                    tx, ty = tx - (tx - x0) / dist * reach, ty - (ty - y0) / dist * reach
                    turnT = sim_turn(robot, dict(b, direction="AUTO", minSpeed=0, earlyExitRange=0,
                                                 timeout=1000),
                                     trace, heading_to(x0, y0, b["x"], b["y"]), None, None, log=log)
                else:
                    turnT = 0
                moveLog = [] if log is not None else None
                dur = turnT + sim_move_to_point(robot, dict(b, x=tx, y=ty, minSpeed=0,
                                                            earlyExitRange=0), trace, moveLog)
                if log is not None:
                    log.extend((row[0] + turnT,) + row[1:] for row in moveLog)
            shape = ("trace", trace)
        elif k == "shootAtGoal":
            subTimes = []
            sub = shoot_blocks(b, robot.pose())
            res = simulate(sub, robot.pose(), times=subTimes)
            for _pose, sh in res:
                if sh and sh[0] in ("trace", "swing"):
                    trace.extend(sh[1] if sh[0] == "trace" else sh[5])
            if res:
                robot = Robot(*res[-1][0])
            dur = sum(subTimes)
            shape = ("trace", trace)
        elif k == "moveToPose":
            dur = sim_move_to_pose(robot, b, trace, log)
            shape = ("trace", trace)
        elif k in ("turnToHeading", "turnToPoint", "swingToHeading", "swingToPoint"):
            if k.endswith("Heading"):
                target = b["theta"]
            else:
                target = heading_to(x0, y0, b["x"], b["y"], b.get("forwards", True)) \
                    if (b["x"], b["y"]) != (x0, y0) else th0
            swingSide = b.get("side") if k.startswith("swing") else None
            heads = [th0]
            dur = sim_turn(robot, b, trace, target, swingSide, heads, log)
            if swingSide:
                shape = ("swing", x0, y0, th0, robot.th, trace, heads)
            else:
                shape = ("turn", x0, y0, th0, robot.th)
        out.append((robot.pose(), shape))
        if times is not None:
            times.append(dur)
        if logs is not None:
            logs.append(log)
    return out


PLAN_CTX = {"size": (DEFAULT_ROBOT_IN, DEFAULT_ROBOT_IN), "avoid": True, "exit": 6.0, "auto": True,
            "front": DEFAULT_ROBOT_IN / 2, "back": DEFAULT_ROBOT_IN / 2}
# ^ what an expansion needs from the window: the robot's (length, width), whether goals are avoided,
#   the early-exit range of a smooth turn, whether timeouts are picked automatically. The window
#   refreshes it before it simulates, uploads or exports.

TIMED_KINDS = MOTIONS | {"goToPin"}     # the blocks whose timeout the planner can work out


def auto_timeout_ms(b, pose):
    """A timeout that suits block `b` started from `pose`: how long the simulated robot takes to
    finish it, with 35% to spare and 300 ms on top, to the next 50 ms (500 ms .. 8 s). The block
    is simulated with a very long timeout so a too-short current one can't cut the measurement."""
    times = []
    simulate([dict(b, timeout=12000)], pose, times=times)
    return int(max(500, min(8000, math.ceil((times[0] * 1.35 + 300) / 50.0) * 50)))


def shoot_standoff(b, size, front=None):
    """Robot centre to goal centre for a shot: its front (`front` inches ahead of the odometry
    centre -- half the length unless measured) at the goal's edge, then driven `push` inches
    further in, so it presses on the goal."""
    front = PLAN_CTX.get("front", size[0] / 2.0) if front is None else front
    return max(1.0, front + GOAL_R - float(b.get("push", GOAL_PRESS_IN)))


def shoot_blocks(b, pose, ctx=None):
    """The ordinary blocks a Shoot at goal block `b` stands for when the robot starts it from
    `pose`: round the other goals (SMOOTH drives through those points, STRAIGHT aims down each
    leg first), out to the point on the chosen side, face the goal, drive in straight, face it
    again to square up, then lift to PIN and claws off (a pin) or shootToGoal."""
    ctx = ctx or PLAN_CTX
    size, avoid, exitRange = ctx["size"], ctx["avoid"], ctx["exit"]
    goal = (float(b["x"]), float(b["y"]))
    pre, stand = shoot_points(goal, b["goalSide"], shoot_standoff(b, size), goal_clearance(*size))
    wps = avoid_route((pose[0], pose[1]), pre, size) if avoid else []
    smooth = b.get("drive", "SMOOTH") == "SMOOTH"
    seq = []
    heading, prev = pose[2], (pose[0], pose[1])

    def goalTurn():
        return [new_block("turnToPoint", x=goal[0], y=goal[1]), new_block("waitUntilDone")]

    for i, p in enumerate(wps + [pre]):
        want = heading_to(prev[0], prev[1], p[0], p[1])
        flow = smooth and i < len(wps)
        if not smooth and abs((want - heading + 180) % 360 - 180) > 3.0:
            seq += [new_block("turnToHeading", theta=round(want, 2)), new_block("waitUntilDone")]
        seq += [new_block("moveToPoint", x=round(p[0], 1), y=round(p[1], 1),
                          minSpeed=40 if flow else 0, earlyExitRange=exitRange if flow else 0.0),
                new_block("waitUntilDone")]
        heading, prev = want, p
    # face the goal BEFORE the last run in, so that run is a straight line along the heading
    seq += goalTurn()
    seq += [new_block("moveToPoint", x=round(stand[0], 1), y=round(stand[1], 1)),
            new_block("waitUntilDone")]
    seq += goalTurn()
    if b.get("isPin") == "YES":         # a pin: lift to the PIN position, then claws off
        level = "PIN" if "PIN" in LIFT_LEVELS else (b["shootLevel"] if b["shootLevel"] != "CURRENT"
                                                    else LIFT_LEVELS[0])
        seq.append(new_block("liftToLevel", level=level, position="LIFT", waitLift="YES",
                             timeout=2000))
        seq.append(new_block("claw", clawWhich="CLAW", clawState="OFF"))
    else:
        if b["shootLevel"] != "CURRENT":
            seq.append(new_block("liftToLevel", level=b["shootLevel"], position="LIFT",
                                 waitLift="YES", timeout=2000))
        seq.append(new_block("shootToGoal", timeout=int(b["timeout"])))
    if ctx.get("auto", True):           # each drive / turn gets the timeout that suits it
        p = tuple(pose)
        for sb in seq:
            if sb["kind"] in TIMED_KINDS:
                sb["timeout"] = auto_timeout_ms(sb, p)
            p = simulate([sb], p)[0][0]
    return seq


# Rough seconds the robot spends on the blocks that are not driven or turned (the simulator times
# those): lift travel, the claws, a drop, homing. Real values depend on the robot; adjust to taste.
EXTRA_S = {"claw": 0.15, "shootToGoal": 0.8, "homeLift": 0.9, "goToPin": 1.1}
MOTION_OVERHEAD_S = 0.05        # per driven / turned block: the hand-over to the next one
AUTO_PERIOD_S = 30.0            # the length of a VEX U autonomous period


CALIBRATION_FILE = os.path.join(HERE, "time_calibration.json")


def _load_calibration():
    try:
        with open(CALIBRATION_FILE, encoding="utf-8") as f:
            d = json.load(f)
        return {"runs": [float(v) for v in d.get("runs", [])][-10:],
                "extra_samples": {k: [float(v) for v in vs][-10:]
                                  for k, vs in d.get("extra_samples", {}).items()}}
    except (OSError, ValueError, AttributeError, TypeError):
        return {"runs": [], "extra_samples": {}}


CAL = _load_calibration()


def _median(vs):
    vs = sorted(vs)
    n = len(vs)
    return vs[n // 2] if n % 2 else (vs[n // 2 - 1] + vs[n // 2]) / 2.0


def drive_scale():
    """How much longer (or shorter) the real robot takes to drive and turn than the simulator,
    learned from the whole-plan runs timed on it. 1.0 until it has run something."""
    return max(0.5, min(2.0, _median(CAL["runs"]))) if CAL["runs"] else 1.0


def extra_seconds(kind):
    """What a lift / claw / drop step really takes, if it has been timed; else the default."""
    samples = CAL["extra_samples"].get(kind)
    return _median(samples) if samples else EXTRA_S.get(kind, 0.0)


def update_calibration(flat, start, step_times):
    """Learn from one completed whole-plan run: `step_times` is robot step -> seconds. A motion
    is async on the robot, so its time lands on the waitUntilDone steps right after it; only
    those motions are compared with the simulator. Returns how many motions it learned from."""
    times = []
    simulate(flat, tuple(start), times=times)
    ratios = []
    n = len(flat)
    for j, b in enumerate(flat):
        sim_s = times[j] / 1000.0 if j < len(times) else 0.0
        if b["kind"] in MOTIONS and sim_s >= 0.3 and j + 1 < n \
                and flat[j + 1]["kind"] == "waitUntilDone":
            real, k = step_times.get(j, 0.0), j + 1
            while k < n and flat[k]["kind"] == "waitUntilDone":
                real += step_times.get(k, 0.0)
                k += 1
            if real > 0:
                ratio = real / (sim_s + MOTION_OVERHEAD_S)
                if 0.3 <= ratio <= 3.0:
                    ratios.append(ratio)
        elif b["kind"] in ("claw", "shootToGoal", "homeLift"):
            t = step_times.get(j, 0.0)
            if 0.02 < t < 6.0:
                CAL["extra_samples"].setdefault(b["kind"], []).append(round(t, 3))
                del CAL["extra_samples"][b["kind"]][:-10]
    if ratios:
        CAL["runs"].append(round(_median(ratios), 4))
        del CAL["runs"][:-10]
    try:
        with open(CALIBRATION_FILE, "w", encoding="utf-8") as f:
            json.dump(CAL, f, indent=1)
    except OSError:
        pass
    return len(ratios)


def step_drift(flat, start, step_poses):
    """flat index of a motion -> inches between where the robot really was when it finished
    (the end of the waitUntilDone steps after it) and where the simulator says it ends."""
    sim = simulate(flat, tuple(start))
    out = {}
    n = len(flat)
    for j, b in enumerate(flat):
        if b["kind"] in MOTIONS and j < len(sim):
            k = j
            while k + 1 < n and flat[k + 1]["kind"] == "waitUntilDone":
                k += 1
            pose = step_poses.get(k)
            if pose and k > j:
                ex, ey = sim[j][0][0], sim[j][0][1]
                out[j] = math.hypot(pose[0] - ex, pose[1] - ey)
    return out


def estimate_seconds(flat, start):
    """(total, driving, other) seconds the plan should take: the simulator's time for every drive
    and turn, plus a fixed allowance for lifts, claws, drops and waits. An estimate: the real
    robot is a little slower than the simulator on long moves and faster on short ones."""
    times = []
    simulate(flat, tuple(start), times=times)
    driving = other = 0.0
    for b, ms in zip(flat, times):
        k = b["kind"]
        if k in TIMED_KINDS:
            driving += ms / 1000.0 * drive_scale() + MOTION_OVERHEAD_S
            if k == "goToPin":
                other += EXTRA_S["goToPin"]
        elif k == "wait":
            other += b["ms"] / 1000.0
        elif k == "liftToLevel":
            if b.get("waitLift") == "YES" or b.get("position") == "DROP":
                other += 0.8
        else:
            other += extra_seconds(k)
    return driving + other, driving, other


def expand_plan(blocks, start):
    """(flat, spans): the blocks as the robot runs them -- every Shoot at goal replaced by the
    blocks it stands for -- and, per original block, its (first, end) range in `flat`."""
    sim = simulate(blocks, tuple(start))
    flat, spans = [], []
    for i, b in enumerate(blocks):
        lo = len(flat)
        if b["kind"] == "shootAtGoal":
            flat += shoot_blocks(b, sim[i - 1][0] if i > 0 else tuple(start))
        else:
            flat.append(b)
        spans.append((lo, len(flat)))
    return flat, spans


def _wrap_like(v, like):
    """v as an angle in the same style as `like`: -180..180 if that is how it was written, else
    0..360."""
    v %= 360.0
    if like < 0 or 0 <= like <= 180:
        return round(v - 360.0 if v > 180.0 else v, 2)
    return round(v, 2)


FLIP_AXES = {"rotate": (True, True), "mirror": (True, False), "mirror_y": (False, True)}

def flip_plan(blocks, start, mode="rotate"):
    """(blocks, start) for the same plan somewhere else on the field.
    "rotate":   turned half a way round the centre (x, y -> -x, -y, heading + 180). That is how
                the two alliances' sides of this field map onto each other: the red goals land on
                the blue ones and the pins land on pins. Turns and swings keep their direction.
    "mirror":   left and right swapped (x -> -x, heading -> -heading).
    "mirror_y": top and bottom swapped (y -> -y, heading -> 180 - heading).
    A mirror reverses turning, so CW <-> CCW and a swing's locked side swap too."""
    fx, fy = FLIP_AXES[mode]
    reverses = fx != fy
    swap = {"CW": "CCW", "CCW": "CW", "LEFT": "RIGHT", "RIGHT": "LEFT"}
    side_x = {"LEFT": "RIGHT", "RIGHT": "LEFT"}
    side_y = {"TOP": "BOTTOM", "BOTTOM": "TOP"}

    def heading(th):
        if fx and fy:
            return _wrap_like(th + 180.0, th)
        return _wrap_like(-th if fx else 180.0 - th, th)

    out = []
    for b in blocks:
        n = dict(b)
        k = n["kind"]
        if k in HAS_POINT or k == "shootAtGoal":
            if fx:
                n["x"] = round(0.0 - n["x"], 2)
            if fy:
                n["y"] = round(0.0 - n["y"], 2)
        if k in HAS_HEADING:
            n["theta"] = heading(n["theta"])
        if reverses:
            if n.get("direction") in swap:
                n["direction"] = swap[n["direction"]]
            if n.get("side") in swap:
                n["side"] = swap[n["side"]]
        if k == "shootAtGoal":
            if fx and n.get("goalSide") in side_x:
                n["goalSide"] = side_x[n["goalSide"]]
            if fy and n.get("goalSide") in side_y:
                n["goalSide"] = side_y[n["goalSide"]]
        out.append(n)
    x, y, th = start
    new_start = [round(0.0 - x, 2) if fx else x, round(0.0 - y, 2) if fy else y, heading(th)]
    return out, new_start


def flip_report(blocks):
    """What a flipped plan no longer lines up with: a pin or a goal that is not there."""
    out = []
    for i, b in enumerate(blocks):
        if b["kind"] == "goToPin" and not any(math.hypot(b["x"] - px, b["y"] - py) <= 4.0
                                              for px, py in PINS):
            out.append(f"Block {i + 1} (goToPin): there is no pin near ({b['x']:g}, {b['y']:g}).")
        if b["kind"] == "shootAtGoal" and not any(math.hypot(b["x"] - gx, b["y"] - gy) <= 3.0
                                                  for gx, gy, _c in GOALS):
            out.append(f"Block {i + 1} (shootAtGoal): there is no goal at ({b['x']:g}, "
                       f"{b['y']:g}).")
    return out


def plan_warnings(blocks, start):
    """Plain-English problems in a plan, found by simulating it: a route that leaves the field or
    runs through a goal, a number that is not a number, a plan longer than the autonomous period.
    A warning is not an error -- the plan may be meant that way -- it is a last look before a run."""
    out = []
    if not blocks:
        return out
    half = FIELD_IN / 2
    for i, b in enumerate(blocks):
        for key in ("x", "y", "theta", "ms", "timeout"):
            v = b.get(key)
            if isinstance(v, float) and not math.isfinite(v):
                out.append(f"Block {i + 1} ({b['kind']}) has an invalid {key}.")
    if any(not math.isfinite(float(v)) for v in start):
        return out + ["The start pose is not a number."]
    sim = simulate(blocks, tuple(start))
    if abs(start[0]) > half or abs(start[1]) > half:
        out.append("The start pose is outside the field.")
    for i, (b, (_pose, shape)) in enumerate(zip(blocks, sim)):
        pts = []
        if shape and shape[0] == "trace":
            pts = shape[1]
        elif shape and shape[0] == "swing":
            pts = shape[5]
        # the target itself too: a move cut short by its timeout may never get there
        target_out = b["kind"] in HAS_POINT and b["kind"] != "setPose" and             (abs(b["x"]) > half or abs(b["y"]) > half)
        if target_out or any(abs(x) > half or abs(y) > half for x, y in pts):
            out.append(f"Block {i + 1} ({b['kind']}) takes the robot outside the field.")
        if b["kind"] in ("moveToPoint", "moveToPose"):
            for gx, gy, _c in GOALS:
                if any(math.hypot(x - gx, y - gy) < GOAL_R for x, y in pts):
                    out.append(f"Block {i + 1} ({b['kind']}) drives through the goal at "
                               f"({gx:g}, {gy:g}).")
                    break
    flat, _spans = expand_plan(blocks, tuple(start))
    total = estimate_seconds(flat, start)[0]
    if total > AUTO_PERIOD_S:
        out.append(f"The estimated time is {total:.1f} s, over the {AUTO_PERIOD_S:.0f} s "
                   "autonomous period.")
    return out


AUTOSAVE_PATH = os.path.join(HERE, ".autosave_plan.json")    # the unsaved plan, kept every 30 s
AUTOSAVE_MS = 30000
