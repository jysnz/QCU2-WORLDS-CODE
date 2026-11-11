"""Run analytics: turns the robot's recording of a run (src/run_log.cpp, sent as LOG| lines) into a
short report -- whether the IMU and the rotation sensors agreed with the drive motors (and in which
step they did not), how long each step took, how far from the plan each move ended, and, for every
goToPin, whether it drove straight, where the claw tip was when the sensor saw the pin and when it
stopped, and what looks wrong. The report is plain text so it can be pasted to whoever is
diagnosing the run.

No window and no robot needed:  python -m unittest discover tools/tests
"""

import json
import math
import os
import re
import time

import planner_core as pc

HERE = os.path.dirname(os.path.abspath(__file__))
REPORTS_DIR = os.path.join(HERE, "run_reports")
KEEP_REPORTS = 40

FLAG_PIN, FLAG_CLAW, FLAG_CLAW2, FLAG_MOVING, FLAG_WATCH = 1, 2, 4, 8, 16


# ── reading the LOG| lines ──────────────────────────────────────────────────────────────────────
class LogReader:
    """Collects the LOG| lines of one recording. feed() returns the finished run once every line
    of it has arrived, otherwise None.

    The robot numbers its lines ("LOG|#<run>.<line>|...") and keeps the recording until the next
    run, so lines the link dropped can be asked for again: need_command() is the request to send,
    take() gives up and returns whatever did arrive."""

    def __init__(self):
        self.run = None

    @property
    def active(self):
        """A recording has started arriving and is not complete yet."""
        return self.run is not None

    def _new(self, rid):
        self.run = {"id": rid, "name": "?", "status": "?", "samples": [], "events": [],
                    "segments": [], "health": {}, "period_ms": 20, "truncated": False,
                    "expected": (0, 0), "lines": None, "_got": set(), "_ms": set(),
                    "_numbered": False}

    def received(self):
        return len(self.run["_got"]) if self.run else 0

    def missing(self):
        """Line numbers still to come; None when the robot does not number its lines."""
        run = self.run
        if run is None or not run["_numbered"]:
            return None
        if run["lines"] is None:
            return [0]                      # the BEGIN line says how many there are
        return [n for n in range(run["lines"]) if n not in run["_got"]]

    def need_command(self, limit=120):
        """The request for the lines still missing, as many as fit in one command:
        LOG NEED <run> 3-9 12 ...   (None when nothing can be asked for)."""
        miss = self.missing()
        if not miss:
            return None
        cmd = f"LOG NEED {self.run['id']}"
        i = 0
        while i < len(miss):
            j = i
            while j + 1 < len(miss) and miss[j + 1] == miss[j] + 1:
                j += 1
            tok = f" {miss[i]}" if i == j else f" {miss[i]}-{miss[j]}"
            if len(cmd) + len(tok) > limit:
                break
            cmd += tok
            i = j + 1
        return cmd

    def take(self):
        """Gives up waiting: the recording as far as it arrived (None if nothing did)."""
        run, self.run = self.run, None
        if run is None or not (run["samples"] or run["events"] or run["segments"]
                               or run["health"]):
            return None
        return self._finish(run, complete=False)

    @staticmethod
    def _finish(run, complete=True):
        run["samples"].sort(key=lambda s: s["ms"])
        run["events"].sort(key=lambda e: (e["ms"], e.get("n", 0)))
        run["segments"].sort(key=lambda g: g["ms"])
        got = len(run.pop("_got"))
        run.pop("_ms")
        run.pop("_numbered")
        run["complete"] = complete
        if not complete:
            run["arrived"] = (got, run["lines"])
            if run["status"] == "?":
                run["status"] = "INCOMPLETE"
        return run

    def feed(self, line):
        line = line.strip()
        if not line.startswith("LOG|"):
            return None
        parts = line.split("|")
        seq = None
        try:
            if parts[-1].startswith("*"):                       # "|*<hh>": the line's checksum
                body = line[:line.rindex("|*")]
                if sum(body.encode("utf-8")) & 0xFF != int(parts[-1][1:], 16):
                    return None                                 # garbled on the way: ask again
                parts = parts[:-1]
            if len(parts) > 2 and parts[1].startswith("#"):     # "#<run>.<line>"
                rid, seq = (int(v) for v in parts[1][1:].split("."))
                if self.run is None or self.run["id"] != rid:
                    self._new(rid)
                    self.run["_numbered"] = True
                if seq in self.run["_got"]:
                    return None                                 # a line sent twice
                parts = parts[:1] + parts[2:]
            kind = parts[1] if len(parts) > 1 else ""
            if kind == "BEGIN" and seq is None:
                self._new(int(parts[2]))
            run = self.run
            if run is None:
                return None
            if kind == "BEGIN":
                run["name"], run["status"] = parts[3], parts[4]
                run["expected"] = (int(parts[5]), int(parts[7]))
                run["period_ms"] = int(parts[6])
                run["truncated"] = parts[8] == "1"
                if len(parts) > 9:
                    run["lines"] = int(parts[9])
            elif kind == "H":
                for kv in parts[2:]:
                    k, _, v = kv.partition("=")
                    run["health"][k] = float(v)
            elif kind == "G":
                ms, step = int(parts[2]), int(parts[3])
                imu, whl, vert, mot, hor, tilt = (float(v) for v in parts[4:10])
                run["segments"].append({"ms": ms, "step": step, "imu_deg": imu, "wheel_deg": whl,
                                        "vert_in": vert, "motor_in": mot, "hor_in": hor,
                                        "tilt_deg": tilt, "still_ms": int(parts[10]),
                                        "still_deg": float(parts[11])})
            elif kind == "E":
                ms, step, what, x, y, th = parts[2:8]
                run["events"].append({"ms": int(ms), "step": int(step), "what": what,
                                      "x": float(x), "y": float(y), "th": float(th),
                                      "n": seq if seq is not None else len(run["events"])})
            elif kind == "S":
                new = []
                for chunk in parts[2].split(";"):
                    v = chunk.split(",")
                    if len(v) not in (6, 9):    # (9 = the first version, with lift and wheels)
                        raise ValueError(chunk)
                    new.append({"ms": int(v[0]), "x": float(v[1]), "y": float(v[2]),
                                "th": float(v[3]), "step": int(v[4]), "flags": int(v[5])})
                for smp in new:
                    if smp["ms"] not in run["_ms"]:
                        run["_ms"].add(smp["ms"])
                        run["samples"].append(smp)
            elif kind == "END":
                if seq is None:                 # an unnumbered recording ends here
                    self.run = None
                    return self._finish(run)
                if len(parts) > 3 and run["lines"] is None:
                    run["lines"] = int(parts[3])
            else:
                return None
            if seq is not None:
                run["_got"].add(seq)
                if run["lines"] is not None and len(run["_got"]) >= run["lines"]:
                    self.run = None
                    return self._finish(run)
        except (ValueError, TypeError, IndexError):
            pass                        # a garbled line costs one line, not the recording
        return None


def demo_lines(sway=0.0, sees=True, speed=30.0, pin=(0.0, 40.0), reach=7.5, start_heading=0.0,
               imu_extra=0.0, vert_scale=1.0, drift_rate=0.0):
    """LOG| lines of a made-up goToPin from (0, 0), in the exact format the robot sends: it drives
    toward `pin`, bulging sideways by up to `sway` inches, and stops with the claw tip on the pin.
    `imu_extra` (deg) makes the IMU disagree with the drive wheels, `vert_scale` the forward
    tracking wheel with the drive motors, `drift_rate` (deg/s) is the IMU drift correction.
    Used by the tests and by Robot > Sample run report."""
    stop = pin[1] - reach
    samples, events = [], []
    ms, along, seen = 0, 0.0, False
    events.append((0, 0, "step 0 begin", 0, 0, 0))
    events.append((10, 0, f"pin target x={pin[0]:.2f} y={pin[1]:.2f} reach={reach:.2f} level=5",
                   0, 0, 0))
    while along < stop:
        x = sway * math.sin(math.pi * along / stop)
        heading = start_heading * (1 - along / stop)
        flags = FLAG_MOVING | (FLAG_PIN if sees and along + reach >= pin[1] - 3.0 else 0)
        if flags & FLAG_PIN and not seen:
            seen = True
            events.append((ms, 0, "pin seen: claws closing", x, along, heading))
        samples.append((ms, x, along, heading, 0, flags | (FLAG_WATCH if seen else 0)))
        ms += 20
        along += speed * 0.02
    samples.append((ms, 0.0, stop, 0.0, 0, FLAG_PIN if sees else 0))
    events.append((ms, 0, f"pin drive ended seen={1 if seen else 0}", 0, stop, 0))
    events.append((ms + 90, 0, "pin claws closed", 0, stop, 0))
    samples.append((ms + 100, 0.0, stop, 0.0, 0, FLAG_PIN if sees else 0))
    turn = samples[-1][3] - samples[0][3]
    body = ["H|imu=1|vert=1|hor=1|imubad=0|vbad=0|hbad=0|mbad=0|imujump=0|vjump=0|hjump=0|"
            f"rate0={drift_rate:.4f}|rate1={drift_rate:.4f}|drift0=0.000|"
            f"drift1={drift_rate * ms / 1000:.3f}|hoff=-1.472|track=10.25|ms={ms + 100}",
            f"G|0|0|{turn + imu_extra:.2f}|{turn * 1.05:.2f}|{stop * vert_scale:.2f}|{stop:.2f}|"
            f"0.10|1.2|100|{drift_rate * 0.1:.3f}"]
    body += [f"E|{e[0]}|{e[1]}|{e[2]}|{e[3]:.2f}|{e[4]:.2f}|{e[5]:.1f}" for e in events]
    for i in range(0, len(samples), 5):
        body.append("S|" + ";".join(
            f"{s[0]},{s[1]:.2f},{s[2]:.2f},{s[3]:.1f},{s[4]},{s[5]}" for s in samples[i:i + 5]))
    total = len(body) + 2
    body.insert(0, f"BEGIN|1|plan|DONE|{len(samples)}|20|{len(events)}|0|{total}|1")
    body.append(f"END|1|{total}")
    lines = [f"LOG|#1.{n}|{b}" for n, b in enumerate(body)]
    lines = [f"{ln}|*{sum(ln.encode('utf-8')) & 0xFF:02X}" for ln in lines]
    return lines


# ── small geometry helpers ──────────────────────────────────────────────────────────────────────
def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _bearing(x0, y0, x1, y1):
    """Compass heading from (x0, y0) to (x1, y1): 0 = +y, clockwise."""
    return math.degrees(math.atan2(x1 - x0, y1 - y0))


def _tip(s, reach):
    r = math.radians(s["th"])
    return s["x"] + math.sin(r) * reach, s["y"] + math.cos(r) * reach


def _to_robot_frame(s, px, py, origin=None):
    """(ahead, right): where point (px, py) is relative to the robot, in inches."""
    ox, oy = origin if origin else (s["x"], s["y"])
    dx, dy = px - ox, py - oy
    r = math.radians(s["th"])
    return dx * math.sin(r) + dy * math.cos(r), dx * math.cos(r) - dy * math.sin(r)


def _speeds(samples):
    """in/s at each sample, from the pose (independent of the wheel size), lightly smoothed."""
    out = []
    for i in range(len(samples)):
        a, b = samples[max(0, i - 2)], samples[min(len(samples) - 1, i + 2)]
        dt = (b["ms"] - a["ms"]) / 1000.0
        out.append(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dt if dt > 0 else 0.0)
    return out


def _at(samples, ms):
    """The sample nearest in time to `ms`."""
    return min(samples, key=lambda s: abs(s["ms"] - ms)) if samples else None


# ── goToPin ─────────────────────────────────────────────────────────────────────────────────────
_PIN_RE = re.compile(r"pin target x=(-?[\d.]+) y=(-?[\d.]+) reach=(-?[\d.]+)")


def analyze_pins(run):
    """One dict per goToPin in the recording: what the approach did, and what looks wrong."""
    events, samples = run["events"], run["samples"]
    speeds = _speeds(samples)
    out = []
    for n, ev in enumerate(events):
        m = _PIN_RE.match(ev["what"])
        if not m:
            continue
        pin = (float(m.group(1)), float(m.group(2)))
        reach = float(m.group(3))
        t0 = ev["ms"]
        later = [e for e in events if e["ms"] > t0]
        close = next((e for e in later if e["what"].startswith("pin claws closed")), None)
        nxt = next((e for e in later if _PIN_RE.match(e["what"])), None)
        t1 = close["ms"] if close else (nxt["ms"] if nxt else samples[-1]["ms"] if samples else t0)
        win = [(s, speeds[i]) for i, s in enumerate(samples) if t0 <= s["ms"] <= t1]
        if len(win) < 2:
            continue
        ss = [w[0] for w in win]
        seen_ev = next((e for e in later if e["what"].startswith("pin seen") and e["ms"] <= t1),
                       None)
        ended = next((e for e in later if e["what"].startswith("pin drive ended")
                      and e["ms"] <= t1 + 1), None)
        start = (ss[0]["x"], ss[0]["y"])
        dist = math.hypot(pin[0] - start[0], pin[1] - start[1])
        res = {"pin": pin, "reach": reach, "t0": t0, "t1": t1, "start": start,
               "distance_in": dist, "findings": []}
        if dist < 1e-6:
            continue
        ux, uy = (pin[0] - start[0]) / dist, (pin[1] - start[1]) / dist
        nx, ny = -uy, ux                                   # left of the direction of travel

        def lat(s):
            return (s["x"] - start[0]) * nx + (s["y"] - start[1]) * ny

        def along(s):
            return (s["x"] - start[0]) * ux + (s["y"] - start[1]) * uy

        moving = [s for s in ss if along(s) > 0.5 or s["flags"] & FLAG_MOVING]
        worst = max(ss, key=lambda s: abs(lat(s)))
        res["max_lateral_in"] = abs(lat(worst))
        res["max_lateral_at_along_in"] = along(worst)
        res["final_lateral_in"] = lat(ss[-1])

        # heading against the bearing to the pin, as the robot closes in
        def head_err(s):
            return _wrap(s["th"] - _bearing(s["x"], s["y"], pin[0], pin[1]))

        res["heading_error_start_deg"] = head_err(ss[0])
        marks = {}
        for label, d in (("20", reach + 20), ("10", reach + 10), ("4", reach + 4)):
            hit = next((s for s in ss if math.hypot(pin[0] - s["x"], pin[1] - s["y"]) <= d), None)
            if hit:
                marks[label] = head_err(hit)
        res["heading_error_deg"] = marks
        signs = [1 if head_err(s) > 1.0 else -1 if head_err(s) < -1.0 else 0 for s in ss]
        signs = [v for v in signs if v]
        res["steer_reversals"] = sum(1 for a, b in zip(signs, signs[1:]) if a != b)

        # the claw tip against the pin at the moments that matter
        def tip_vs_pin(s):
            tx, ty = _tip(s, reach)
            ahead, right = _to_robot_frame(s, pin[0], pin[1], origin=(tx, ty))
            return {"ahead_in": ahead, "right_in": right,
                    "dist_in": math.hypot(pin[0] - tx, pin[1] - ty)}

        flagged = [s for s in ss if s["flags"] & FLAG_PIN]
        edges = sum(1 for a, b in zip(ss, ss[1:])
                    if not a["flags"] & FLAG_PIN and b["flags"] & FLAG_PIN)
        res["sensor_edges"] = edges
        res["sensor_ever_saw"] = bool(flagged)
        if flagged:
            res["tip_at_first_sensor"] = tip_vs_pin(flagged[0])
            res["first_sensor_ms"] = flagged[0]["ms"] - t0
        if seen_ev:
            s_seen = _at(ss, seen_ev["ms"])
            res["seen_ms"] = seen_ev["ms"] - t0
            res["tip_at_seen"] = tip_vs_pin(s_seen)
            res["speed_at_seen_in_s"] = speeds[samples.index(s_seen)] if s_seen in samples else 0.0
        res["claws_closed_by_watch"] = seen_ev is not None
        res["drive_ended_ms"] = (ended["ms"] - t0) if ended else None
        s_end = _at(ss, ended["ms"]) if ended else ss[-1]
        res["tip_at_stop"] = tip_vs_pin(s_end)
        res["duration_ms"] = t1 - t0
        if seen_ev and ended:
            seg = [s for s in ss if seen_ev["ms"] <= s["ms"] <= ended["ms"]]
            res["drive_after_seen_in"] = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                                             for a, b in zip(seg, seg[1:]))
            res["time_after_seen_ms"] = ended["ms"] - seen_ev["ms"]
        res["trace"] = [(s["ms"] - t0, round(s["x"], 2), round(s["y"], 2), round(s["th"], 1),
                         s["flags"]) for s in ss[::5]]
        res["findings"] = _pin_findings(res)
        out.append(res)
    return out


def _side(v, pos, neg):
    return pos if v > 0 else neg


def _pin_findings(r):
    f = []
    if r["max_lateral_in"] >= 1.5:
        f.append(f"Not straight: it got {r['max_lateral_in']:.1f} in off the straight line to the "
                 f"pin, {r['max_lateral_at_along_in']:.0f} in into a {r['distance_in']:.0f} in "
                 "drive.")
    h10 = r["heading_error_deg"].get("10")
    if h10 is not None and abs(h10) >= 3.0:
        f.append(f"With 10 in to go the robot was pointing {abs(h10):.1f} deg "
                 f"{_side(h10, 'right', 'left')} of the pin.")
    if abs(r["heading_error_start_deg"]) >= 3.0:
        f.append(f"It started the drive {abs(r['heading_error_start_deg']):.1f} deg off the pin "
                 "(the turn before it did not finish square).")
    if r["steer_reversals"] >= 3:
        f.append(f"The heading error changed sign {r['steer_reversals']} times: it is "
                 "steering back and forth.")
    if not r["sensor_ever_saw"]:
        f.append("The IR sensor never saw a pin during the approach; the claws closed on "
                 "arrival, wherever the robot stopped.")
    if r["sensor_edges"] >= 3:
        f.append(f"The IR sensor flickered ({r['sensor_edges']} times on/off): check its "
                 "mounting and ambient light.")
    spd = r.get("speed_at_seen_in_s")
    if spd is not None and spd > 20:
        f.append(f"The claws started closing while it was still moving at {spd:.0f} in/s.")
    if r.get("drive_after_seen_in") is not None and r["drive_after_seen_in"] > 3.0:
        f.append(f"It kept driving {r['drive_after_seen_in']:.1f} in after the sensor saw the pin.")
    stop = r["tip_at_stop"]
    if abs(stop["right_in"]) >= 1.5:
        f.append(f"At the stop the claw tip was {abs(stop['right_in']):.1f} in "
                 f"{_side(stop['right_in'], 'left', 'right')} of the pin.")
    if abs(stop["ahead_in"]) >= 1.5:
        f.append(f"At the stop the claw tip was {abs(stop['ahead_in']):.1f} in "
                 f"{'short of' if stop['ahead_in'] > 0 else 'past'} the pin.")
    return f


# ── the sensors: the IMU and the two rotation sensors, checked against the drive motors ────────
def analyze_sensors(run, flat=None):
    """What the robot's own cross-checks say about its sensors during the run (src/run_log.cpp):
    totals, one row per plan step, and what looks wrong -- each finding tagged with the part it
    points at ("imu", "forward", "sideways", "drive"). None when the recording has no sensor data
    (older firmware, or those lines never arrived)."""
    h = run.get("health") or {}
    segs = run.get("segments") or []
    if not h and not segs:
        return None
    last_ms = run["samples"][-1]["ms"] if run.get("samples") else 0
    total_ms = h.get("ms") or last_ms
    hoff = h.get("hoff", 0.0)
    # In a turn on the spot the drive wheels scrub, so they over-read by a steady share that
    # depends on the robot (this one: about a quarter). They never UNDER-read: a turn that is
    # blocked, or wheels that spin, only make the share worse. So the cleanest turns of the run
    # (the upper quartile of IMU / wheels) are this robot's normal, and each step is judged
    # against that.
    spins = [g for g in segs if abs(g["wheel_deg"]) >= 20 and abs(g["motor_in"]) < 4]
    shares = sorted(abs(g["imu_deg"]) / abs(g["wheel_deg"]) for g in spins)
    wsum = sum(abs(g["wheel_deg"]) for g in spins)
    ratio = shares[int(0.75 * (len(shares) - 1) + 0.5)] if len(shares) >= 2 and wsum >= 90 \
        else None
    k = min(1.1, max(0.5, ratio)) if ratio else 1.0
    # The drive motors are the yardstick, so check the yardstick first. Two sides of a tank drive
    # roll about as far as each other over a run; one side reading next to nothing, or an IMU /
    # wheels ratio no real robot gives, means the motor readings are wrong, not everything else.
    ref_problem = None
    lin, rin = h.get("lin"), h.get("rin")
    if lin is not None and rin is not None:
        if max(lin, rin) >= 10 and min(lin, rin) < 0.3 * max(lin, rin):
            ref_problem = (f"the left drive motors read {lin:.0f} in over the run and the right "
                           f"{rin:.0f} in")
    elif any(abs(g["motor_in"]) >= 8 and abs(g["wheel_deg"]) > 8 * abs(g["motor_in"])
             and abs(g["imu_deg"]) < 0.3 * abs(g["wheel_deg"]) for g in segs):
        ref_problem = ("on straight drives the motors say the robot spun round while the IMU "
                       "says it went straight: one side of the drive is not being read")
    rows, findings = [], []
    motion_n = 0
    for n, g in enumerate(segs):
        end = segs[n + 1]["ms"] if n + 1 < len(segs) else max(total_ms, g["ms"])
        if g["step"] >= 0:
            kind = flat[g["step"]]["kind"] if flat and g["step"] < len(flat) else ""
            label = f"step {g['step']} {kind}".strip()
        else:
            motion_n += 1
            label = f"motion {motion_n}"
        diff = g["imu_deg"] - k * g["wheel_deg"]
        slide = g["hor_in"] + hoff * math.radians(g["imu_deg"])   # what odometry takes as sideways
        mot, vert = g["motor_in"], g["vert_in"]
        flags = []
        expect = k * g["wheel_deg"]
        on_spot = abs(g["motor_in"]) < 4 and abs(g["wheel_deg"]) >= 20
        # how far short of the wheels the robot's turn fell (+ = the wheels turned more)
        short = abs(expect) - abs(g["imu_deg"]) if expect * g["imu_deg"] >= 0 else abs(diff)
        if ref_problem:
            mot = 0.0                   # (no yardstick: skip the comparisons that need it)
        elif on_spot and ratio and short > max(10.0, 0.25 * abs(expect)):
            # the wheels worked for a bigger turn than the robot made: that is the robot, not the IMU
            flags.append("blocked")
            findings.append(("blocked", f"{label}: the drive wheels turned enough for "
                             f"{expect:+.0f} deg but the robot only turned {g['imu_deg']:+.0f} deg"
                             + (f", and it slid {abs(slide):.1f} in sideways" if abs(slide) > 2
                                else "")
                             + ". It was turning against something (a goal, a pin, the wall) or "
                             "its wheels were spinning: get clear before this turn."))
        elif on_spot and ratio and -short > max(10.0, 0.2 * abs(expect)):
            flags.append("turn")
            findings.append(("imu", f"{label}: the IMU turned {g['imu_deg']:+.1f} deg, more than "
                             f"the {expect:+.1f} deg the drive wheels can account for. Wheels "
                             "cannot under-read a turn, so the IMU over-read it (or the robot "
                             "was knocked round)."))
        elif not on_spot and abs(diff) > 8.0 + 0.2 * abs(expect) + 0.8 * abs(g["motor_in"]):
            # while driving, a little wheel slip is a lot of "turn" (0.2 in = 1 deg): only a gross
            # difference means anything
            flags.append("turn")
            findings.append(("imu", f"{label}: the IMU turned {g['imu_deg']:+.1f} deg but the "
                             f"drive wheels turned the robot about {expect:+.1f} deg "
                             f"({diff:+.1f} deg apart). Either the IMU mis-read this or a wheel "
                             "spun / the robot was knocked round."))
        if abs(mot) >= 6 and (abs(vert) < 0.6 * abs(mot) or vert * mot < 0):
            flags.append("forward")
            findings.append(("forward", f"{label}: the drive motors rolled {mot:+.1f} in but the "
                             f"forward tracking wheel only read {vert:+.1f} in -- the wheel is "
                             "lifting / slipping, or the drive wheels were spinning."))
        elif abs(mot) >= 6 and abs(vert) > 1.25 * abs(mot) + 1:
            flags.append("forward")
            findings.append(("forward", f"{label}: the forward tracking wheel read {vert:+.1f} in "
                             f"but the drive motors only rolled {mot:+.1f} in (the robot was "
                             "pushed / coasted, or the rotation sensor mis-read)."))
        if abs(slide) > 3.0 and "blocked" not in flags:
            flags.append("sideways")
            findings.append(("sideways", f"{label}: odometry took {abs(slide):.1f} in of sideways "
                             "travel from the sideways tracking wheel. A tank drive cannot do "
                             "that on its own: the robot slid, or the wheel bounced."))
        if g["tilt_deg"] >= 8:
            flags.append("tilt")
            findings.append(("imu", f"{label}: the robot tipped {g['tilt_deg']:.0f} deg. The "
                             "heading is less trustworthy while it is tipped, and the tracking "
                             "wheels lift."))
        rows.append({"label": label, "ms": g["ms"], "duration_ms": end - g["ms"],
                     "imu_deg": g["imu_deg"], "wheel_deg": g["wheel_deg"], "diff_deg": diff,
                     "vert_in": vert, "motor_in": g["motor_in"], "slide_in": slide,
                     "tilt_deg": g["tilt_deg"], "still_ms": g["still_ms"],
                     "still_deg": g["still_deg"], "flags": flags})

    res = {"rows": rows, "ratio": ratio, "total_s": total_ms / 1000.0,
           "imu_deg": sum(g["imu_deg"] for g in segs),
           "wheel_deg": sum(g["wheel_deg"] for g in segs),
           "vert_in": sum(abs(g["vert_in"]) for g in segs),
           "motor_in": sum(abs(g["motor_in"]) for g in segs),
           "still_ms": sum(g["still_ms"] for g in segs),
           "still_deg": sum(g["still_deg"] for g in segs),
           "tilt_deg": max([g["tilt_deg"] for g in segs] or [0.0]), "health": h}
    head = []
    if h:
        for key, part, name in (("imu", "imu", "IMU"),
                                ("vert", "forward", "forward (vertical) rotation sensor"),
                                ("hor", "sideways", "sideways (horizontal) rotation sensor")):
            if h.get(key, 1) < 1:
                head.append((part, f"The {name} was not connected when the run ended: check "
                             "its cable and port."))
        for key, part, name in (("imubad", "imu", "IMU"),
                                ("vbad", "forward", "forward rotation sensor"),
                                ("hbad", "sideways", "sideways rotation sensor"),
                                ("mbad", "drive", "drive motors (at least one of them)")):
            if h.get(key, 0) > 0:
                head.append((part, f"The {name} gave no reading on {h[key]:.0f} of the 20 ms "
                             "checks: a loose cable or port."))
        for key, part, name, unit in (("imujump", "imu", "IMU heading", "30 deg"),
                                      ("vjump", "forward", "forward rotation sensor", "5 in"),
                                      ("hjump", "sideways", "sideways rotation sensor", "5 in")):
            if h.get(key, 0) > 0:
                head.append((part, f"The {name} jumped more than {unit} in one 20 ms step, "
                             f"{h[key]:.0f} time(s): no robot moves that fast, the reading "
                             "glitched."))
        rate = h.get("rate0", 0.0)
        res["drift_run_deg"] = h.get("drift1", 0.0) - h.get("drift0", 0.0)
        if abs(rate) >= 0.05:
            head.append(("imu", f"The IMU drift correction was taking {rate:+.3f} deg/s off the "
                         f"heading when the run began ({rate * 15:+.1f} deg over a 15 s run). A "
                         "VEX IMU normally creeps well under 0.05 deg/s. It learns this while the robot "
                         "stands still, so carrying or turning the robot by hand (wheels not "
                         "rolling) just before the run teaches it a drift that is not there: "
                         "put the robot down and leave it still for 10 s before starting."))
    if res["still_ms"] >= 1000 and (abs(res["still_deg"]) >= 0.4 or
                                    abs(res["still_deg"]) / (res["still_ms"] / 1000.0) >= 0.1):
        head.append(("imu", f"While the robot stood still ({res['still_ms'] / 1000:.1f} s in "
                     f"total) its heading moved {res['still_deg']:+.2f} deg. Standing still it "
                     "should not move at all: that is IMU drift, or the drift correction "
                     "over-correcting."))
    res["reference_problem"] = ref_problem
    if ref_problem:
        head.append(("drive", f"The drive motor readings cannot be used as the yardstick in this "
                     f"run: {ref_problem}. That is the motors' position reading (encoder units / "
                     "a motor not reporting), not the IMU or the tracking wheels, so the IMU-"
                     "against-wheels and tracking-wheel-against-motors comparisons were skipped."))
    elif ratio is not None and not 0.55 <= ratio <= 1.05:
        head.append(("imu", f"On its cleanest turns the IMU turned {ratio * 100:.0f}% of what the "
                     "drive wheels say (60-100% is normal, wheels scrub in a turn): check the "
                     "IMU scale (INERTIAL_SCALE in main.cpp, ODOM SETUP > spin calibration) and "
                     "that the IMU is mounted flat and tight."))
    res["findings"] = head + findings
    res["suspects"] = sorted({part for part, _ in res["findings"]})
    return res


def _sensor_lines(run, flat=None):
    r = analyze_sensors(run, flat)
    if r is None:
        return ["", "SENSOR CHECK: this recording has no sensor data (the robot's firmware is "
                "older than this report, or those lines never arrived)."]
    h = r["health"]
    L = ["", "SENSOR CHECK (the IMU and both rotation sensors against the drive motors, "
         "every 20 ms of the run)"]
    names = (("imu", "IMU"), ("forward", "forward wheel (vertical rotation sensor)"),
             ("sideways", "sideways wheel (horizontal rotation sensor)"), ("drive", "drive motors"),
             ("blocked", "robot turning against something"))
    # The verdict is only as good as what arrived: the totals line (connected, glitches, drift) and
    # the per-step comparison are separate lines, and a bad link can deliver one without the other.
    whole = run.get("complete", True)
    if r["reference_problem"]:
        scope = "NOT CHECKED against the drive motors (their readings are unusable, see below)"
    elif not r["rows"]:
        scope = "NOT CHECKED against the wheels (those lines did not arrive)"
    elif not whole:
        scope = f"only {len(r['rows'])} step(s) checked (the rest did not arrive)"
    else:
        scope = ""
    okay = ("connected, no glitches" if h else "no totals arrived") + ("; " + scope if scope else "")
    for part, name in names:
        if part in ("drive", "blocked") and part not in r["suspects"]:
            continue
        L.append(f"  {name:<44} " + ("LOOKS WRONG, see below" if part in r["suspects"]
                                      else ("ok" if not scope and h else okay)))
    if h.get("lin") is not None:
        L.append(f"  drive motors over the run: left {h['lin']:.0f} in, right {h.get('rin', 0):.0f} "
                 f"in (get_position: left {h.get('lpos', 0):.1f}, right {h.get('rpos', 0):.1f}; "
                 f"encoder units {h.get('lunit', -1):.0f} / {h.get('runit', -1):.0f}; "
                 f"gearing {h.get('lgear', -1):.0f} / {h.get('rgear', -1):.0f})")
    if h:
        L.append(f"  IMU drift correction: {h.get('rate0', 0):+.3f} deg/s at the start, "
                 f"{h.get('rate1', 0):+.3f} at the end; it took {r.get('drift_run_deg', 0):+.2f} "
                 "deg off the heading during this run")
    if r["still_ms"] > 0:
        L.append(f"  standing still: {r['still_ms'] / 1000:.1f} s in total, the heading moved "
                 f"{r['still_deg']:+.2f} deg (should be 0)")
    if r["rows"]:
        L.append(f"  turning: IMU {r['imu_deg']:+.1f} deg in total, drive wheels "
                 f"{r['wheel_deg']:+.1f}"
                 + (f" (on clean turns on the spot the IMU = {r['ratio'] * 100:.0f}% of the "
                    "wheels: this robot's wheel scrub)" if r["ratio"] else "")
                 + ("" if whole else "  [only the steps that arrived]"))
        L.append(f"  driving: forward tracking wheel {r['vert_in']:.1f} in, drive motors "
                 f"{r['motor_in']:.1f} in; most the robot tipped {r['tilt_deg']:.1f} deg")
    shown = [w for w in r["rows"] if w["flags"] or abs(w["wheel_deg"]) >= 3 or
             abs(w["imu_deg"]) >= 3 or abs(w["motor_in"]) >= 1 or abs(w["vert_in"]) >= 1]
    if shown:
        L.append("  per step:   turn IMU / wheels (deg)   forward wheel / motors (in)   "
                 "slid sideways (in)   tilt (deg)")
        for w in shown[:70]:
            L.append(f"   {w['label']:<24} {w['imu_deg']:+7.1f} /{w['wheel_deg']:+7.1f}     "
                     f"{w['vert_in']:+7.1f} /{w['motor_in']:+7.1f}     {w['slide_in']:+5.1f}     "
                     f"{w['tilt_deg']:4.1f}"
                     + ("   <-- " + ", ".join(w["flags"]) if w["flags"] else ""))
    if r["findings"]:
        L.append("  LOOKS WRONG:")
        L += [f"   - {text}" for _, text in r["findings"]]
    elif scope:
        L.append("  NO VERDICT: nothing wrong in what arrived, but "
                 + ("the IMU-against-wheels comparison never arrived." if not r["rows"]
                    else "part of the IMU-against-wheels comparison is missing."))
        L.append("  This run cannot say whether the IMU is at fault. Run again over the USB cable "
                 "for a whole recording.")
    else:
        L.append("  nothing looks wrong: the IMU, the tracking wheels and the drive motors agree "
                 "with each other.")
        L.append("  So where a step below ends with a heading error, the robot KNEW it was off: "
                 "that turn did not finish")
        L.append("  (its timeout, early exit or the turn PID), it is not the IMU mis-reading.")
    return L


# ── the plan's steps ────────────────────────────────────────────────────────────────────────────
def analyze_steps(run, flat, start):
    """Per plan step: when it began, how long it took, and for moves how far from the simulator's
    end point (and heading) the robot was when the step after it began. Needs the plan the run
    used (`flat`: the uploaded blocks)."""
    begins = {}
    for e in run["events"]:
        m = re.match(r"step (\d+) begin", e["what"])
        if m:
            begins[int(m.group(1))] = e
    if not begins:
        return []
    sim = pc.simulate(flat, tuple(start))
    last_ms = run["samples"][-1]["ms"] if run["samples"] else 0
    order = sorted(begins)
    rows = []
    for i in order:
        if i >= len(flat):
            continue
        nxt = [j for j in order if j > i]
        ev = begins[i]
        end_ms = begins[nxt[0]]["ms"] if nxt else last_ms
        row = {"step": i, "kind": flat[i]["kind"], "begin_ms": ev["ms"],
               "duration_ms": end_ms - ev["ms"]}
        if flat[i]["kind"] in pc.MOTIONS or flat[i]["kind"] == "goToPin":
            j = next((k for k in nxt if flat[k]["kind"] != "waitUntilDone"), None)
            arrive = begins[j] if j is not None else None
            at = arrive if arrive else (run["samples"][-1] if run["samples"] else None)
            if at is not None and i < len(sim):
                px, py, pth = sim[i][0]
                ax, ay, ath = (at["x"], at["y"], at["th"])
                row["end_error_in"] = math.hypot(ax - px, ay - py)
                row["heading_error_deg"] = _wrap(ath - pth)
                row["planned_end"] = (round(px, 1), round(py, 1), round(pth, 1))
                row["actual_end"] = (round(ax, 1), round(ay, 1), round(ath, 1))
        rows.append(row)
    return rows


# ── the report ──────────────────────────────────────────────────────────────────────────────────
def format_report(run, flat=None, start=None):
    L = []
    samples = run["samples"]
    dur = samples[-1]["ms"] / 1000.0 if samples else 0.0
    L.append(f"RUN REPORT  #{run['id']}  {run['name']}  {run['status']}  {dur:.2f} s  "
             f"({len(samples)} samples every {run['period_ms']} ms"
             + (", TRUNCATED" if run["truncated"] else "") + ")")
    got_s, got_e = len(samples), len(run["events"])
    exp_s, exp_e = run["expected"]
    if got_s < exp_s or got_e < exp_e:
        L.append(f"WARNING: the link dropped data: {got_s}/{exp_s} samples, "
                 f"{got_e}/{exp_e} events arrived.")
    if samples:
        a, b = samples[0], samples[-1]
        L.append(f"start pose ({a['x']:.1f}, {a['y']:.1f}) {a['th']:.1f} deg   "
                 f"end pose ({b['x']:.1f}, {b['y']:.1f}) {b['th']:.1f} deg")
    if not run.get("complete", True):
        got, total = run.get("arrived", (0, None))
        asks = run.get("asks")
        L.append(f"WARNING: only {got} of {total if total else '?'} lines of this recording "
                 "arrived, even after asking the robot again"
                 + (f" ({asks[0]} requests sent, the robot acknowledged {asks[1]})" if asks else "")
                 + ". What follows is what did arrive.")
    try:
        L += _sensor_lines(run, flat if run["name"] in ("plan", "step") else None)
    except Exception as err:            # one section failing must not cost the whole report
        L += ["", f"SENSOR CHECK failed: {err}"]
    rows = []
    if flat is not None and start is not None and run["name"] in ("plan", "step"):
        try:
            rows = analyze_steps(run, flat, start)
        except Exception as err:
            L += ["", f"STEPS failed: {err} (is this the plan the robot ran?)"]
        if rows:
            L.append("")
            L.append("STEPS (end error = robot vs the simulator when the next step began)")
            for r in rows:
                line = f"  {r['step']:3d} {r['kind']:<15} {r['begin_ms'] / 1000:6.2f} s " \
                       f"+{r['duration_ms'] / 1000:5.2f} s"
                if "end_error_in" in r:
                    line += f"   end error {r['end_error_in']:4.1f} in  {r['heading_error_deg']:+5.1f} deg"
                L.append(line)
    try:
        pins = analyze_pins(run)
    except Exception as err:
        pins = []
        L += ["", f"GO TO PIN analysis failed: {err}"]
    for n, r in enumerate(pins, 1):
        L.append("")
        L.append(f"GO TO PIN {n}: pin ({r['pin'][0]:g}, {r['pin'][1]:g}), claw reach "
                 f"{r['reach']:g} in, drove {r['distance_in']:.0f} in from "
                 f"({r['start'][0]:.1f}, {r['start'][1]:.1f}) in {r['duration_ms'] / 1000:.2f} s")
        L.append(f"  straightness: max {r['max_lateral_in']:.1f} in off the line "
                 f"(at {r['max_lateral_at_along_in']:.0f} in along), "
                 f"final {r['final_lateral_in']:+.1f} in (+ = left)")
        hm = r["heading_error_deg"]
        L.append(f"  heading vs the pin (deg, + = pointing right of it): start "
                 f"{r['heading_error_start_deg']:+.1f}"
                 + "".join(f", {k} in out {v:+.1f}" for k, v in hm.items())
                 + f"; steering reversals {r['steer_reversals']}")
        L.append(f"  IR sensor: " + (f"first saw the pin at {r['first_sensor_ms'] / 1000:.2f} s, "
                                     if r.get("first_sensor_ms") is not None else "never saw it, ")
                 + f"{r['sensor_edges']} on/off edges; claw watch "
                 + ("closed the claws" if r["claws_closed_by_watch"] else "did NOT fire"))
        for label, key in (("first sensor", "tip_at_first_sensor"), ("claws closing", "tip_at_seen"),
                           ("stop", "tip_at_stop")):
            t = r.get(key)
            if t:
                L.append(f"  claw tip at {label:<13}: pin is {t['ahead_in']:+.1f} in ahead, "
                         f"{t['right_in']:+.1f} in right of the tip ({t['dist_in']:.1f} in away)")
        if r.get("speed_at_seen_in_s") is not None:
            extra = ""
            if r.get("drive_after_seen_in") is not None:
                extra = (f"; then {r['drive_after_seen_in']:.1f} in more driving over "
                         f"{r['time_after_seen_ms']} ms")
            L.append(f"  speed when the sensor fired: {r['speed_at_seen_in_s']:.0f} in/s{extra}")
        if r["findings"]:
            L.append("  LOOKS WRONG:")
            L += [f"   - {t}" for t in r["findings"]]
        else:
            L.append("  nothing looks wrong")
        L.append("  trace (ms, x, y, heading, flags 1=IR 8=moving):")
        L.append("   " + " ".join(f"({t[0]},{t[1]},{t[2]},{t[3]},{t[4]})" for t in r["trace"][:40]))
    if not pins and not any(e["what"].startswith("step") for e in run["events"]):
        L.append("(no steps or pin grabs were logged in this run)")
    if run["events"]:
        L.append("")
        L.append("EVENTS")
        for e in run["events"][:80]:
            L.append(f"  {e['ms'] / 1000:6.2f} s  step {e['step']:3d}  {e['what']:<44} "
                     f"({e['x']:.1f}, {e['y']:.1f}) {e['th']:.1f} deg")
    return "\n".join(L)


def save_report(run, text):
    """Writes the report (and the raw recording) to tools/run_reports/; returns the .txt path."""
    os.makedirs(REPORTS_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(REPORTS_DIR, f"{stamp}-{re.sub(r'[^A-Za-z0-9]+', '', run['name'])}")
    with open(base + ".txt", "w", encoding="utf-8") as f:
        f.write(text + "\n")
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump(run, f)
    old = sorted(p for p in os.listdir(REPORTS_DIR) if p.endswith((".txt", ".json")))
    for p in old[:-KEEP_REPORTS * 2]:
        try:
            os.remove(os.path.join(REPORTS_DIR, p))
        except OSError:
            pass
    return base + ".txt"
