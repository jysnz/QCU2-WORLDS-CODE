"""Tests for the run analytics (run_analysis.py), using made-up recordings in the exact format the
robot sends (src/run_log.cpp). Run:  python -m unittest discover tools/tests"""
import math
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import planner_core as pc  # noqa: E402
import run_analysis as ra  # noqa: E402


fake_pin_run = ra.demo_lines


def read(lines):
    reader, run = ra.LogReader(), None
    for ln in lines:
        run = reader.feed(ln) or run
    return run


def resum(line):
    """The line with its checksum made right again, after a test has changed its text."""
    body = line[:line.rindex("|*")]
    return f"{body}|*{sum(body.encode('utf-8')) & 0xFF:02X}"


def wanted(cmd):
    """The line numbers a "LOG NEED <run> 3-9 12" command asks for."""
    out = []
    for tok in cmd.split()[3:]:
        a, _, b = tok.partition("-")
        out += range(int(a), int(b or a) + 1)
    return out


class Reading(unittest.TestCase):
    def test_a_recording_is_read_back_in_full(self):
        lines = fake_pin_run()
        run = read(lines)
        head = lines[0].split("|")
        self.assertEqual(run["name"], "plan")
        self.assertTrue(run["complete"])
        self.assertEqual(len(run["samples"]), int(head[6]))
        self.assertEqual(len(run["events"]), int(head[8]))
        self.assertEqual(run["events"][1]["what"].split()[0:2], ["pin", "target"])
        self.assertEqual(len(run["segments"]), 1)
        self.assertEqual(run["health"]["imu"], 1)

    def test_garbage_and_foreign_lines_are_ignored(self):
        reader = ra.LogReader()
        self.assertIsNone(reader.feed("RUI|HOME|..."))
        self.assertIsNone(reader.feed("LOG|S|not,numbers"))
        self.assertIsNone(reader.feed("LOG|E|1|2"))
        self.assertIsNone(reader.feed("LOG|#x.y|E|1|2"))

    def test_no_run_until_the_last_line(self):
        reader = ra.LogReader()
        out = [reader.feed(ln) for ln in fake_pin_run()]
        self.assertTrue(all(o is None for o in out[:-1]))
        self.assertIsNotNone(out[-1])

    def test_the_first_version_without_line_numbers_is_still_read(self):
        lines = ["LOG|BEGIN|2|plan|DONE|2|20|1|0",
                 "LOG|E|0|0|step 0 begin|0.00|0.00|0.0",
                 "LOG|S|0,0.00,0.00,0.0,0,8;20,0.5,1.0,0.0,0,8",
                 "LOG|END|2"]
        run = read(lines)
        self.assertEqual((len(run["samples"]), len(run["events"])), (2, 1))
        self.assertIn("no sensor data", ra.format_report(run))


class Resending(unittest.TestCase):
    """The link drops lines; the reader says which, and the robot sends those again."""

    def test_missing_lines_are_asked_for_by_number(self):
        lines = fake_pin_run()
        reader = ra.LogReader()
        for n, ln in enumerate(lines):
            if n not in (3, 4, 5, 9):
                self.assertIsNone(reader.feed(ln))
        self.assertEqual(reader.need_command(), "LOG NEED 1 3-5 9")
        run = None
        for n in wanted(reader.need_command()):
            run = reader.feed(lines[n]) or run
        self.assertTrue(run["complete"])
        self.assertEqual(run["samples"], read(lines)["samples"])

    def test_a_lost_first_line_is_asked_for_first(self):
        lines = fake_pin_run()
        reader = ra.LogReader()
        for ln in lines[1:6]:
            reader.feed(ln)
        self.assertEqual(reader.need_command(), "LOG NEED 1 0")

    def test_a_line_with_a_character_lost_is_rejected_and_asked_for_again(self):
        lines = fake_pin_run()
        reader = ra.LogReader()
        bad = next(n for n, ln in enumerate(lines) if "|E|" in ln)
        for n, ln in enumerate(lines):
            if n == bad:                        # one digit of the time goes missing
                head, sep, rest = ln.partition("|E|")
                ln = head + sep + rest[1:] if rest[0].isdigit() and rest[1].isdigit() \
                    else head + sep + rest.replace(".", "", 1)
            self.assertIsNone(reader.feed(ln))
        self.assertEqual(reader.need_command(), f"LOG NEED 1 {bad}")

    def test_a_line_arriving_twice_counts_once(self):
        lines = fake_pin_run()
        run = read(lines[:5] + lines[:5] + lines[5:])
        self.assertEqual(len(run["samples"]), int(lines[0].split("|")[6]))
        self.assertEqual(len(run["events"]), int(lines[0].split("|")[8]))

    def test_a_request_fits_the_robots_command_buffer(self):
        lines = fake_pin_run()
        reader = ra.LogReader()
        for n, ln in enumerate(lines):
            if n % 2 == 0:
                reader.feed(ln)
        self.assertLessEqual(len(reader.need_command()), 120)   # the brain reads 160 bytes

    def test_a_very_bad_link_still_ends_with_the_whole_recording(self):
        rng = random.Random(7)
        lines = fake_pin_run()
        reader = ra.LogReader()
        run = None
        for ln in lines:
            if rng.random() > 0.85:                 # 85% of the lines are lost...
                run = reader.feed(ln) or run
        asks = 0
        while run is None and asks < 400:
            cmd = reader.need_command()
            self.assertIsNotNone(cmd)
            asks += 1
            for n in wanted(cmd):
                if rng.random() > 0.85:             # ...and of every resend as well
                    run = reader.feed(lines[n]) or run
        self.assertIsNotNone(run)
        self.assertTrue(run["complete"])
        self.assertEqual(run["samples"], read(lines)["samples"])

    def test_giving_up_reports_what_arrived(self):
        lines = fake_pin_run()
        reader = ra.LogReader()
        for ln in lines[:3] + lines[4:12]:
            reader.feed(ln)
        run = reader.take()
        self.assertFalse(run["complete"])
        self.assertFalse(reader.active)
        text = ra.format_report(run)
        self.assertIn("WARNING: only", text)
        self.assertIn("SENSOR CHECK", text)         # the sensor lines go first: they arrived

    def test_a_new_run_replaces_an_unfinished_one(self):
        lines = fake_pin_run()
        reader = ra.LogReader()
        for ln in lines[:6]:
            reader.feed(ln)
        nxt = [resum(ln.replace("LOG|#1.", "LOG|#2.")) for ln in lines]
        run = None
        for ln in nxt:
            run = reader.feed(ln) or run
        self.assertTrue(run["complete"])
        self.assertEqual(run["id"], 2)


class Sensors(unittest.TestCase):
    def check(self, **kw):
        return ra.analyze_sensors(read(fake_pin_run(speed=12.0, **kw)))

    def test_sensors_that_agree_are_clean(self):
        r = self.check()
        self.assertEqual(r["findings"], [])
        self.assertEqual(r["suspects"], [])
        text = ra.format_report(read(fake_pin_run(speed=12.0)))
        self.assertIn("nothing looks wrong: the IMU", text)

    def test_an_imu_that_disagrees_with_the_wheels_is_flagged_in_its_step(self):
        r = self.check(imu_extra=45.0)
        self.assertIn("imu", r["suspects"])
        self.assertIn("turn", r["rows"][0]["flags"])
        self.assertTrue(any("step 0" in t and "IMU turned" in t for _, t in r["findings"]))

    def test_a_tracking_wheel_that_reads_short_is_flagged(self):
        r = self.check(vert_scale=0.4)
        self.assertEqual(r["suspects"], ["forward"])

    def test_a_large_drift_correction_is_flagged(self):
        r = self.check(drift_rate=0.12)
        self.assertTrue(any("drift correction" in t for _, t in r["findings"]))
        self.assertEqual(self.check(drift_rate=0.02)["findings"], [])

    def test_a_disconnected_sensor_and_a_glitch_are_flagged(self):
        lines = [resum(ln.replace("|hor=1|", "|hor=0|").replace("|imujump=0|", "|imujump=2|"))
                 for ln in fake_pin_run(speed=12.0)]
        r = ra.analyze_sensors(read(lines))
        self.assertEqual(r["suspects"], ["imu", "sideways"])

    def test_steady_wheel_scrub_in_turns_is_not_an_imu_fault(self):
        # three 90 deg turns where the wheels over-read by the same 12%, then one that does not fit
        def seg(ms, step, imu, whl):
            return {"ms": ms, "step": step, "imu_deg": imu, "wheel_deg": whl, "vert_in": 0.0,
                    "motor_in": 0.0, "hor_in": 0.0, "tilt_deg": 0.5, "still_ms": 0, "still_deg": 0.0}
        run = {"samples": [], "health": {"ms": 4000, "hoff": 0.0},
               "segments": [seg(0, 0, 90, 101), seg(1000, 1, -90, -101), seg(2000, 2, 90, 101)]}
        self.assertEqual(ra.analyze_sensors(run)["findings"], [])
        run["segments"].append(seg(3000, 3, 55, 101))
        r = ra.analyze_sensors(run)
        self.assertEqual([w["flags"] for w in r["rows"]], [[], [], [], ["blocked"]])
        self.assertEqual(r["suspects"], ["blocked"])        # the robot, not the IMU
        run["segments"][3] = seg(3000, 3, 130, 101)         # more turn than the wheels made
        self.assertEqual(ra.analyze_sensors(run)["suspects"], ["imu"])

    def test_turning_explains_the_sideways_wheel(self):
        # a 360 deg spin drags a sideways wheel mounted 1.5 in behind the centre by 9.4 in
        seg = {"ms": 0, "step": 0, "imu_deg": 360.0, "wheel_deg": 380.0, "vert_in": 0.0,
               "motor_in": 0.0, "hor_in": 1.5 * 2 * math.pi, "tilt_deg": 0.0, "still_ms": 0,
               "still_deg": 0.0}
        run = {"samples": [], "health": {"ms": 2000, "hoff": -1.5}, "segments": [seg]}
        self.assertEqual(ra.analyze_sensors(run)["findings"], [])
        run["segments"][0] = dict(seg, hor_in=seg["hor_in"] + 5.0)
        self.assertEqual(ra.analyze_sensors(run)["suspects"], ["sideways"])


class PinAnalysis(unittest.TestCase):
    def pin(self, **kw):
        pins = ra.analyze_pins(read(fake_pin_run(**kw)))
        self.assertEqual(len(pins), 1)
        return pins[0]

    def test_a_straight_approach_is_clean(self):
        r = self.pin(speed=12.0)
        self.assertLess(r["max_lateral_in"], 0.1)
        self.assertEqual(r["findings"], [])
        self.assertAlmostEqual(r["tip_at_stop"]["ahead_in"], 0.0, delta=0.5)

    def test_a_bulging_approach_is_flagged_with_its_size(self):
        r = self.pin(sway=3.0)
        self.assertAlmostEqual(r["max_lateral_in"], 3.0, delta=0.3)
        self.assertTrue(any("Not straight" in f for f in r["findings"]))

    def test_a_late_turn_is_flagged(self):
        r = self.pin(start_heading=6.0)
        self.assertTrue(any("off the pin" in f for f in r["findings"]))

    def test_a_sensor_that_never_fires_is_flagged(self):
        r = self.pin(sees=False)
        self.assertFalse(r["sensor_ever_saw"])
        self.assertTrue(any("never saw" in f for f in r["findings"]))

    def test_speed_when_the_sensor_fires_is_measured(self):
        r = self.pin(speed=30.0)
        self.assertAlmostEqual(r["speed_at_seen_in_s"], 30.0, delta=4.0)
        self.assertTrue(any("still moving" in f for f in r["findings"]))

    def test_a_tip_off_to_the_side_is_reported_in_the_right_direction(self):
        # the pin is 2 in to the right of where the robot is heading
        r = self.pin(pin=(2.0, 40.0))
        self.assertGreater(r["tip_at_stop"]["right_in"], 0.5)


class Report(unittest.TestCase):
    def test_the_report_names_the_problem(self):
        text = ra.format_report(read(fake_pin_run(sway=3.0)))
        self.assertIn("GO TO PIN 1", text)
        self.assertIn("LOOKS WRONG", text)
        self.assertIn("Not straight", text)

    def test_the_sensor_check_comes_before_the_steps(self):
        flat = [pc.new_block("moveToPoint", x=0, y=40)]
        text = ra.format_report(read(fake_pin_run()), flat, (0, 0, 0))
        self.assertLess(text.index("SENSOR CHECK"), text.index("STEPS"))
        self.assertIn("step 0 moveToPoint", text)

    def test_steps_compare_the_end_with_the_plan(self):
        flat = [pc.new_block("moveToPoint", x=0, y=40), pc.new_block("waitUntilDone"),
                pc.new_block("claw")]
        lines = ["LOG|BEGIN|2|plan|DONE|3|20|4|0",
                 "LOG|E|0|0|step 0 begin|0.00|0.00|0.0",
                 "LOG|E|10|1|step 1 begin|0.00|1.00|0.0",
                 "LOG|E|2000|2|step 2 begin|1.00|38.00|0.0",
                 "LOG|E|2300|2|cancelled|1.00|38.00|0.0",
                 "LOG|S|0,0.00,0.00,0.0,0,8,0,0,0;1000,0.5,20.0,0.0,1,8,0,0,0;"
                 "2000,1.00,38.00,0.0,2,0,0,0,0;2300,1.00,38.00,0.0,2,0,0,0,0",
                 "LOG|END|2"]
        run = read(lines)
        rows = ra.analyze_steps(run, flat, (0, 0, 0))
        move = rows[0]
        self.assertEqual(move["kind"], "moveToPoint")
        self.assertAlmostEqual(move["end_error_in"], math.hypot(1.0, 2.0), delta=0.2)
        self.assertIn("end error", ra.format_report(run, flat, (0, 0, 0)))

    def test_reports_are_saved_and_old_ones_pruned(self):
        old = ra.REPORTS_DIR
        with tempfile.TemporaryDirectory() as tmp:
            ra.REPORTS_DIR = tmp
            try:
                run = read(fake_pin_run())
                path = ra.save_report(run, ra.format_report(run))
                self.assertTrue(os.path.exists(path))
                self.assertTrue(os.path.exists(path[:-4] + ".json"))
            finally:
                ra.REPORTS_DIR = old


if __name__ == "__main__":
    unittest.main()
