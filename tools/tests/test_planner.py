"""Tests for the planner's logic (planner_core.py) -- no window, no robot.

Run from the repository root:   python -m unittest discover tools/tests
"""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import planner_core as pc  # noqa: E402

B = pc.new_block
START = (-48.0, -60.0, 0.0)


def route():
    return [B("moveToPoint", x=-48, y=-40), B("waitUntilDone"),
            B("turnToHeading", theta=90, direction="CW"), B("waitUntilDone"),
            B("swingToHeading", theta=-45, side="LEFT"), B("waitUntilDone"),
            B("moveToPose", x=-20, y=0, theta=30), B("waitUntilDone"),
            B("goToPin", x=-24, y=-24), B("waitUntilDone")]


class Simulation(unittest.TestCase):
    def test_drives_to_its_target(self):
        end = pc.simulate([B("moveToPoint", x=-48, y=-30)], START)[0][0]
        self.assertAlmostEqual(end[0], -48, delta=1.0)
        self.assertAlmostEqual(end[1], -30, delta=1.0)

    def test_every_block_gets_a_result(self):
        self.assertEqual(len(pc.simulate(route(), START)), len(route()))

    def test_estimate_is_positive_and_splits(self):
        total, driving, other = pc.estimate_seconds(route(), START)
        self.assertGreater(total, 0)
        self.assertAlmostEqual(total, driving + other)


class Flip(unittest.TestCase):
    def ends(self, blocks, start):
        return [p for p, _shape in pc.simulate(blocks, tuple(start))]

    def test_rotate_matches_the_transformed_original(self):
        blocks = route() + [B("shootAtGoal", x=-24, y=-48, goalSide="RIGHT")]
        fb, fs = pc.flip_plan(blocks, START, "rotate")
        for a, b in zip(self.ends(blocks, START), self.ends(fb, fs)):
            self.assertAlmostEqual(b[0], -a[0], delta=0.05)
            self.assertAlmostEqual(b[1], -a[1], delta=0.05)

    def test_mirror_matches_the_transformed_original(self):
        blocks = route()
        fb, fs = pc.flip_plan(blocks, START, "mirror")
        for a, b in zip(self.ends(blocks, START), self.ends(fb, fs)):
            self.assertAlmostEqual(b[0], -a[0], delta=0.05)
            self.assertAlmostEqual(b[1], a[1], delta=0.05)

    def test_flipping_twice_gives_the_original(self):
        for mode in ("rotate", "mirror"):
            fb, fs = pc.flip_plan(route(), START, mode)
            bb, _bs = pc.flip_plan(fb, fs, mode)
            for a, b in zip(route(), bb):
                for key in ("x", "y", "theta", "direction", "side"):
                    if key in a:
                        if isinstance(a[key], float):
                            self.assertAlmostEqual(a[key], b[key], places=1, msg=f"{mode} {key}")
                        else:
                            self.assertEqual(a[key], b[key], f"{mode} {key}")

    def test_mirror_swaps_turn_direction_and_swing_side(self):
        fb, _ = pc.flip_plan([B("turnToHeading", theta=90, direction="CW"),
                              B("swingToHeading", theta=10, side="LEFT")], START, "mirror")
        self.assertEqual((fb[0]["direction"], fb[1]["side"]), ("CCW", "RIGHT"))

    def test_rotate_keeps_direction_but_flips_goal_side(self):
        fb, _ = pc.flip_plan([B("turnToHeading", theta=90, direction="CW"),
                              B("shootAtGoal", x=-24, y=-48, goalSide="TOP")], START, "rotate")
        self.assertEqual(fb[0]["direction"], "CW")
        self.assertEqual(fb[1]["goalSide"], "BOTTOM")

    def test_rotate_lands_goals_and_pins_on_goals_and_pins(self):
        blocks = [B("goToPin", x=p[0], y=p[1]) for p in pc.PINS] + \
                 [B("shootAtGoal", x=g[0], y=g[1]) for g in pc.GOALS]
        fb, _ = pc.flip_plan(blocks, START, "rotate")
        self.assertEqual(pc.flip_report(fb), [])


class Warnings(unittest.TestCase):
    def test_clean_plan(self):
        self.assertEqual(pc.plan_warnings([B("moveToPoint", x=-48, y=-40)], START), [])

    def test_leaving_the_field(self):
        w = pc.plan_warnings([B("moveToPoint", x=90, y=-40)], START)
        self.assertTrue(any("outside the field" in t for t in w))

    def test_driving_through_a_goal(self):
        w = pc.plan_warnings([B("moveToPoint", x=0, y=-20), B("moveToPoint", x=0, y=20)], START)
        self.assertTrue(any("goal" in t for t in w))

    def test_too_long_for_autonomous(self):
        w = pc.plan_warnings([B("wait", ms=16000), B("wait", ms=16000)], START)
        self.assertTrue(any("autonomous" in t for t in w))

    def test_empty_plan(self):
        self.assertEqual(pc.plan_warnings([], START), [])


class Avoiding(unittest.TestCase):
    def test_goes_round_the_goal_in_the_way(self):
        way = pc.avoid_route((0, -30), (0, 30), (15, 15))
        self.assertTrue(way)
        pts = [(0, -30)] + way + [(0, 30)]
        for p, q in zip(pts, pts[1:]):
            self.assertGreater(pc._swept_gap(p, q, (0, 0), 15, 15), pc.GOAL_R)

    def test_clear_way_needs_no_detour(self):
        self.assertEqual(pc.avoid_route((-60, -60), (-60, -30), (15, 15)), [])


class Shooting(unittest.TestCase):
    def test_shoot_expands_to_blocks_ending_in_a_shot(self):
        flat, spans = pc.expand_plan([B("shootAtGoal", x=-24, y=-48, goalSide="RIGHT")], START)
        self.assertEqual(flat[-1]["kind"], "shootToGoal")
        self.assertEqual(spans, [(0, len(flat))])
        self.assertNotIn("shootAtGoal", [b["kind"] for b in flat])

    def test_a_pin_shot_lifts_and_opens_the_claws(self):
        flat, _ = pc.expand_plan([B("shootAtGoal", x=-24, y=-48, goalSide="RIGHT",
                                    isPin="YES")], START)
        kinds = {b["kind"] for b in flat if b["kind"] not in pc.MOTIONS}
        self.assertTrue({"liftToLevel", "claw"} <= kinds)
        self.assertNotIn("shootToGoal", kinds)


class Blocks(unittest.TestCase):
    def test_relocalize_is_gone(self):
        self.assertNotIn("relocalize", pc.KINDS)

    def test_every_block_can_be_sent_and_exported(self):
        for b in route():
            self.assertTrue(pc.block_to_plan_add(b).startswith("PLAN ADD "))
            self.assertTrue(pc.block_code(b))
        self.assertIn("chassis.moveToPoint", pc.export_code(route(), START))

    def test_export_has_no_nan(self):
        self.assertNotIn("nan", pc.export_code(route(), START).lower())


class Calibration(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.old = (pc.CALIBRATION_FILE, pc.CAL["runs"][:], dict(pc.CAL["extra_samples"]))
        pc.CALIBRATION_FILE = os.path.join(self.tmp.name, "cal.json")   # never the real file
        pc.CAL["runs"], pc.CAL["extra_samples"] = [], {}

    def tearDown(self):
        pc.CALIBRATION_FILE = self.old[0]
        pc.CAL["runs"], pc.CAL["extra_samples"] = self.old[1], self.old[2]
        self.tmp.cleanup()

    def flat(self):
        return [B("moveToPoint", x=-48, y=0), B("waitUntilDone"),
                B("turnToHeading", theta=90), B("waitUntilDone"), B("claw")]

    def test_learns_that_the_robot_is_slower(self):
        flat, times = self.flat(), []
        pc.simulate(flat, START, times=times)

        def real(j):
            return (times[j] / 1000 + pc.MOTION_OVERHEAD_S) * 1.25
        steps = {0: 0.01, 1: real(0), 2: 0.01, 3: real(2), 4: 0.3}
        before = pc.estimate_seconds(flat, START)[0]
        self.assertEqual(pc.update_calibration(flat, START, steps), 2)
        self.assertAlmostEqual(pc.drive_scale(), 1.25, delta=0.03)
        self.assertGreater(pc.estimate_seconds(flat, START)[0], before)
        self.assertAlmostEqual(pc.extra_seconds("claw"), 0.3)

    def test_ignores_absurd_times(self):
        self.assertEqual(pc.update_calibration(self.flat(), START,
                                               {0: 0, 1: 90.0, 2: 0, 3: 90.0}), 0)
        self.assertEqual(pc.drive_scale(), 1.0)

    def test_drift_is_how_far_from_the_simulated_end(self):
        flat = self.flat()
        sim = pc.simulate(flat, START)
        poses = {1: (sim[0][0][0] + 3, sim[0][0][1] + 4, 0), 3: (*sim[2][0][:2], 90)}
        drift = pc.step_drift(flat, START, poses)
        self.assertAlmostEqual(drift[0], 5.0, places=1)
        self.assertAlmostEqual(drift[2], 0.0, places=1)


if __name__ == "__main__":
    unittest.main()
