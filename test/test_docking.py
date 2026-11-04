"""Unittests voor het docken onder de kar — zonder ROS.

Draaien:  python3 -m unittest discover -s test -p "test_docking*" -v   (vanuit navigatie_ezwheels/)
"""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from docking import (  # noqa: E402
    axis_cmd, cart_legs, compose, find_leg_candidates, fit_pose, inverse, match_cart, point_cmd,
    rotate_cmd, transform)

LEGS = cart_legs(0.29, 0.3625)
LEG_R = 0.02
CART = (6.0, 6.6, 0.3)


def scan(sensor, circles, walls=(), noise=0.0, step=math.radians(0.375), fov=math.radians(135)):
    """Gesimuleerde lidar: stralen vanaf sensor (pose) op cirkels (x, y, r) en muren (x = c)."""
    rng = random.Random(1)
    points = []
    a = -fov
    while a <= fov:
        h = sensor[2] + a
        dx, dy = math.cos(h), math.sin(h)
        best = 12.0
        for cx, cy, r in circles:
            fx, fy = sensor[0] - cx, sensor[1] - cy
            b = fx * dx + fy * dy
            disc = b * b - (fx * fx + fy * fy - r * r)
            if disc >= 0 and -b - math.sqrt(disc) > 0.12:
                best = min(best, -b - math.sqrt(disc))
        for wx in walls:
            if dx > 1e-6 and (wx - sensor[0]) / dx > 0.12:
                best = min(best, (wx - sensor[0]) / dx)
        if best < 12.0:
            d = best + rng.gauss(0, noise)
            points.append((sensor[0] + d * dx, sensor[1] + d * dy))
        a += step
    return points


def legs_world(cart=CART):
    return [(*transform(cart, leg), LEG_R) for leg in LEGS]


class TestGeometry(unittest.TestCase):

    def test_compose_inverse(self):
        p = (1.0, -2.0, 0.7)
        q = compose(p, inverse(p))
        self.assertAlmostEqual(q[0], 0)
        self.assertAlmostEqual(q[1], 0)
        self.assertAlmostEqual(q[2], 0)

    def test_fit_pose_uit_twee_en_vier_poten(self):
        for model in (LEGS, LEGS[:2], [LEGS[0], LEGS[3]]):
            pose, rms = fit_pose(model, [transform(CART, p) for p in model])
            for got, want in zip(pose, CART):
                self.assertAlmostEqual(got, want, places=9)
            self.assertLess(rms, 1e-9)


class TestDetectie(unittest.TestCase):

    def test_poten_gevonden_tot_op_millimeters(self):
        sensor = compose(CART, (-1.2, 0.05, 0.0))
        points = scan(sensor, legs_world(), walls=[20.0])
        centers = find_leg_candidates(points, sensor[:2], LEG_R)
        self.assertEqual(len(centers), 4)
        for leg in legs_world():
            self.assertLess(min(math.dist(c, leg[:2]) for c in centers), 0.002)

    def test_muur_is_geen_poot(self):
        sensor = (0.0, 0.0, 0.0)
        self.assertEqual(find_leg_candidates(scan(sensor, [], walls=[2.0]), sensor[:2], LEG_R), [])

    def test_kar_gevonden_met_scheve_verwachting_en_stoorobject(self):
        sensor = compose(CART, (-1.4, 0.1, 0.1))
        circles = legs_world() + [(*transform(CART, (-0.6, 0.0)), 0.025)]   # paaltje voor de kar
        centers = find_leg_candidates(scan(sensor, circles, noise=0.005), sensor[:2], LEG_R)
        prior = compose(CART, (0.2, -0.15, math.radians(6)))               # AMCL zit ernaast
        pose, n, rms = match_cart(centers, prior, LEGS, gate=0.4)
        self.assertEqual(n, 4)
        self.assertLess(math.dist(pose[:2], CART[:2]), 0.005)
        self.assertLess(abs(pose[2] - CART[2]), math.radians(0.5))

    def test_onder_de_kar_twee_poten_genoeg(self):
        sensor = compose(CART, (0.2, 0.0, 0.0))   # lidar van een gedockte robot
        centers = find_leg_candidates(scan(sensor, legs_world()), sensor[:2], LEG_R)
        pose, n, _ = match_cart(centers, CART, LEGS, gate=0.15)
        self.assertEqual(n, 2)
        self.assertLess(math.dist(pose[:2], CART[:2]), 0.003)

    def test_geen_kar_geen_match(self):
        self.assertIsNone(match_cart([(0.0, 0.0)], CART, LEGS, gate=0.4))


def drive(pose, v, w, dt=0.05):
    x, y, t = pose
    return (x + v * math.cos(t) * dt, y + v * math.sin(t) * dt, t + w * dt)


class TestRegelaar(unittest.TestCase):

    def run_until_done(self, pose, cmd, steps=4000):
        for _ in range(steps):
            out = cmd(pose)
            if out is None:
                return pose
            pose = drive(pose, *out)
        self.fail('regelaar niet klaar')

    def test_draaien(self):
        pose = self.run_until_done((0, 0, 1.0), lambda p: (
            None if rotate_cmd(p[2]) is None else (0.0, rotate_cmd(p[2]))))
        self.assertLess(abs(pose[2]), math.radians(0.3))

    def test_naar_punt(self):
        pose = self.run_until_done((0, 0, 2.5), lambda p: point_cmd(p, (0.4, -0.3)))
        self.assertLess(math.dist(pose[:2], (0.4, -0.3)), 0.01)

    def test_as_volgen_vooruit_en_achteruit(self):
        for start, stop in (((-1.0, 0.015, math.radians(1)), 0.13), ((0.13, 0.01, 0.0), -0.9)):
            pose = self.run_until_done(start, lambda p: axis_cmd(p, stop))
            self.assertLess(abs(pose[0] - stop), 0.005)
            self.assertLess(abs(pose[1]), 0.003)
            self.assertLess(abs(pose[2]), math.radians(1))


if __name__ == '__main__':
    unittest.main()
