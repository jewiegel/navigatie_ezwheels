"""Unittests voor het docken onder de kar (wielen herkennen, kar vinden, rijden) — zonder ROS.

Draaien:  python3 -m unittest discover -s test -p "test_docking*" -v   (vanuit navigatie_ezwheels/)
"""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from docking import (  # noqa: E402
    axis_cmd, cart_castors, compose, find_wheels, fit_pose, fit_wheel, inverse, match_cart,
    point_cmd, points_in_area, rotate_cmd, transform, wheels_in_lane)

CASTORS = cart_castors(0.29, 0.3625)
WHEEL_L, WHEEL_W, TRAIL = 0.205, 0.05, 0.04
CART = (6.0, 6.6, 0.3)
MIXED = [20, 110, -45, 75]   # standen van de zwenkwielen in graden (kar-frame)


def wheel_boxes(cart=CART, yaws_deg=MIXED, trail=TRAIL):
    """Wielen als (midden, richting): wielmidden ligt trail achter de draai-as."""
    boxes = []
    for axis, yaw in zip(CASTORS, yaws_deg):
        a = cart[2] + math.radians(yaw)
        ax, ay = transform(cart, axis)
        boxes.append(((ax - trail * math.cos(a), ay - trail * math.sin(a)), a))
    return boxes


def scan(sensor, boxes, walls=(), noise=0.0, step=math.radians(0.375), fov=math.radians(100)):
    """Gesimuleerde lidar: stralen vanaf sensor (pose) op rechthoeken (midden, richting[, lengte,
    breedte]; standaard een wiel) en muren (x = c)."""
    rng = random.Random(1)
    segments = []
    for box in boxes:
        (cx, cy), a = box[:2]
        hl, hw = (box[2] / 2, box[3] / 2) if len(box) > 2 else (WHEEL_L / 2, WHEEL_W / 2)
        c, s = math.cos(a), math.sin(a)
        corners = [(cx + c * u - s * v, cy + s * u + c * v)
                   for u, v in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))]
        segments += [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    segments += [((x, -100), (x, 100)) for x in walls]
    points = []
    a = -fov
    while a <= fov:
        h = sensor[2] + a
        dx, dy = math.cos(h), math.sin(h)
        best = 12.0
        for (x1, y1), (x2, y2) in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - sensor[0]) * ey - (y1 - sensor[1]) * ex) / den
            u = ((x1 - sensor[0]) * dy - (y1 - sensor[1]) * dx) / den
            if 0.12 < t < best and 0 <= u <= 1:
                best = t
        if best < 12.0:
            d = best + rng.gauss(0, noise)
            points.append((sensor[0] + d * dx, sensor[1] + d * dy))
        a += step
    return points


def wheels_seen(sensor, boxes=None, noise=0.0):
    boxes = wheel_boxes() if boxes is None else boxes
    return find_wheels(scan(sensor, boxes, noise=noise), sensor[:2], WHEEL_L, WHEEL_W)


def error(pose, true=CART):
    return math.dist(pose[:2], true[:2]), abs(math.degrees(pose[2] - true[2]))


class TestGeometry(unittest.TestCase):

    def test_compose_inverse(self):
        p = (1.0, -2.0, 0.7)
        q = compose(p, inverse(p))
        self.assertAlmostEqual(q[0], 0)
        self.assertAlmostEqual(q[1], 0)
        self.assertAlmostEqual(q[2], 0)

    def test_fit_pose_uit_twee_en_vier_punten(self):
        for model in (CASTORS, CASTORS[:2], [CASTORS[0], CASTORS[3]]):
            pose, rms = fit_pose(model, [transform(CART, p) for p in model])
            for got, want in zip(pose, CART):
                self.assertAlmostEqual(got, want, places=9)
            self.assertLess(rms, 1e-9)


class TestWielen(unittest.TestCase):

    def test_wiel_in_elke_stand(self):
        sensor = (0.0, 0.0, 0.0)
        for yaw in range(0, 180, 15):   # van opzij, schuin en met de kop naar de lidar
            a = math.radians(yaw)
            box = ((1.0, 0.1), a)
            center, angle, rms = fit_wheel(scan(sensor, [box]), sensor[:2], WHEEL_L, WHEEL_W)
            self.assertLess(math.dist(center, box[0]), 0.005, f'{yaw} graden')   # hoekstap
            diff = abs((angle - a + math.pi / 2) % math.pi - math.pi / 2)
            self.assertLess(diff, math.radians(3), f'{yaw} graden')

    def test_stoelpoot_is_geen_wiel(self):
        sensor = (0.0, 0.0, 0.0)
        for size in (0.02, 0.025):   # buizen van 2-3 cm; schuin gezien tot ~3,5 cm
            legs = [((1.0, y), 0.3, size, size) for y in (-0.3, 0.0, 0.3)]
            self.assertEqual(find_wheels(scan(sensor, legs), sensor[:2], WHEEL_L, WHEEL_W), [],
                             f'poot {size} m')

    def test_muur_is_geen_wiel(self):
        sensor = (0.0, 0.0, 0.0)
        self.assertEqual(find_wheels(scan(sensor, [], walls=[2.0]), sensor[:2], WHEEL_L, WHEEL_W), [])


class TestKarVinden(unittest.TestCase):

    def test_wielen_verschillend_gedraaid_exact(self):
        sensor = compose(CART, (-1.4, 0.1, 0.1))
        prior = compose(CART, (0.2, -0.15, math.radians(6)))   # AMCL zit ernaast
        pose, n, _ = match_cart(wheels_seen(sensor), prior, CASTORS, 0.4, TRAIL)
        self.assertEqual(n, 4)
        dist, angle = error(pose)
        self.assertLess(dist, 0.004)
        self.assertLess(angle, 0.5)

    def test_wielen_allemaal_dezelfde_kant_op_hooguit_trail(self):
        sensor = compose(CART, (-1.4, 0.0, 0.0))
        for yaw in (0, 90, 30):
            boxes = wheel_boxes(yaws_deg=[yaw] * 4)
            pose, n, _ = match_cart(wheels_seen(sensor, boxes), CART, CASTORS, 0.4, TRAIL)
            self.assertEqual(n, 4)
            dist, angle = error(pose)
            self.assertLess(dist, TRAIL + 0.004, f'{yaw} graden')
            self.assertLess(angle, 0.5)

    def test_volgen_met_goede_verwachting_kiest_juiste_kant(self):
        sensor = compose(CART, (-1.0, 0.0, 0.0))
        boxes = wheel_boxes(yaws_deg=[0] * 4)
        pose, _, _ = match_cart(wheels_seen(sensor, boxes), compose(CART, (0.005, 0, 0)),
                                CASTORS, 0.12, TRAIL, trust_prior=True)
        self.assertLess(error(pose)[0], 0.004)

    def test_met_ruis_van_mediaan_over_scans(self):
        # mediaan van 10 scans met 1 cm ruis (Gazebo) ~ 4 mm ruis
        for seed in range(5):
            rng = random.Random(seed)
            sensor = compose(CART, (-1.4, rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1)))
            points = [(x + rng.gauss(0, 0.004), y + rng.gauss(0, 0.004))
                      for x, y in scan(sensor, wheel_boxes())]
            wheels = find_wheels(points, sensor[:2], WHEEL_L, WHEEL_W)
            pose, n, _ = match_cart(wheels, CART, CASTORS, 0.4, TRAIL)
            self.assertGreaterEqual(n, 3)
            self.assertLess(error(pose)[0], 0.015)

    def test_wiel_in_de_rijbaan(self):
        lane = 0.245 + 0.02
        self.assertEqual(wheels_in_lane(wheel_boxes(), CART, WHEEL_L, WHEEL_W, lane), [])
        inward = wheel_boxes(yaws_deg=[20, 110, -45, -90])   # laatste wiel recht naar binnen
        self.assertEqual(len(wheels_in_lane(inward, CART, WHEEL_L, WHEEL_W, lane)), 1)

    def test_zoekgebied_met_veel_stoorobjecten(self):
        # Rond de kar: stoelpoten (3 x 3 cm) en blokjes zo groot als een wiel, dichtbij en ver weg.
        rng = random.Random(3)
        clutter = []
        for _ in range(40):
            x, y = rng.uniform(-2.5, 2.5), rng.uniform(-2.5, 2.5)
            if abs(x) < 0.9 and abs(y) < 0.9:
                continue   # niet onder of tegen de kar
            pos = transform(CART, (x, y))
            if rng.random() < 0.5:
                clutter.append((pos, rng.uniform(0, math.pi), 0.03, 0.03))
            else:
                clutter.append((pos, rng.uniform(0, math.pi), WHEEL_L, WHEEL_W))
        sensor = compose(CART, (-1.4, 0.05, 0.05))
        points = scan(sensor, wheel_boxes() + clutter)
        everything = find_wheels(points, sensor[:2], WHEEL_L, WHEEL_W)
        half_x, half_y = 0.70 / 2 + 0.25, 0.845 / 2 + 0.25   # buitenmaat kar + zoekmarge
        near = points_in_area(points, CART, half_x, half_y)
        in_area = find_wheels(near, sensor[:2], WHEEL_L, WHEEL_W)
        self.assertGreater(len(everything), len(in_area))
        self.assertLessEqual(len(in_area), 5)
        pose, n, _ = match_cart(in_area, compose(CART, (0.15, -0.1, 0.05)), CASTORS, 0.4, TRAIL)
        self.assertEqual(n, 4)
        self.assertLess(error(pose)[0], 0.005)

    def test_grovere_lidar(self):
        sensor = compose(CART, (-1.4, 0.05, 0.05))
        step = math.radians(1.0)   # echte lidar met 1 graad per straal
        points = scan(sensor, wheel_boxes(), step=step)
        wheels = find_wheels(points, sensor[:2], WHEEL_L, WHEEL_W, angle_tol=2.5 * step)
        pose, n, _ = match_cart(wheels, CART, CASTORS, 0.4, TRAIL)
        self.assertGreaterEqual(n, 3)
        self.assertLess(error(pose)[0], 0.015)

    def test_geen_kar_geen_match(self):
        self.assertIsNone(match_cart([((0.0, 0.0), 0.0)], CART, CASTORS, 0.4, TRAIL))


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

    def test_punt_ernaast_niet_klaar_net_erachter_wel(self):
        self.assertIsNotNone(point_cmd((0, 0, 0), (0.0, 0.02)))     # 2 cm opzij: corrigeren
        self.assertIsNone(point_cmd((0, 0, 0), (-0.01, 0.005)))     # net voorbij: klaar

    def test_as_volgen_vooruit_en_achteruit(self):
        for start, stop in (((-1.0, 0.015, math.radians(1)), 0.13), ((0.13, 0.01, 0.0), -0.9)):
            pose = self.run_until_done(start, lambda p: axis_cmd(p, stop))
            self.assertLess(abs(pose[0] - stop), 0.005)
            self.assertLess(abs(pose[1]), 0.003)
            self.assertLess(abs(pose[2]), math.radians(1))


if __name__ == '__main__':
    unittest.main()
