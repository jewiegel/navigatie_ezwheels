"""Unittests voor het docken onder de kar (kar meten in de ROI, rijbaan, rijden) — zonder ROS.

Ook op drie echte scans van de zorgkar (test/data/echte_kar_*.json; 1 en 2: mediaan van 10
scans vanaf ~1,3 m met veel rafels, 3: stabiele punten van 30 scans, zie 'note').

Draaien:  python3 -m unittest discover -s test -p "test_docking*" -v   (vanuit navigatie_ezwheels/)
"""
import json
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from docking import (  # noqa: E402
    axis_cmd, compose, inverse, measure_cart, point_cmd, points_in_lane, rotate_cmd,
    stable_points, transform)

# De echte zorgkar: draai-assen van de zwenkwielen, wiel op lidarhoogte, naloop
CASTORS = [(0.31, 0.35), (0.31, -0.35), (-0.31, 0.35), (-0.31, -0.35)]
WHEEL_L, WHEEL_W, TRAIL = 0.19, 0.03, 0.03
HALF = (0.35, 0.4225)          # halve buitenmaat (cart_length, cart_width)
LANE = 2 * 0.245               # robotbreedte (simulatie)
CART = (6.0, 6.6, 0.3)
MIXED = [20, 110, -45, 75]   # standen van de zwenkwielen in graden (kar-frame)
DATA = os.path.join(os.path.dirname(__file__), 'data')


def wheel_boxes(cart=CART, yaws_deg=MIXED, trail=TRAIL):
    """Wielen als (midden, richting): wielmidden ligt trail achter de draai-as."""
    boxes = []
    for axis, yaw in zip(CASTORS, yaws_deg):
        a = cart[2] + math.radians(yaw)
        ax, ay = transform(cart, axis)
        boxes.append(((ax - trail * math.cos(a), ay - trail * math.sin(a)), a))
    return boxes


def scan(sensor, boxes, walls=(), noise=0.0, step=math.radians(0.375), fov=math.radians(100),
         seed=1):
    """Gesimuleerde lidar: stralen vanaf sensor (pose) op rechthoeken (midden, richting[, lengte,
    breedte]; standaard een wiel) en muren (x = c)."""
    rng = random.Random(seed)
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


def error(pose, true=CART):
    """(diepte, zijwaarts, hoek in graden) van pose t.o.v. true, in het kar-frame."""
    e = compose(inverse(true), pose)
    return abs(e[0]), abs(e[1]), abs(math.degrees(e[2]))


def measure(points, prior):
    result, why = measure_cart(points, prior, *HALF, 0.25, LANE)
    return result, why


class TestGeometry(unittest.TestCase):

    def test_compose_inverse(self):
        p = (1.0, -2.0, 0.7)
        q = compose(p, inverse(p))
        self.assertAlmostEqual(q[0], 0)
        self.assertAlmostEqual(q[1], 0)
        self.assertAlmostEqual(q[2], 0)


class TestStabielePunten(unittest.TestCase):

    def test_losse_ruis_valt_weg_vaste_punten_blijven(self):
        rng = random.Random(4)
        wall = [(1.0, 0.01 * i) for i in range(50)]
        scans = []
        for _ in range(30):
            ghost = [(rng.uniform(0, 2), rng.uniform(0, 2)) for _ in range(20)]
            flaky = [(0.5, 0.5)] if rng.random() < 0.4 else []   # slecht weerkaatsend: 40%
            scans.append([(x + rng.gauss(0, 0.002), y) for x, y in wall] + ghost + flaky)
        kept = stable_points(scans)
        self.assertGreaterEqual(len(kept), 45)
        self.assertTrue(any(math.dist(p, (0.5, 0.5)) < 0.02 for p in kept))
        self.assertLess(sum(1 for p in kept if abs(p[0] - 1.0) > 0.02 and
                            math.dist(p, (0.5, 0.5)) > 0.02), 3)


class TestKarMeten(unittest.TestCase):
    """Synthetische scans. Grenzen = wat de aanpak haalt: midden van de punten per wiel is
    niet de draai-as. Een slecht zichtbaar achterwiel (4 punten) geeft ~2 cm / 2 graden;
    diepte hangt af van hoe de wielen staan (dwars: ~8 cm; depth_offset)."""

    def check(self, result, why, side=0.025, angle=2.5):
        self.assertIsNotNone(result, why)
        depth, lateral, turn = error(result['pose'])
        self.assertLess(lateral, side)
        self.assertLess(turn, angle)
        self.assertLess(depth, 0.10)

    def test_scheve_startgok_wielen_verschillend_gedraaid(self):
        sensor = compose(CART, (-1.3, 0.1, 0.1))
        prior = compose(CART, (0.15, -0.12, math.radians(5)))   # AMCL zit ernaast
        self.check(*measure(scan(sensor, wheel_boxes()), prior))

    def test_wielen_recht_en_dwars(self):
        sensor = compose(CART, (-1.3, 0.0, 0.0))
        for yaw in (0, 90):   # alle vier dwars dezelfde kant op: midden schuift de naloop op
            self.check(*measure(scan(sensor, wheel_boxes(yaws_deg=[yaw] * 4)),
                                compose(CART, (-0.1, 0.1, -0.05))), side=TRAIL + 0.01)

    def test_muur_achter_de_kar_en_stoelen_ernaast(self):
        rng = random.Random(7)
        clutter = [(transform(CART, (rng.choice([-1, 1]) * rng.uniform(0.8, 1.6),
                                     rng.uniform(-1.2, 1.2))), rng.uniform(0, 3), 0.03, 0.03)
                   for _ in range(25)]                       # stoelpoten rond de kar
        clutter += [(transform(CART, (0.62, 0.0)), CART[2], 0.02, 3.0)]   # muur erachter
        sensor = compose(CART, (-1.3, 0.05, 0.05))
        points = [p for p in scan(sensor, wheel_boxes() + clutter, noise=0.005)
                  if rng.random() > 0.25]                   # kwart van de punten weg
        self.check(*measure(points, compose(CART, (0.2, 0.1, -0.05))))

    def test_geen_kar_geen_resultaat(self):
        sensor = compose(CART, (-1.3, 0.0, 0.0))
        result, why = measure(scan(sensor, [], walls=[8.0]), CART)
        self.assertIsNone(result)
        self.assertTrue(why)

    def test_wiel_in_de_rijbaan(self):
        lane, half_x = 0.245 + 0.02, 0.35
        sensor = compose(CART, (-1.0, 0.0, 0.0))
        free = scan(sensor, wheel_boxes())
        self.assertEqual(points_in_lane(free, CART, half_x, lane), [])
        inward = scan(sensor, wheel_boxes(yaws_deg=[20, 110, 90, 75]))   # wiel 2 recht naar binnen
        self.assertGreater(len(points_in_lane(inward, CART, half_x, lane)), 0)


class TestEchteKar(unittest.TestCase):
    """Drie echte scans van de zorgkar: de kar moet gevonden worden, en op dezelfde plek, ook
    als AMCL er anders naast zit."""

    def check(self, name, side=0.005, angle=0.5, may_miss=False):
        """Rond de opgegeven plek met andere startgokken meten: alle uitkomsten liggen bij
        elkaar. may_miss: niet vinden mag (slecht zichtbaar wiel), verkeerd vinden niet."""
        with open(os.path.join(DATA, name)) as f:
            d = json.load(f)
        rng = random.Random(11)
        priors = [d['prior']] + [compose(d['prior'], (rng.uniform(-0.15, 0.15),
                                                      rng.uniform(-0.15, 0.15),
                                                      math.radians(rng.uniform(-5, 5))))
                                 for _ in range(8)]
        found = []
        for prior in priors:
            r, why = measure(d['points'], prior)
            if r is None and may_miss:
                continue
            self.assertIsNotNone(r, f'{name}: {why}')
            found.append(r['pose'])
        if not may_miss:
            self.assertEqual(len(found), len(priors))
        for pose in found[1:]:
            depth, lateral, turn = error(pose, found[0])
            self.assertLess(lateral, side, name)
            self.assertLess(depth, 0.03, name)
            self.assertLess(turn, angle, name)

    # 1 en 2: oude, rafelige scans (mediaan van 10, zonder stabiele punten): ruimere grens.
    # In 1 heeft het wiel rechtsachter maar 2 punten (plus een spookpunt): wordt niet
    # gevonden (robot stopt met melding); de test bewaakt dat hij nooit iets verkeerds vindt.
    def test_echte_kar_1(self):
        self.check('echte_kar_1.json', 0.02, 1.5, may_miss=True)

    def test_echte_kar_2(self):
        self.check('echte_kar_2.json', 0.02, 1.5)

    def test_echte_kar_3(self):
        self.check('echte_kar_3.json')


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
