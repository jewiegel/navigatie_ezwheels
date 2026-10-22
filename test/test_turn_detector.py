"""Unittests voor de bochtherkenning — zonder ROS.

Draaien:  python3 -m unittest discover -s test -p "test_turn*" -v   (vanuit navigatie_ezwheels/)
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from turn_detection import (  # noqa: E402
    LEFT, NONE, RIGHT, extend_with_route, find_turn, turning_direction)

TURN = math.radians(30)


def line(x0, y0, heading_deg, length, step=0.05):
    h = math.radians(heading_deg)
    n = int(round(length / step))
    return [(x0 + i * step * math.cos(h), y0 + i * step * math.sin(h)) for i in range(n + 1)]


def corner(straight, heading_after_deg, after=3.0):
    """Recht vooruit (+x) over `straight` m, dan een knik naar heading_after_deg."""
    first = line(0, 0, 0, straight)
    x, y = first[-1]
    return first + line(x, y, heading_after_deg, after)[1:]


def grid(points, cell=0.05):
    """Pad op het costmap-rooster leggen, zoals NavFn het teruggeeft."""
    out = []
    for x, y in points:
        p = (round(x / cell) * cell, round(y / cell) * cell)
        if not out or out[-1] != p:
            out.append(p)
    return out


class TestFindTurn(unittest.TestCase):

    def test_recht_pad_geen_bocht(self):
        self.assertIsNone(find_turn(line(0, 0, 0, 5), 3.0, TURN))

    def test_schuin_pad_op_rooster_geen_bocht(self):
        # Trapjes van het rooster mogen niet als bocht tellen.
        self.assertIsNone(find_turn(grid(line(0, 0, 20, 5)), 3.0, TURN))

    def test_bocht_links_met_afstand(self):
        turn = find_turn(corner(1.5, 90), 3.0, TURN)
        self.assertEqual(turn.direction, LEFT)
        self.assertAlmostEqual(turn.distance, 1.5, delta=0.5)
        self.assertAlmostEqual(math.degrees(turn.angle), 90, delta=10)
        self.assertAlmostEqual(turn.point[0], 1.5, delta=0.5)   # bochtpunt op het pad
        self.assertAlmostEqual(turn.point[1], 0.0, delta=0.3)

    def test_bocht_rechts_op_rooster(self):
        turn = find_turn(grid(corner(2.0, -90)), 3.0, TURN)
        self.assertEqual(turn.direction, RIGHT)
        self.assertAlmostEqual(turn.distance, 2.0, delta=0.5)
        self.assertLess(turn.angle, 0)

    def test_bocht_achter_lookahead_telt_niet(self):
        self.assertIsNone(find_turn(corner(4.0, 90), 3.0, TURN))

    def test_kleine_koerscorrectie_geen_bocht(self):
        self.assertIsNone(find_turn(corner(1.0, 15), 3.0, TURN))

    def test_bocht_vlak_voor_de_robot(self):
        turn = find_turn(corner(0.6, -90), 3.0, TURN)
        self.assertEqual(turn.direction, RIGHT)
        self.assertLess(turn.distance, 1.0)

    def test_bocht_binnen_eerste_stap_alleen_met_robotrichting(self):
        # Vlak voor het waypoint: zonder robotrichting is het eerste stuk al de bocht.
        self.assertIsNone(find_turn(corner(0.1, -90), 3.0, TURN))
        turn = find_turn(corner(0.1, -90), 3.0, TURN, heading=0.0)
        self.assertEqual(turn.direction, RIGHT)
        self.assertEqual(turn.distance, 0.0)
        self.assertAlmostEqual(math.degrees(turn.angle), -90, delta=10)

    def test_robotrichting_zelfde_afstand_als_zonder(self):
        turn = find_turn(corner(1.5, 90), 3.0, TURN, heading=0.0)
        self.assertAlmostEqual(turn.distance, find_turn(corner(1.5, 90), 3.0, TURN).distance)

    def test_robotrichting_recht_pad_geen_bocht(self):
        self.assertIsNone(find_turn(grid(line(0, 0, 20, 5)), 3.0, TURN,
                                    heading=math.radians(25)))

    def test_te_kort_of_leeg_pad(self):
        self.assertIsNone(find_turn([], 3.0, TURN))
        self.assertIsNone(find_turn([(0, 0), (0.1, 0)], 3.0, TURN))


class TestExtendWithRoute(unittest.TestCase):

    ROUTE = [(2.0, 0.0), (2.0, 3.0), (5.0, 3.0)]

    def test_bocht_bij_waypoint_wordt_zichtbaar(self):
        plan = line(0, 0, 0, 2.0)                  # naar waypoint 0, recht
        self.assertIsNone(find_turn(plan, 3.0, TURN))
        turn = find_turn(extend_with_route(plan, self.ROUTE, 3), 3.0, TURN)
        self.assertEqual(turn.direction, LEFT)
        self.assertAlmostEqual(turn.distance, 2.0, delta=0.5)

    def test_route_langs_zelfde_punt_twee_keer(self):
        # Heen en terug door de gang: (2, 0) komt twee keer voor; op positie matchen gaf de
        # vervolgwaypoints van het eerste bezoek (keren) i.p.v. de bocht erna.
        route = [(2.0, 0.0), (6.0, 0.0), (2.0, 0.0), (2.0, -3.0)]
        plan = line(6.0, 0.0, 180, 4.0)            # terug naar (2, 0), het tweede bezoek
        turn = find_turn(extend_with_route(plan, route, 2), 5.0, TURN)
        self.assertEqual(turn.direction, LEFT)     # rijdt richting -x, dan -y: linksaf
        self.assertEqual(extend_with_route(plan, route, 2)[-1], (2.0, -3.0))

    def test_laatste_waypoint_niets_erachter(self):
        plan = line(5.0, 0.0, 90, 3.0)
        self.assertEqual(extend_with_route(plan, self.ROUTE, 1), plan)

    def test_onbekend_aantal_of_leeg(self):
        plan = line(0, 0, 0, 2.0)
        self.assertEqual(extend_with_route(plan, self.ROUTE, 0), plan)    # geen feedback
        self.assertEqual(extend_with_route(plan, self.ROUTE, 9), plan)    # andere route
        self.assertEqual(extend_with_route([], self.ROUTE, 3), [])


class TestTurningDirection(unittest.TestCase):

    def test_drempel(self):
        self.assertEqual(turning_direction(0.5, 0.25), LEFT)
        self.assertEqual(turning_direction(-0.5, 0.25), RIGHT)
        self.assertEqual(turning_direction(0.1, 0.25), NONE)


if __name__ == '__main__':
    unittest.main()
