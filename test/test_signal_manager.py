"""Unittests voor de signal_manager (sources en actuators) — zonder ROS.

Draaien:  python3 -m unittest discover -s test -p "test_signal*" -v   (vanuit navigatie_ezwheels/)
"""
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from signal_manager.RobotFacts import (  # noqa: E402
    CollisionAction, RobotFacts, RobotMode, TurnDirection)
from signal_manager.actuators.Blinkers import Blinkers, BlinkerState  # noqa: E402
from signal_manager.actuators.Horn import Horn, HornPattern  # noqa: E402
from signal_manager.sources.CollisionStateSource import CollisionStateSource  # noqa: E402
from signal_manager.sources.RobotStateSource import RobotStateSource  # noqa: E402
from signal_manager.sources.TurnSource import TurnSource  # noqa: E402

LEFT, RIGHT, NONE = TurnDirection.LEFT, TurnDirection.RIGHT, TurnDirection.NONE
DRIVING = RobotMode.DRIVING


def facts(**kwargs):
    return RobotFacts(**{'mode': DRIVING, **kwargs})


class TestSources(unittest.TestCase):

    def turn(self, direction, turning, distance=0.0):
        source, f = TurnSource(timeout_s=1.0, at_turn_m=0.5), RobotFacts()
        source.update(SimpleNamespace(direction=direction, turning=turning,
                                      distance=distance), 0.0)
        source.apply(f)
        return f.turn

    def test_turn_onderweg_gaat_komende_bocht_voor(self):
        # Stuurcorrectie naar rechts op 2 m voor een linkse bocht: blijft links.
        self.assertEqual(self.turn(LEFT, RIGHT, distance=2.0), LEFT)

    def test_turn_bij_de_bocht_gaat_draaien_voor(self):
        # Op het waypoint eerst rechtsom naar de yaw, daarna pas links de bocht in.
        self.assertEqual(self.turn(LEFT, RIGHT, distance=0.0), RIGHT)
        self.assertEqual(self.turn(LEFT, NONE, distance=0.0), LEFT)    # draait (nog) niet

    def test_turn_korte_dip_in_draaien_springt_niet_om(self):
        source, f = TurnSource(timeout_s=1.0, at_turn_m=0.5, turning_hold_s=1.0), RobotFacts()
        for now, turning, expected in ((0.0, RIGHT, RIGHT), (0.4, NONE, RIGHT),
                                       (0.8, RIGHT, RIGHT), (2.0, NONE, LEFT)):
            source.update(SimpleNamespace(direction=LEFT, turning=turning, distance=0.0), now)
            source.apply(f)
            self.assertEqual(f.turn, expected, now)

    def test_turn_zonder_bocht_alleen_draaien(self):
        self.assertEqual(self.turn(NONE, RIGHT), RIGHT)
        self.assertEqual(self.turn(NONE, NONE), NONE)

    def test_turn_versheid(self):
        source = TurnSource(timeout_s=1.0)
        self.assertFalse(source.is_fresh(0.0))      # nog niets ontvangen
        source.update(SimpleNamespace(direction=NONE, turning=NONE, distance=0.0), 10.0)
        self.assertTrue(source.is_fresh(10.5))
        self.assertFalse(source.is_fresh(11.5))

    def test_mission_status_met_routenaam(self):
        source, f = RobotStateSource(), RobotFacts()
        source.update(SimpleNamespace(data='DRIVING gang_b'), 0.0)
        source.apply(f)
        self.assertEqual(f.mode, DRIVING)
        source.update(SimpleNamespace(data='ONZIN'), 0.0)   # onbekend: blijft staan
        source.apply(f)
        self.assertEqual(f.mode, DRIVING)

    def test_blocked(self):
        source, f = CollisionStateSource(), RobotFacts()
        source.update(SimpleNamespace(data=True), 0.0)
        source.apply(f)
        self.assertEqual(f.collision, CollisionAction.BLOCKED)


class TestBlinkers(unittest.TestCase):

    def setUp(self):
        self.blinkers = Blinkers(hold_s=1.0)

    def state(self, now, **kwargs):
        self.blinkers.update(facts(**kwargs), now)
        return self.blinkers.state

    def test_bocht_en_nabranden(self):
        self.assertEqual(self.state(0.0, turn=LEFT), BlinkerState.LEFT)
        self.assertEqual(self.state(0.5), BlinkerState.LEFT)
        self.assertEqual(self.state(1.0), BlinkerState.OFF)

    def test_andere_kant_direct(self):
        self.state(0.0, turn=LEFT)
        self.assertEqual(self.state(0.1, turn=RIGHT), BlinkerState.RIGHT)

    def test_alarmlichten(self):
        self.assertEqual(self.state(0.0, turn=LEFT, collision=CollisionAction.BLOCKED),
                         BlinkerState.HAZARD)
        self.assertEqual(self.state(0.1, mode=RobotMode.WAITING_OPERATOR), BlinkerState.HAZARD)
        self.assertEqual(self.state(0.2, safety_inputs_ok=False), BlinkerState.HAZARD)

    def test_niet_vers_buiten_rijden_geen_alarm(self):
        self.assertEqual(self.state(0.0, mode=RobotMode.IDLE, safety_inputs_ok=False),
                         BlinkerState.OFF)


class TestHorn(unittest.TestCase):

    def setUp(self):
        self.horn = Horn()

    def pattern(self, **kwargs):
        self.horn.update(RobotFacts(**kwargs), 0.0)
        pattern, self.horn.pending = self.horn.pending, None
        return pattern

    def test_vertrek_en_aankomst(self):
        self.assertEqual(self.pattern(mode=DRIVING), HornPattern.SINGLE)
        self.assertIsNone(self.pattern(mode=DRIVING))                       # alleen bij wissel
        self.assertEqual(self.pattern(mode=RobotMode.ARRIVED), HornPattern.SINGLE)

    def test_blokkade_herhaalt_tot_uit(self):
        self.pattern(mode=DRIVING)
        blocked = {'mode': RobotMode.BLOCKED, 'collision': CollisionAction.BLOCKED}
        self.assertEqual(self.pattern(**blocked), HornPattern.EVERY_5S)
        self.assertIsNone(self.pattern(**blocked))                          # één keer sturen
        self.assertEqual(self.pattern(mode=DRIVING), HornPattern.OFF)       # stoppen
        self.assertIsNone(self.pattern(mode=DRIVING))                       # geen nieuw vertrek

    def test_wachten_op_operator_dubbel(self):
        self.pattern(mode=DRIVING)
        self.assertEqual(self.pattern(mode=RobotMode.WAITING_OPERATOR), HornPattern.DOUBLE)

    def test_eenmalig_niet_tijdens_herhalen(self):
        self.pattern(mode=DRIVING)
        self.pattern(mode=DRIVING, collision=CollisionAction.BLOCKED)
        self.assertIsNone(self.pattern(mode=RobotMode.WAITING_OPERATOR,
                                       collision=CollisionAction.BLOCKED))


if __name__ == '__main__':
    unittest.main()
