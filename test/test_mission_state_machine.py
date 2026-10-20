"""Unittests voor de MissionStateMachine — zonder ROS.

Draaien:  python3 -m unittest discover -s test -p "test_mission*" -v   (vanuit navigatie_ezwheels/)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from mission_manager import MissionEvent, MissionStateMachine  # noqa: E402
from mission_manager.commands import (  # noqa: E402
    CancelGoalCommand, NotifyUICommand, SendGoalCommand)
from mission_manager.enums import MissionKind as K, MissionState as S  # noqa: E402
from mission_manager.ports import MissionPorts, NavigationClient, UINotifier  # noqa: E402
from route_manager import Route, Waypoint  # noqa: E402

ROUTE = Route('gang_a', 'ziekenhuis', [Waypoint('a', 1.0, 2.0, 0.0), Waypoint('b', 3.0, 4.0, 1.57)])


class FakeNav(NavigationClient):
    def __init__(self):
        self.sent, self.cancels = [], 0

    def send_goal(self, waypoints):
        self.sent.append(waypoints)

    def cancel(self):
        self.cancels += 1


class FakeUI(UINotifier):
    def __init__(self):
        self.messages = []

    def notify(self, message):
        self.messages.append(message)


def start(sm):
    return sm.handle(MissionEvent(K.START_ROUTE, ROUTE.name, ROUTE))


class TestMissionStateMachine(unittest.TestCase):

    def test_start_stuurt_doel_en_meldt_ui(self):
        sm = MissionStateMachine()
        nav, ui = FakeNav(), FakeUI()
        for cmd in start(sm):
            cmd.execute(MissionPorts(nav, ui))
        self.assertIs(sm.state, S.DRIVING)
        self.assertEqual(nav.sent, [ROUTE.waypoints])
        self.assertEqual(ui.messages, ['DRIVING gang_a'])

    def test_blokkade_en_weer_rijden(self):
        sm = MissionStateMachine()
        start(sm)
        sm.handle(MissionEvent(K.BT_BLOCKED))
        self.assertIs(sm.state, S.BLOCKED)
        sm.handle(MissionEvent(K.BT_DRIVING))
        self.assertIs(sm.state, S.DRIVING)

    def test_aankomen_en_opnieuw_starten(self):
        sm = MissionStateMachine()
        start(sm)
        sm.handle(MissionEvent(K.NAV_SUCCEEDED))
        self.assertIs(sm.state, S.ARRIVED)
        self.assertIsNotNone(start(sm))
        self.assertIs(sm.state, S.DRIVING)

    def test_nav_aborted_wacht_op_operator_tot_reset(self):
        sm = MissionStateMachine()
        start(sm)
        sm.handle(MissionEvent(K.NAV_ABORTED))
        self.assertIs(sm.state, S.WAITING_OPERATOR)
        self.assertIsNone(start(sm))   # eerst resetten
        commands = sm.handle(MissionEvent(K.RESET))
        self.assertIs(sm.state, S.IDLE)
        self.assertFalse(any(isinstance(c, CancelGoalCommand) for c in commands))

    def test_reset_tijdens_rijden_annuleert_doel(self):
        sm = MissionStateMachine()
        start(sm)
        commands = sm.handle(MissionEvent(K.RESET))
        self.assertIsInstance(commands[0], CancelGoalCommand)
        self.assertIsInstance(commands[-1], NotifyUICommand)
        self.assertIs(sm.state, S.IDLE)

    def test_start_tijdens_rijden_genegeerd(self):
        sm = MissionStateMachine()
        start(sm)
        self.assertIsNone(start(sm))
        self.assertIs(sm.state, S.DRIVING)

    def test_bt_events_in_idle_genegeerd(self):
        sm = MissionStateMachine()
        for kind in (K.BT_BLOCKED, K.BT_DRIVING, K.NAV_SUCCEEDED, K.NAV_ABORTED):
            self.assertIsNone(sm.handle(MissionEvent(kind)))
        self.assertIs(sm.state, S.IDLE)

    def test_alleen_start_stuurt_doel(self):
        sm = MissionStateMachine()
        self.assertIsInstance(start(sm)[0], SendGoalCommand)
        for kind in (K.BT_BLOCKED, K.BT_DRIVING, K.NAV_SUCCEEDED):
            commands = sm.handle(MissionEvent(kind)) or []
            self.assertFalse(any(isinstance(c, SendGoalCommand) for c in commands))


if __name__ == '__main__':
    unittest.main()
