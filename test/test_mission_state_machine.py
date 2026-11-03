"""Unittests voor de MissionStateMachine — zonder ROS.

Draaien:  python3 -m unittest discover -s test -p "test_mission*" -v   (vanuit navigatie_ezwheels/)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from mission_manager import MissionEvent, MissionStateMachine  # noqa: E402
from mission_manager.commands import (  # noqa: E402
    CancelGoalCommand, CancelTaskCommand, ExecuteTaskCommand, NotifyUICommand, SendGoalCommand)
from mission_manager.enums import MissionKind as K, MissionState as S  # noqa: E402
from mission_manager.ports import MissionPorts, NavigationClient, UINotifier  # noqa: E402
from route_manager import ActionSpecification, Route, Waypoint  # noqa: E402

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


WAIT = ActionSpecification('wait', {'seconds': 1}, max_retries=1)
SPIN = ActionSpecification('spin', max_retries=0)
TASK_ROUTE = Route('taken', 'ziekenhuis', [
    Waypoint('a', 1, 0, 0), Waypoint('b', 2, 0, 0, [WAIT, SPIN]),
    Waypoint('c', 3, 0, 0), Waypoint('d', 4, 0, 0, [SPIN])])


def of(commands, cls):
    return [c for c in commands if isinstance(c, cls)]


class TestWaypointActions(unittest.TestCase):

    def setUp(self):
        self.sm = MissionStateMachine()
        self.first = self.sm.handle(MissionEvent(K.START_ROUTE, TASK_ROUTE.name, TASK_ROUTE))

    def test_route_wordt_geknipt_na_waypoint_met_acties(self):
        self.assertEqual([w.name for w in of(self.first, SendGoalCommand)[0].waypoints], ['a', 'b'])

    def test_acties_na_elkaar_dan_volgend_stuk(self):
        cmds = self.sm.handle(MissionEvent(K.NAV_SUCCEEDED))
        self.assertIs(self.sm.state, S.EXECUTING_TASKS)
        self.assertIs(of(cmds, ExecuteTaskCommand)[0].task, WAIT)
        cmds = self.sm.handle(MissionEvent(K.TASK_SUCCEEDED))
        self.assertIs(of(cmds, ExecuteTaskCommand)[0].task, SPIN)
        cmds = self.sm.handle(MissionEvent(K.TASK_SUCCEEDED))
        self.assertIs(self.sm.state, S.DRIVING)
        self.assertEqual([w.name for w in of(cmds, SendGoalCommand)[0].waypoints], ['c', 'd'])

    def test_mislukte_actie_opnieuw_en_daarna_toch_verder(self):
        self.sm.handle(MissionEvent(K.NAV_SUCCEEDED))
        cmds = self.sm.handle(MissionEvent(K.TASK_FAILED))       # wait: 1 retry
        self.assertIs(of(cmds, ExecuteTaskCommand)[0].task, WAIT)
        cmds = self.sm.handle(MissionEvent(K.TASK_FAILED))       # opgebruikt -> spin
        self.assertIs(of(cmds, ExecuteTaskCommand)[0].task, SPIN)
        cmds = self.sm.handle(MissionEvent(K.TASK_FAILED))       # spin: 0 retries -> rijden
        self.assertIs(self.sm.state, S.DRIVING)
        self.assertEqual(len(of(cmds, SendGoalCommand)), 1)

    def test_actie_op_laatste_waypoint_daarna_arrived(self):
        for kind in (K.NAV_SUCCEEDED, K.TASK_SUCCEEDED, K.TASK_SUCCEEDED, K.NAV_SUCCEEDED):
            self.sm.handle(MissionEvent(kind))
        self.assertIs(self.sm.state, S.EXECUTING_TASKS)
        self.sm.handle(MissionEvent(K.TASK_SUCCEEDED))
        self.assertIs(self.sm.state, S.ARRIVED)

    def test_reset_tijdens_actie_annuleert_actie(self):
        self.sm.handle(MissionEvent(K.NAV_SUCCEEDED))
        cmds = self.sm.handle(MissionEvent(K.RESET))
        self.assertEqual(len(of(cmds, CancelTaskCommand)), 1)
        self.assertIs(self.sm.state, S.IDLE)

    def test_nav_en_bt_events_tijdens_actie_genegeerd(self):
        self.sm.handle(MissionEvent(K.NAV_SUCCEEDED))
        for kind in (K.NAV_SUCCEEDED, K.NAV_ABORTED, K.BT_BLOCKED, K.START_ROUTE):
            self.assertIsNone(self.sm.handle(MissionEvent(kind, '', TASK_ROUTE)))

    def test_route_zonder_acties_in_een_keer(self):
        sm = MissionStateMachine()
        self.assertEqual(of(start(sm), SendGoalCommand)[0].waypoints, ROUTE.waypoints)
        sm.handle(MissionEvent(K.NAV_SUCCEEDED))
        self.assertIs(sm.state, S.ARRIVED)


if __name__ == '__main__':
    unittest.main()
