#!/usr/bin/env python3

from typing import Optional

from route_manager import Route
from .MissionEvent import MissionEvent
from .commands import CancelGoalCommand, Command, NotifyUICommand, SendGoalCommand
from .enums import MissionKind, MissionState

S, K = MissionState, MissionKind


class MissionStateMachine():
    """Pure toestandsmachine: geen ROS. handle() geeft de uit te voeren
    commands terug; de node voert ze uit via de MissionPorts."""

    def __init__(self):
        self.state: MissionState = S.IDLE
        self.route: Optional[Route] = None
        # (huidige toestand, event) -> nieuwe toestand. Niet in de tabel = event genegeerd.
        self.transitions: dict = {
            (S.IDLE,             K.START_ROUTE):   S.DRIVING,
            (S.ARRIVED,          K.START_ROUTE):   S.DRIVING,
            (S.DRIVING,          K.BT_BLOCKED):    S.BLOCKED,
            (S.BLOCKED,          K.BT_DRIVING):    S.DRIVING,
            (S.DRIVING,          K.NAV_SUCCEEDED): S.ARRIVED,
            (S.BLOCKED,          K.NAV_SUCCEEDED): S.ARRIVED,
            # BT heeft alle herstelpogingen opgegeven -> operator moet kijken
            (S.DRIVING,          K.NAV_ABORTED):   S.WAITING_OPERATOR,
            (S.BLOCKED,          K.NAV_ABORTED):   S.WAITING_OPERATOR,
            (S.DRIVING,          K.RESET):         S.IDLE,
            (S.BLOCKED,          K.RESET):         S.IDLE,
            (S.WAITING_OPERATOR, K.RESET):         S.IDLE,
            (S.ARRIVED,          K.RESET):         S.IDLE,
            (S.ERROR,            K.RESET):         S.IDLE,
        }

    def handle(self, event: MissionEvent) -> Optional[list[Command]]:
        """Geeft de commands voor deze overgang, of None als het event in
        de huidige toestand niet is toegestaan."""
        new_state = self.transitions.get((self.state, event.kind))
        if new_state is None:
            return None

        commands: list[Command] = []
        if event.kind is K.START_ROUTE:
            self.route = event.route
            commands.append(SendGoalCommand(event.route.waypoints))
        elif event.kind is K.RESET and self.state in (S.DRIVING, S.BLOCKED):
            commands.append(CancelGoalCommand())

        self.state = new_state
        route = self.route.name if self.route else ''
        commands.append(NotifyUICommand(f'{new_state.name} {route}'.strip()))
        return commands
