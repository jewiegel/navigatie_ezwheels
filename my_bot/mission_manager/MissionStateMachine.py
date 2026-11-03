#!/usr/bin/env python3

from typing import Optional

from route_manager import ActionSpecification, Route, Waypoint
from .MissionEvent import MissionEvent
from .commands import (
    CancelGoalCommand, CancelTaskCommand, Command, ExecuteTaskCommand, NotifyUICommand,
    SendGoalCommand)
from .enums import MissionKind, MissionState

S, K = MissionState, MissionKind


def split_into_legs(waypoints: list[Waypoint]) -> list[list[Waypoint]]:
    """Knip de route na elk waypoint met acties: Nav2 rijdt één stuk, daarna de acties."""
    legs, leg = [], []
    for wp in waypoints:
        leg.append(wp)
        if wp.actions:
            legs.append(leg)
            leg = []
    if leg:
        legs.append(leg)
    return legs


class MissionStateMachine():
    """Pure toestandsmachine: geen ROS. handle() geeft de uit te voeren
    commands terug; de node voert ze uit via de MissionPorts.

    De route gaat in stukken naar Nav2: elk stuk eindigt op een waypoint met
    acties. Na aankomst worden die acties na elkaar uitgevoerd; geslaagd of
    (na max_retries) mislukt, daarna rijdt de robot het volgende stuk."""

    def __init__(self):
        self.state: MissionState = S.IDLE
        self.route: Optional[Route] = None
        self._legs: list[list[Waypoint]] = []
        self._leg = 0
        self._tasks: list[ActionSpecification] = []
        self._retries = 0
        # (huidige toestand, event) -> nieuwe toestand, of een functie die de nieuwe
        # toestand kiest. Niet in de tabel = event genegeerd.
        self.transitions: dict = {
            (S.IDLE,             K.START_ROUTE):    S.DRIVING,
            (S.ARRIVED,          K.START_ROUTE):    S.DRIVING,
            (S.DRIVING,          K.BT_BLOCKED):     S.BLOCKED,
            (S.BLOCKED,          K.BT_DRIVING):     S.DRIVING,
            # stuk klaar -> acties, volgend stuk of ARRIVED
            (S.DRIVING,          K.NAV_SUCCEEDED):  self._after_leg,
            (S.BLOCKED,          K.NAV_SUCCEEDED):  self._after_leg,
            (S.EXECUTING_TASKS,  K.TASK_SUCCEEDED): self._after_task,
            (S.EXECUTING_TASKS,  K.TASK_FAILED):    self._after_task,
            # BT heeft alle herstelpogingen opgegeven -> operator moet kijken
            (S.DRIVING,          K.NAV_ABORTED):    S.WAITING_OPERATOR,
            (S.BLOCKED,          K.NAV_ABORTED):    S.WAITING_OPERATOR,
            (S.DRIVING,          K.RESET):          S.IDLE,
            (S.BLOCKED,          K.RESET):          S.IDLE,
            (S.EXECUTING_TASKS,  K.RESET):          S.IDLE,
            (S.WAITING_OPERATOR, K.RESET):          S.IDLE,
            (S.ARRIVED,          K.RESET):          S.IDLE,
            (S.ERROR,            K.RESET):          S.IDLE,
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
            self._legs, self._leg = split_into_legs(event.route.waypoints), 0
            commands.append(SendGoalCommand(self._legs[0]))
        elif event.kind is K.RESET and self.state in (S.DRIVING, S.BLOCKED):
            commands.append(CancelGoalCommand())
        elif event.kind is K.RESET and self.state is S.EXECUTING_TASKS:
            commands.append(CancelTaskCommand())
        if callable(new_state):
            new_state = new_state(event, commands)

        self.state = new_state
        route = self.route.name if self.route else ''
        commands.append(NotifyUICommand(f'{new_state.name} {route}'.strip()))
        return commands

    def _after_leg(self, event, commands) -> MissionState:
        self._tasks, self._retries = list(self._legs[self._leg][-1].actions), 0
        return self._next_task(commands)

    def _after_task(self, event, commands) -> MissionState:
        if event.kind is K.TASK_FAILED and self._retries < self._tasks[0].max_retries:
            self._retries += 1
            commands.append(ExecuteTaskCommand(self._tasks[0]))
            return S.EXECUTING_TASKS
        # Geslaagd, of mislukt zonder pogingen over: altijd door met de route.
        self._tasks.pop(0)
        self._retries = 0
        return self._next_task(commands)

    def _next_task(self, commands) -> MissionState:
        if self._tasks:
            commands.append(ExecuteTaskCommand(self._tasks[0]))
            return S.EXECUTING_TASKS
        self._leg += 1
        if self._leg < len(self._legs):
            commands.append(SendGoalCommand(self._legs[self._leg]))
            return S.DRIVING
        return S.ARRIVED
