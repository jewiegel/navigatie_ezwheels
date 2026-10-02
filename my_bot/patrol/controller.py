"""PatrolController: de NavigationContext van het State-pattern.

Houdt de actieve toestand en de route-voortgang vast en biedt de acties die
toestanden en ladderstappen gebruiken. Bevat geen ROS-code: alle I/O loopt via
de ports (navigator, signals, scheduler, log), zodat dit volledig zonder robot
te testen is.
"""
from __future__ import annotations

from typing import List, Optional

from .blockage_ladder import DEFAULT_LADDER, WaitStep
from .config import PatrolConfig, Waypoint
from .navigation_states import (
    CompletedState, DrivingState, ErrorState, IdleState, NavigationState,
    ReturningState, StartingState, StoppedState)


class PatrolController:

    def __init__(self, config: PatrolConfig, navigator, signals, scheduler, logger,
                 ladder=DEFAULT_LADDER):
        self.config    = config
        self.navigator = navigator
        self.signals   = signals
        self.scheduler = scheduler
        self.log       = logger
        self._ladder   = tuple(ladder)
        self._routes   = {r.route_id: r for r in config.routes}
        self._listeners = []

        self.state: Optional[NavigationState] = None
        self.route_id: Optional[int] = None
        self.waypoints: List[Waypoint] = []
        self.current_index       = 0
        self.last_successful_idx = 0
        self.trying_alternative  = False
        self.goal_retries        = 0
        self.safety_stopped      = False
        self.ladder_index        = 0   # volgende ladderstap bij een blokkade

    # ── Observers ─────────────────────────────────────────────────────────────

    def add_listener(self, listener) -> None:
        self._listeners.append(listener)

    def _notify(self, old, new) -> None:
        for listener in self._listeners:
            listener.on_state_changed(old, new)

    # ── Toestandswissel ───────────────────────────────────────────────────────

    def begin(self) -> None:
        """Start in idle. Aanroepen nadat de listeners zijn toegevoegd."""
        self.transition(IdleState(self))

    def transition(self, new_state: NavigationState) -> None:
        old_state = self.state
        if old_state is not None:
            old_state.leave()
        self.state = new_state
        self.safety_stopped = False
        self._notify(old_state, new_state)
        new_state.enter()

    def set_safety_stopped(self, value: bool) -> None:
        self.safety_stopped = value
        self._notify(self.state, self.state)   # gepubliceerde naam kan veranderen

    # ── Commando's van buitenaf (HMI / topics) ────────────────────────────────

    def start(self, route_id: int) -> None:
        route = self._routes.get(route_id)
        if route is None:
            self.log.warn(f'Start genegeerd — onbekende route {route_id}')
            return
        if not self.state.can_start:
            self.log.warn(f'Start genegeerd — robot is momenteel: {self.state}')
            return
        # Altijd vanaf het begin starten (geen voortgang onthouden).
        self.log.info(f'Startsignaal route {route_id} ontvangen — start bij waypoint 1')
        self.route_id            = route_id
        self.waypoints           = list(route.waypoints)
        self.current_index       = 0
        self.last_successful_idx = 0
        self.trying_alternative  = False
        self.goal_retries        = 0
        self.ladder_index        = 0
        self.signals.stop_wheels()
        self.navigator.select_planner(self.config.planner_id)
        self.transition(StartingState(self))

    def stop(self) -> None:
        if not self.state.can_stop:
            return
        self.log.info('Stopsignaal ontvangen — route wordt onderbroken')
        self.transition(StoppedState(self))

    # ── Acties voor toestanden en ladderstappen ───────────────────────────────

    def drive_remaining(self) -> None:
        self.transition(DrivingState(self))

    def return_to_last(self) -> None:
        self.transition(ReturningState(self))

    def complete(self) -> None:
        self.log.info('Volledige route voltooid — robot stopt, gevarenlichten aan.')
        self.last_successful_idx = len(self.waypoints) - 1
        self.trying_alternative  = False
        self.ladder_index        = 0
        self.transition(CompletedState(self))

    def fail(self, message: str) -> None:
        self.log.error(message)
        self.transition(ErrorState(self))

    def waypoint_passed(self, new_index: int, poses_remaining: int) -> None:
        old = self.current_index
        self.current_index       = new_index
        self.last_successful_idx = old
        # Voortgang geboekt → blokkade voorbij: een vólgend obstakel krijgt
        # weer de volledige ladder (wacht → achteruit → ...).
        self.ladder_index        = 0
        self.trying_alternative  = False
        self.log.info(
            f'Waypoint {old + 1} gepasseerd → navigeert naar {new_index + 1} '
            f'({poses_remaining} resterend)')

    def on_blocked(self) -> None:
        """Voer de volgende stap van de escalatieladder uit."""
        if self.ladder_index >= len(self._ladder):
            self.fail('Blokkade-ladder uitgeput — operator ingrijpen vereist')
            return
        step = self._ladder[self.ladder_index]
        self.ladder_index += 1
        step.execute(self)

    def on_person_blocked(self) -> None:
        """Veiligheidsstop duurt te lang. Er is dan al gewacht, dus een
        eventuele wachtstap aan het begin van de ladder slaan we over."""
        if self.ladder_index == 0 and isinstance(self._ladder[0], WaitStep):
            self.ladder_index = 1
        self.on_blocked()
