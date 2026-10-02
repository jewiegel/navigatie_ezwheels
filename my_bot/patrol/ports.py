"""Ports: de interfaces waarmee de patrouille-logica met de buitenwereld praat.

De controller en de toestanden kennen alleen deze interfaces, nooit rclpy of
Nav2 zelf. Op de robot worden ze ingevuld door de adapters in patrol/ros/;
in de unittests door simpele fakes. Daardoor is de logica zonder ROS te testen.
"""
from __future__ import annotations

from typing import Callable, Optional, Protocol, Sequence

from .config import Waypoint


class TimerHandle(Protocol):
    def cancel(self) -> None: ...


class Scheduler(Protocol):
    def now(self) -> float:
        """Huidige tijd in seconden."""

    def call_later(self, delay: float, fn: Callable[[], None]) -> TimerHandle:
        """Roep fn één keer aan na `delay` seconden."""

    def call_every(self, period: float, fn: Callable[[], None]) -> TimerHandle:
        """Roep fn elke `period` seconden aan tot cancel()."""


class Navigator(Protocol):
    """Nav2-adapter. Callbacks van een doel komen alleen binnen zolang dat doel
    het actuele is: na cancel() of een nieuw doel worden ze stil genegeerd."""

    def select_planner(self, planner_id: str) -> None: ...

    def prepare(self, on_ready: Callable[[bool], None]) -> None:
        """Annuleer zombie-doelen en reset de controller_server.
        on_ready(controller_reset) — True als de reset is aangevraagd."""

    def drive_through(self, waypoints: Sequence[Waypoint],
                      on_accepted: Callable[[], None],
                      on_rejected: Callable[[], None],
                      on_feedback: Callable[[int], None],
                      on_result: Callable[[bool, int], None]) -> bool:
        """NavigateThroughPoses. on_feedback(poses_remaining),
        on_result(succeeded, status). False als de server niet bereikbaar is."""

    def drive_to(self, waypoint: Waypoint,
                 on_rejected: Callable[[], None],
                 on_result: Callable[[bool, int], None]) -> bool:
        """NavigateToPose. False als de server niet bereikbaar is."""

    def cancel(self) -> None:
        """Annuleer het actieve doel en negeer alle late callbacks ervan."""


class Signals(Protocol):
    """Alles wat de robot naar buiten laat zien of doet buiten Nav2 om."""

    def stop_wheels(self) -> None: ...

    def drive(self, linear_x: float) -> None: ...

    def beep(self) -> None: ...

    def set_indicator(self, mode: str) -> None:
        """'links', 'rechts', 'gevaar' of 'uit'."""

    def publish_state(self, name: str) -> None: ...


class Logger(Protocol):
    def info(self, msg: str) -> None: ...

    def warn(self, msg: str) -> None: ...

    def error(self, msg: str) -> None: ...


class StateListener(Protocol):
    """Observer: krijgt elke toestandswissel. old is new bij een herpublicatie
    (bv. veiligheidsstop aan/uit zonder echte wissel)."""

    def on_state_changed(self, old: Optional[object], new: object) -> None: ...
