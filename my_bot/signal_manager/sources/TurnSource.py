#!/usr/bin/env python3

from typing import Optional

from signal_manager.RobotFacts import RobotFacts, TurnDirection
from signal_manager.sources.SignalSource import SignalSource


class TurnSource(SignalSource):
    """turn (my_bot_interfaces/Turn, 5 Hz van de turn_detector)."""

    def __init__(self, timeout_s: float = 1.0, at_turn_m: float = 0.5,
                 turning_hold_s: float = 1.0):
        self.timeout_s = timeout_s
        self.at_turn_m = at_turn_m
        self.turning_hold_s = turning_hold_s
        self.turn = TurnDirection.NONE
        self._time: Optional[float] = None
        self._turning = TurnDirection.NONE      # laatste draairichting
        self._turning_time: Optional[float] = None

    def update(self, msg, now):
        # Op weg naar de bocht gaat de komende bocht voor: een kleine stuurcorrectie de
        # andere kant op mag het knipperlicht niet omzetten. Bij de bocht gaat het echte
        # draaien voor: op een waypoint draait de robot soms eerst de andere kant op (naar
        # de yaw van het waypoint), en dat is wat mensen zien.
        # Tijdens het draaien zakt de draaisnelheid soms even onder de drempel: onthoud de
        # draairichting turning_hold_s, anders springt het licht kort naar de andere kant.
        if msg.turning:
            self._turning, self._turning_time = msg.turning, now
        recent = self._turning_time is not None and now - self._turning_time < self.turning_hold_s
        turning = msg.turning or (self._turning if recent else TurnDirection.NONE)

        at_turn = msg.direction and msg.distance < self.at_turn_m
        first, second = (turning, msg.direction) if at_turn else (msg.direction, turning)
        self.turn = TurnDirection(first or second)
        self._time = now

    def is_fresh(self, now):
        return self._time is not None and now - self._time <= self.timeout_s

    def apply(self, facts: RobotFacts):
        facts.turn = self.turn
