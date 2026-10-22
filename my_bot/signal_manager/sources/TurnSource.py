#!/usr/bin/env python3

from typing import Optional

from signal_manager.RobotFacts import RobotFacts, TurnDirection
from signal_manager.sources.SignalSource import SignalSource


class TurnSource(SignalSource):
    """turn (my_bot_interfaces/Turn, 5 Hz van de turn_detector)."""

    def __init__(self, timeout_s: float = 1.0):
        self.timeout_s = timeout_s
        self.turn = TurnDirection.NONE
        self._time: Optional[float] = None

    def update(self, msg, now):
        # Komende bocht gaat voor: een kleine stuurcorrectie de andere kant op mag het
        # aangekondigde knipperlicht niet omzetten. Zonder bocht: draaien (bv. op de plek).
        self.turn = TurnDirection(msg.direction or msg.turning)
        self._time = now

    def is_fresh(self, now):
        return self._time is not None and now - self._time <= self.timeout_s

    def apply(self, facts: RobotFacts):
        facts.turn = self.turn
