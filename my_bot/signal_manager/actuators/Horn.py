#!/usr/bin/env python3

from dataclasses import replace
from enum import Enum

from signal_manager.RobotFacts import CollisionAction, RobotFacts, RobotMode
from signal_manager.actuators.Actuator import Actuator


class HornPattern(Enum):
    """Afwisselend aan/uit in seconden, beginnend met aan."""
    NONE = ()
    SHORT = (0.3,)
    DOUBLE = (0.15, 0.15, 0.15)


START_FROM = (RobotMode.IDLE, RobotMode.ARRIVED)   # DRIVING na BLOCKED is geen vertrek
ATTENTION_MODES = (RobotMode.WAITING_OPERATOR, RobotMode.ERROR)


class Horn(Actuator):
    """Piept bij een verandering: vertrek, aankomst en begin blokkade kort, wachten op
    operator of fout dubbel. pending wordt door de node afgespeeld en op NONE gezet."""

    def __init__(self):
        self.pending = HornPattern.NONE
        self.previous = RobotFacts()

    def update(self, facts: RobotFacts, now: float):
        prev = self.previous
        if facts.mode != prev.mode:
            if facts.mode in ATTENTION_MODES:
                self.pending = HornPattern.DOUBLE
            elif facts.mode == RobotMode.ARRIVED or (
                    facts.mode == RobotMode.DRIVING and prev.mode in START_FROM):
                self.pending = HornPattern.SHORT
        if (facts.collision == CollisionAction.BLOCKED and
                prev.collision != CollisionAction.BLOCKED):
            self.pending = HornPattern.SHORT
        self.previous = replace(facts)
