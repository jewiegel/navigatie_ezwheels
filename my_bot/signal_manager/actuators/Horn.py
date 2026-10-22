#!/usr/bin/env python3

from dataclasses import replace
from enum import Enum
from typing import Optional

from signal_manager.RobotFacts import CollisionAction, RobotFacts, RobotMode
from signal_manager.actuators.Actuator import Actuator


class HornPattern(Enum):
    """Wat de buzzer moet doen; de ontvanger speelt het patroon zelf af.

    Eenmalig (enkel, dubbel) speelt één keer. Herhalend (elke_5s) loopt door tot er een
    ander patroon komt; uit stopt het.
    """
    OFF = 'uit'
    SINGLE = 'enkel'            # één piep
    DOUBLE = 'dubbel'           # twee piepjes
    EVERY_5S = 'elke_5s'        # elke 5 s een piep


START_FROM = (RobotMode.IDLE, RobotMode.ARRIVED)   # DRIVING na BLOCKED is geen vertrek
ATTENTION_MODES = (RobotMode.WAITING_OPERATOR, RobotMode.ERROR)


class Horn(Actuator):
    """Herhalend: elke_5s zolang de route geblokkeerd is. Eenmalig, alleen als er niets
    herhaalt: enkel bij vertrek en aankomst, dubbel bij wachten op operator of fout.

    pending is het patroon dat verstuurd moet worden (None: niets); de node verstuurt het
    en zet het op None.
    """

    def __init__(self):
        self.pending: Optional[HornPattern] = None
        self.previous = RobotFacts()
        self._repeating = HornPattern.OFF

    def update(self, facts: RobotFacts, now: float):
        prev = self.previous
        self.previous = replace(facts)

        repeating = (HornPattern.EVERY_5S if facts.collision == CollisionAction.BLOCKED
                     else HornPattern.OFF)
        if repeating != self._repeating:
            self._repeating = self.pending = repeating
            return
        if repeating != HornPattern.OFF or facts.mode == prev.mode:
            return
        if facts.mode in ATTENTION_MODES:
            self.pending = HornPattern.DOUBLE
        elif facts.mode == RobotMode.ARRIVED or (
                facts.mode == RobotMode.DRIVING and prev.mode in START_FROM):
            self.pending = HornPattern.SINGLE
