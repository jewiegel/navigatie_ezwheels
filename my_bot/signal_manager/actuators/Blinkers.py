#!/usr/bin/env python3

from enum import Enum
from typing import Optional

from signal_manager.RobotFacts import CollisionAction, RobotFacts, RobotMode, TurnDirection
from signal_manager.actuators.Actuator import Actuator


class BlinkerState(Enum):
    # Zelfde waarden als /indicators (mqtt_hmi_bridge zet ze om naar de lampen)
    OFF = 'uit'
    LEFT = 'links'
    RIGHT = 'rechts'
    HAZARD = 'gevaar'


SIDES = {TurnDirection.LEFT: BlinkerState.LEFT, TurnDirection.RIGHT: BlinkerState.RIGHT}
HAZARD_MODES = (RobotMode.WAITING_OPERATOR, RobotMode.ERROR)


class Blinkers(Actuator):
    """Alarmlichten > bocht > uit.

    Alarmlichten: route geblokkeerd, wachten op operator of fout, of tijdens het rijden een
    source die niet vers is. Een bocht brandt hold_s na: vlak voor en na het draaien ziet de
    turn_detector soms even geen bocht, en het licht mag dan niet flikkeren.
    """

    def __init__(self, hold_s: float = 1.0):
        self.hold_s = hold_s
        self.state = BlinkerState.OFF
        self._side = BlinkerState.OFF
        self._side_time: Optional[float] = None

    def update(self, facts: RobotFacts, now: float):
        if facts.turn != TurnDirection.NONE:
            self._side, self._side_time = SIDES[facts.turn], now
        elif self._side != BlinkerState.OFF and now - self._side_time >= self.hold_s:
            self._side = BlinkerState.OFF

        hazard = (facts.collision == CollisionAction.BLOCKED or facts.mode in HAZARD_MODES or
                  (facts.mode == RobotMode.DRIVING and not facts.safety_inputs_ok))
        self.state = BlinkerState.HAZARD if hazard else self._side
