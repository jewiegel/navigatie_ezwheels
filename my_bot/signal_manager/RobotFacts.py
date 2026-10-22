#!/usr/bin/env python3
"""Wat de signal_manager over de robot weet, samengesteld uit de sources."""

from dataclasses import dataclass
from enum import Enum, IntEnum

from mission_manager.enums.MissionState import MissionState as RobotMode


class TurnDirection(IntEnum):
    # Zelfde waarden als my_bot_interfaces/msg/Turn.msg
    NONE = 0
    LEFT = 1
    RIGHT = 2


class CollisionAction(Enum):
    NONE = 0
    BLOCKED = 1     # route geblokkeerd (navigation/blocked van de ziekenhuis-BT)


@dataclass
class RobotFacts:
    mode: RobotMode = RobotMode.IDLE
    collision: CollisionAction = CollisionAction.NONE
    turn: TurnDirection = TurnDirection.NONE
    safety_inputs_ok: bool = True    # alle sources vers


__all__ = ['RobotFacts', 'RobotMode', 'CollisionAction', 'TurnDirection']
