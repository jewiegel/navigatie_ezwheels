#!/usr/bin/env python3

from enum import Enum

class MissionState(Enum):
    IDLE = 0
    DRIVING = 1
    BLOCKED = 2
    WAITING_OPERATOR = 3
    ARRIVED = 4
    ERROR = -1