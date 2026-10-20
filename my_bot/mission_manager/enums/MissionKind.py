#!/usr/bin/env python3

from enum import Enum

class MissionKind(Enum):
    START_ROUTE = 1
    NAV_SUCCEEDED = 2
    NAV_ABORTED = 3
    BT_BLOCKED = 4
    BT_DRIVING = 5
    RESET = 6