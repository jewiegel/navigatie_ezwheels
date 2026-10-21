#!/usr/bin/env python3

from typing import Optional

from route_manager import Route
from .enums import MissionKind


class MissionEvent():

    def __init__(self, kind: MissionKind, route_id: str = '', route: Optional[Route] = None):
        self.kind = kind
        self.route_id = route_id
        self.route = route   # alleen bij START_ROUTE
