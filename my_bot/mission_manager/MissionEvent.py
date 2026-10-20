#!/usr/bin/env python3

from enums import MissionKind

class MissionEvent():

    def __init__(self):
        self.kind: MissionKind
        self.route_id: str