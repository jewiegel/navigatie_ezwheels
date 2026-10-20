#!/usr/bin/env python3

import my_bot.mission_manager.enums.MissionState as MissionState
from route_manager import Route

class MissionStateMachine():

    def __init__(self):
        self.state: MissionState = MissionState.IDLE
        self.route: Route
        self.Transitions: dict

    def handle(event: MissionEvent):
        pass
