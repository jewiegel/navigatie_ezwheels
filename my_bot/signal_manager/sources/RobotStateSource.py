#!/usr/bin/env python3

from signal_manager.RobotFacts import RobotFacts, RobotMode
from signal_manager.sources.SignalSource import SignalSource


class RobotStateSource(SignalSource):
    """mission_status (std_msgs/String, bv. "DRIVING gang_b") van de MissionManager."""

    def __init__(self):
        self.mode = RobotMode.IDLE

    def update(self, msg, now):
        name = msg.data.split(' ', 1)[0]
        if name in RobotMode.__members__:
            self.mode = RobotMode[name]

    def is_fresh(self, now):
        return True     # komt alleen bij een wissel: geen hartslag om op te controleren

    def apply(self, facts: RobotFacts):
        facts.mode = self.mode
