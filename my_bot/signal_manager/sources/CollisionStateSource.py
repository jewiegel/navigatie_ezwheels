#!/usr/bin/env python3

from signal_manager.RobotFacts import CollisionAction, RobotFacts
from signal_manager.sources.SignalSource import SignalSource


class CollisionStateSource(SignalSource):
    """navigation/blocked (std_msgs/Bool, transient_local) van de ziekenhuis-BT."""

    def __init__(self):
        self.collision = CollisionAction.NONE

    def update(self, msg, now):
        self.collision = CollisionAction.BLOCKED if msg.data else CollisionAction.NONE

    def is_fresh(self, now):
        return True     # transient_local en alleen bij een wissel: geen hartslag

    def apply(self, facts: RobotFacts):
        facts.collision = self.collision
