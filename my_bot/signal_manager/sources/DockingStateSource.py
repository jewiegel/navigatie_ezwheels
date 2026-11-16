#!/usr/bin/env python3

from signal_manager.RobotFacts import RobotFacts
from signal_manager.sources.SignalSource import SignalSource


class DockingStateSource(SignalSource):
    """docking/active (std_msgs/Bool, transient_local) van dock_cart: robot koppelt aan de kar."""

    def __init__(self):
        self.docking = False

    def update(self, msg, now):
        self.docking = bool(msg.data)

    def is_fresh(self, now):
        return True     # transient_local en alleen bij een wissel: geen hartslag

    def apply(self, facts: RobotFacts):
        facts.docking = self.docking
