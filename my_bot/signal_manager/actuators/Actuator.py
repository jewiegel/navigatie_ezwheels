#!/usr/bin/env python3

from abc import ABC, abstractmethod

from signal_manager.RobotFacts import RobotFacts


class Actuator(ABC):
    """Bepaalt uit RobotFacts wat een uitgang moet doen; het versturen doet de node."""

    @abstractmethod
    def update(self, facts: RobotFacts, now: float) -> None:
        pass
