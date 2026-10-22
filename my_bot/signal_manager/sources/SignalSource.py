#!/usr/bin/env python3

from abc import ABC, abstractmethod

from signal_manager.RobotFacts import RobotFacts


class SignalSource(ABC):
    """Eén invoer: verwerkt zijn ROS-bericht en schrijft wat hij weet in RobotFacts."""

    @abstractmethod
    def update(self, msg, now: float) -> None:
        pass

    @abstractmethod
    def is_fresh(self, now: float) -> bool:
        pass

    @abstractmethod
    def apply(self, facts: RobotFacts) -> None:
        pass
