#!/usr/bin/env python3

from abc import ABC, abstractmethod

from route_manager import Waypoint


class NavigationClient(ABC):

    @abstractmethod
    def send_goal(self, waypoints: list[Waypoint]) -> None:
        pass

    @abstractmethod
    def cancel(self) -> None:
        pass
