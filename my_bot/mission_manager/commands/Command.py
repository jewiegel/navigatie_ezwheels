#!/usr/bin/env python3

from abc import ABC, abstractmethod

from ..ports import MissionPorts


class Command(ABC):

    @abstractmethod
    def execute(self, ports: MissionPorts) -> None:
        pass
