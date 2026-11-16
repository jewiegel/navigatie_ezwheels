#!/usr/bin/env python3

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable


@dataclass
class TaskResult:
    success: bool
    message: str = ''


class WaypointTaskServerBase(ABC):
    """Eén waypoint-actie. WaypointTaskNode biedt hem aan als action server met de naam
    task_name (my_bot_interfaces/action/WaypointTask); de mission_manager roept hem aan
    via de TaskRegistry."""

    def __init__(self, task_name: str):
        self.task_name = task_name

    @abstractmethod
    def execute(self, params: dict, feedback: Callable[[str], None],
                cancelled: Callable[[], bool]) -> TaskResult:
        """Blokkeert tot de actie klaar is. feedback(tekst) meldt de huidige stap;
        cancelled() wordt True als de actie afgebroken moet worden."""
