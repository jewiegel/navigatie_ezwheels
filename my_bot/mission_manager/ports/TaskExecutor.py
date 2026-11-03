#!/usr/bin/env python3

from abc import ABC, abstractmethod

from route_manager import ActionSpecification


class TaskExecutor(ABC):
    """Voert één waypoint-actie uit en meldt daarna TASK_SUCCEEDED of TASK_FAILED."""

    @abstractmethod
    def execute(self, task: ActionSpecification) -> None:
        pass

    @abstractmethod
    def cancel(self) -> None:
        pass
