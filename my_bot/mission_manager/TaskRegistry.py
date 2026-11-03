#!/usr/bin/env python3

from rclpy.action import ActionClient


class TaskRegistry():
    """Zet de naam van een waypoint-actie om in de ActionClient van de bijbehorende
    action server en een functie die uit de params van de actie het doel maakt.

        registry.register('wait', Wait, 'wait', lambda p: Wait.Goal(...))
        client, make_goal = registry.get('wait')
    """

    def __init__(self, node, callback_group=None):
        self._node = node
        self._callback_group = callback_group
        self._tasks = {}   # naam -> (action-type, servernaam, make_goal)
        self._clients = {}

    def register(self, name: str, action_type, server: str, make_goal) -> None:
        self._tasks[name] = (action_type, server, make_goal)

    def names(self) -> list[str]:
        return sorted(self._tasks)

    def get(self, name: str):
        """Geeft (ActionClient, make_goal); KeyError bij een onbekende naam."""
        action_type, server, make_goal = self._tasks[name]
        if name not in self._clients:   # één client per actie, pas bij het eerste gebruik
            self._clients[name] = ActionClient(
                self._node, action_type, server, callback_group=self._callback_group)
        return self._clients[name], make_goal
