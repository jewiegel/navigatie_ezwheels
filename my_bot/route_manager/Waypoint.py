#!/usr/bin/env python3

from .ActionSpecification import ActionSpecification


class Waypoint():

    def __init__(self, name: str, x: float, y: float, yaw: float,
                 actions: list[ActionSpecification] = None):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.actions = actions or []
