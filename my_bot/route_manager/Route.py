#!/usr/bin/env python3

from .Waypoint import Waypoint


class Route():

    def __init__(self, name: str, map_id: str, waypoints: list[Waypoint]):
        self.name = name
        self.map_id = map_id
        self.waypoints = waypoints
