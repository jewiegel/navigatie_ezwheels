#!/usr/bin/env python3

import Waypoint

class Route():

    def __init__(self, name, map_id, waypoints: list[Waypoint]):
        self.name = name
        self.map_id = map_id
        self.waypoints = waypoints
