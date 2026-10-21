#!/usr/bin/env python3

from route_manager import Waypoint
from .Command import Command


class SendGoalCommand(Command):

    def __init__(self, waypoints: list[Waypoint]):
        self.waypoints = waypoints

    def execute(self, ports):
        ports.nav.send_goal(self.waypoints)
