#!/usr/bin/env python3

import math
import re
from typing import Optional

from .Route import Route
from .Waypoint import Waypoint

ROUTE_NAME = re.compile(r'^[A-Za-z0-9_-]+$')


class RouteDraft():
    """Route in opbouw: waypoints één voor één toevoegen en dan als Route opslaan."""

    def __init__(self):
        self.waypoints: list[Waypoint] = []

    def add(self, x: float, y: float, yaw: float) -> Waypoint:
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('x, y en yaw moeten eindige getallen zijn')
        waypoint = Waypoint(f'wp{len(self.waypoints) + 1}', x, y, yaw)
        self.waypoints.append(waypoint)
        return waypoint

    def undo(self) -> Optional[Waypoint]:
        return self.waypoints.pop() if self.waypoints else None

    def clear(self) -> None:
        self.waypoints.clear()

    def build(self, name: str, map_id: str) -> Route:
        """Maak er een Route van; waypoints heten <naam>_1, <naam>_2, ..."""
        if not ROUTE_NAME.match(name or ''):
            raise ValueError(f'Ongeldige routenaam "{name}": gebruik letters, cijfers, _ of -')
        if not self.waypoints:
            raise ValueError('Concept-route heeft geen waypoints')
        waypoints = [
            Waypoint(f'{name}_{i}', w.x, w.y, w.yaw) for i, w in enumerate(self.waypoints, 1)]
        return Route(name, map_id, waypoints)
