"""Instellingen van de patrouille: tijden, afstanden en routes.

Alle waarden hier zijn de standaardwaarden. Op de robot worden ze overschreven
door ROS-parameters (config/patrol_params.yaml) en het routebestand
(config/patrol_routes.yaml) — zie patrol/ros/ros_config.py.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import List, Tuple

Waypoint = Tuple[float, float, float]   # x, y, yaw (rad) in het map-frame


@dataclass
class Route:
    route_id: int
    start_topic: str          # Bool-topic dat deze route start (bv. HMI-knop)
    waypoints: List[Waypoint]


DEFAULT_ROUTES = [
    Route(1, '/start_patrol', [
        (3.1859422942077407, 0.21455550145977087, 0.0),
        (0.346053203926163, 0.1555398774080503, 0.0)
    ]),
    Route(2, '/start_patrol2', [
        (7.0, 2.9, 0.0),
        (5.48, -0.6, 0.0),
        (7.66, -3.49, 0.0),
        (7.99, 3.32, 0.0),
    ]),
]


@dataclass
class PatrolConfig:
    # Blokkade-afhandeling
    wait_seconds: float = 5.0            # wachten op vrije doorgang bij blokkade
    backup_speed: float = 0.1              # m/s achteruit
    backup_distance: float = 0.5           # m achteruit na de wachttijd
    renav_pause: float = 2.0               # s pauze tussen oud doel annuleren en nieuw doel

    # Navigatie
    nav_timeout_sec: float = 60.0          # per waypoint zonder voortgang → blokkade
    return_timeout_sec: float = 120.0      # terugkeren naar vorig waypoint → anders fout
    server_timeout_sec: float = 5.0        # wachten op Nav2 action server
    goal_retry_delay: float = 4.0          # s tot nieuwe poging na geweigerd doel
    max_goal_retries: int = 1

    # Opstarten (zombie-doelen annuleren + controller_server resetten)
    controller_reset_delay: float = 8.0    # s na controller-reset tot vertrek
    start_delay: float = 1.0               # s tot vertrek als reset niet mogelijk was
    planner_id: str = 'GridBased'

    # EZ-Wheel veiligheidsstop-detectie (stilstand tijdens rijden = persoon/object)
    safety_monitor_enabled: bool = False   # tijdelijk uit voor debugging
    safety_still_seconds: float = 15.0
    person_wait_seconds: float = 120.0

    cmd_vel_topic: str = '/cmd_vel'

    routes: List[Route] = field(default_factory=lambda: copy.deepcopy(DEFAULT_ROUTES))

    @property
    def backup_duration(self) -> float:
        return self.backup_distance / self.backup_speed


def routes_from_dict(data: dict) -> List[Route]:
    """Zet de inhoud van patrol_routes.yaml om naar Route-objecten.

    Formaat:
        routes:
          - id: 1
            start_topic: /start_patrol
            waypoints:
              - [7.66, -3.49, 0.0]    # x, y, yaw
    """
    routes = []
    for entry in data.get('routes', []):
        waypoints = []
        for wp in entry['waypoints']:
            if len(wp) != 3:
                raise ValueError(
                    f'Route {entry["id"]}: waypoint {wp} moet [x, y, yaw] zijn')
            waypoints.append((float(wp[0]), float(wp[1]), float(wp[2])))
        if not waypoints:
            raise ValueError(f'Route {entry["id"]} heeft geen waypoints')
        routes.append(Route(int(entry['id']), str(entry['start_topic']), waypoints))
    if not routes:
        raise ValueError('Geen routes gevonden')
    return routes


def load_routes(path: str) -> List[Route]:
    import yaml
    with open(path) as f:
        return routes_from_dict(yaml.safe_load(f) or {})
