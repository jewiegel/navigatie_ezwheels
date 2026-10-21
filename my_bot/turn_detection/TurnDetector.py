#!/usr/bin/env python3
"""Bochtherkenning zonder ROS: komende bocht uit een pad, huidige draai uit de hoeksnelheid."""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

# Zelfde waarden als in my_bot_interfaces/msg/Turn.msg
NONE, LEFT, RIGHT = 0, 1, 2

Point = Tuple[float, float]


@dataclass
class UpcomingTurn:
    direction: int    # LEFT of RIGHT
    distance: float   # m langs het pad tot de bocht begint
    angle: float      # rad, + = links


def find_turn(points: Sequence[Point], lookahead_m: float, turn_angle: float,
              step_m: float = 0.5) -> Optional[UpcomingTurn]:
    """Zoek de eerste bocht van minstens turn_angle (rad) binnen lookahead_m langs het pad.

    Het pad begint bij de robot. Het wordt eerst bemonsterd om de step_m meter: Nav2-paden
    liggen op het rooster van de costmap en trappen anders bij elke cel een paar graden.
    De richting van het eerste stuk is de referentie. De bocht begint waar de richting
    voor het eerst de helft van turn_angle afwijkt.

    shortcut: een bocht binnen de eerste step_m wordt niet gezien (het eerste stuk is de
    referentie); geef de robotrichting mee als dat nodig blijkt.
    """
    samples = _resample(points, step_m, lookahead_m + step_m)
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(samples, samples[1:])]
    if len(headings) < 2:
        return None

    start_index = None
    for i, heading in enumerate(headings):
        deviation = _wrap(heading - headings[0])
        if start_index is None and abs(deviation) >= turn_angle / 2:
            start_index = i
        if abs(deviation) >= turn_angle:
            # Een bocht die pas achter de lookahead begint, telt (nog) niet.
            distance = (start_index - 0.5) * step_m
            if distance > lookahead_m:
                return None
            # Totale draaiing: de grootste afwijking binnen het bekeken stuk.
            angle = max((_wrap(h - headings[0]) for h in headings[i:]),
                        key=abs, default=deviation)
            return UpcomingTurn(LEFT if angle > 0 else RIGHT, max(distance, 0.0), angle)
    return None


def extend_with_route(plan: Sequence[Point], route: Sequence[Point],
                      tolerance_m: float = 0.3) -> list:
    """Plak de route na het waypoint waar het pad eindigt erachter.

    Nav2 plant per waypoint, dus een bocht bij een waypoint staat nooit in het pad zelf.
    Eindigt het pad op een waypoint van de route, dan volgen de volgende waypoints.
    """
    if not plan:
        return list(plan)
    end = plan[-1]
    for i, wp in enumerate(route):
        if math.hypot(wp[0] - end[0], wp[1] - end[1]) <= tolerance_m:
            return list(plan) + list(route[i + 1:])
    return list(plan)


def turning_direction(angular_z: float, turn_rate: float) -> int:
    """LEFT/RIGHT als de robot sneller dan turn_rate (rad/s) draait, anders NONE."""
    if angular_z >= turn_rate:
        return LEFT
    if angular_z <= -turn_rate:
        return RIGHT
    return NONE


def _resample(points: Sequence[Point], step_m: float, max_length: float) -> list:
    """Punten om de step_m meter langs de lijn, tot max_length vanaf het begin."""
    if not points:
        return []
    samples = [points[0]]
    target = step_m      # afstand langs het pad van het volgende sample
    walked = 0.0         # afstand langs het pad tot het begin van het huidige segment
    for a, b in zip(points, points[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        while seg > 0 and walked + seg >= target and target <= max_length:
            t = (target - walked) / seg
            samples.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            target += step_m
        walked += seg
        if target > max_length:
            break
    return samples


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
