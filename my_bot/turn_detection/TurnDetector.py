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
    point: Point      # waar de bocht begint, in het frame van het pad


def find_turn(points: Sequence[Point], lookahead_m: float, turn_angle: float,
              step_m: float = 0.5, heading: Optional[float] = None) -> Optional[UpcomingTurn]:
    """Zoek de eerste bocht van minstens turn_angle (rad) binnen lookahead_m langs het pad.

    Het pad begint bij de robot. Het wordt eerst bemonsterd om de step_m meter: Nav2-paden
    liggen op het rooster van de costmap en trappen anders bij elke cel een paar graden.
    De referentie is heading (rijrichting van de robot, rad, frame van het pad), of anders
    de richting van het eerste stuk. De bocht begint waar de richting voor het eerst de
    helft van turn_angle afwijkt. Zonder heading wordt een bocht binnen de eerste step_m
    niet gezien: vlak voor een waypoint loopt dat eerste stuk de bocht al in.
    """
    samples = _resample(points, step_m, lookahead_m + step_m)
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(samples, samples[1:])]
    if len(headings) < 2:
        return None
    reference = headings[0] if heading is None else heading

    start_index = None
    for i, h in enumerate(headings):
        deviation = _wrap(h - reference)
        if start_index is None and abs(deviation) >= turn_angle / 2:
            start_index = i
        if abs(deviation) >= turn_angle:
            # Een bocht die pas achter de lookahead begint, telt (nog) niet.
            distance = (start_index - 0.5) * step_m
            if distance > lookahead_m:
                return None
            # Totale draaiing: de grootste afwijking binnen het bekeken stuk.
            angle = max((_wrap(h - reference) for h in headings[i:]),
                        key=abs, default=deviation)
            return UpcomingTurn(LEFT if angle > 0 else RIGHT, max(distance, 0.0), angle,
                                samples[start_index])
    return None


def extend_with_route(plan: Sequence[Point], waypoints: Sequence[Point],
                      tolerance_m: float = 0.3) -> list:
    """Plak de waypoints na het huidige doel achter het pad.

    Nav2 plant per waypoint, dus een bocht bij een waypoint staat nooit in het pad zelf.
    waypoints zijn de resterende waypoints, huidig doel eerst. Alleen als het pad op dat
    doel eindigt: een oude lijst (route geannuleerd, nu een los doel) telt niet.
    """
    if not plan or not waypoints:
        return list(plan)
    goal, end = waypoints[0], plan[-1]
    if math.hypot(goal[0] - end[0], goal[1] - end[1]) > tolerance_m:
        return list(plan)
    return list(plan) + list(waypoints[1:])


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
