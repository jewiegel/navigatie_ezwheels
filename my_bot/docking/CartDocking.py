#!/usr/bin/env python3
"""Rekenwerk voor het docken onder de kar, zonder ROS: poses, de rijbaancontrole en de
regelaars. De kar zelf meten: CartRoi.

Poses zijn (x, y, theta) in een plat frame; punten zijn (x, y). Het kar-frame heeft
de oorsprong midden onder de kar en de x-as in de rijrichting van de gedockte robot.
"""
import math

# Regelaar: snelheden in m/s en rad/s, versterkingen per meter of radiaal
V_MAX, V_MIN = 0.12, 0.02
W_MAX, W_MIN = 0.5, 0.04
K_ROT = 1.5          # draaien op de plek
K_LAT = 3.0          # gewenste koershoek per meter zijwaartse fout
K_HEAD = 2.0         # koersfout -> draaisnelheid
MAX_COURSE = 0.3     # rad: grootste koershoek om naar de as terug te sturen


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def transform(pose, p):
    """Punt p uit het frame van pose naar het frame waarin pose staat."""
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return (x + c * p[0] - s * p[1], y + s * p[0] + c * p[1])


def compose(a, b):
    x, y = transform(a, b[:2])
    return (x, y, wrap(a[2] + b[2]))


def inverse(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return (-c * p[0] - s * p[1], s * p[0] - c * p[1], -p[2])


def points_in_lane(points, cart, half_x: float, half_lane: float):
    """Scanpunten in de rijbaan van de robot onder de kar: kar-frame |x| <= half_x en
    |y| < half_lane. Een naar binnen gedraaid wiel, zijn beugel of iets onder de kar."""
    inv = inverse(cart)
    return [p for p in points
            if abs(transform(inv, p)[0]) <= half_x and abs(transform(inv, p)[1]) < half_lane]


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _with_min(v, minimum, maximum):
    """Begrens op maximum, maar niet langzamer dan minimum (anders kruipt de robot)."""
    return math.copysign(_clamp(abs(v), minimum, maximum), v)


def rotate_cmd(heading_error: float, tol: float = math.radians(0.3)):
    """Draaisnelheid om heading_error (huidig - gewenst) weg te draaien; None = klaar."""
    if abs(heading_error) <= tol:
        return None
    return _with_min(-K_ROT * heading_error, W_MIN, W_MAX)


def point_cmd(robot, point, tol: float = 0.005, lateral_tol: float = 0.015):
    """(v, w) om de as van de robot recht naar point te rijden; None = klaar.
    Eerst draaien tot het punt voor de robot ligt, dan rijden met koerscorrectie. Net
    voorbij het punt is ook goed, mits het hooguit lateral_tol opzij ligt: anders zou hij
    voor een paar millimeter omkeren."""
    dx, dy = point[0] - robot[0], point[1] - robot[1]
    dist = math.hypot(dx, dy)
    bearing = wrap(math.atan2(dy, dx) - robot[2])
    ahead = dist * math.cos(bearing)
    lateral = abs(dist * math.sin(bearing))
    if dist <= tol or (dist < 0.05 and ahead <= tol and lateral <= lateral_tol):
        return None
    if abs(bearing) > 0.3:
        return 0.0, _with_min(K_ROT * bearing, W_MIN, W_MAX)
    w = 0.0 if dist < 0.03 else _clamp(K_ROT * bearing, -W_MAX, W_MAX)
    return _with_min(0.6 * ahead, V_MIN, V_MAX), w


def axis_cmd(rel, stop_x: float, v_max: float = 0.08, tol: float = 0.003):
    """(v, w) om langs de x-as van een frame naar x = stop_x te rijden; None = klaar.

    rel: pose van de robotas (midden tussen de wielen) in dat frame. Rijdt vooruit of
    achteruit; stuurt via een koershoek terug naar y = 0 (stabiel voor een diff-drive).
    """
    dist = stop_x - rel[0]
    if abs(dist) <= tol:
        return None
    direction = 1.0 if dist > 0 else -1.0
    course = _clamp(-K_LAT * rel[1] * direction, -MAX_COURSE, MAX_COURSE)
    w = _clamp(-K_HEAD * wrap(rel[2] - course), -W_MAX, W_MAX)
    return direction * _clamp(0.5 * abs(dist), V_MIN, v_max), w
