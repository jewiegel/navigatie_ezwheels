#!/usr/bin/env python3
"""Rekenwerk voor het docken onder de kar, zonder ROS.

Poses zijn (x, y, theta) in een plat frame; punten zijn (x, y). Het kar-frame heeft
de oorsprong midden onder de kar en de x-as in de rijrichting van de gedockte robot.
De lidar ziet van de kar alleen de vier poten (dunne cilinders); hun onderlinge
afstanden liggen vast, dus uit twee of meer gevonden poten volgt de pose van de kar.
"""
import itertools
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


def blend(old, new, alpha: float):
    """Nieuwe meting voor een fractie alpha laten meetellen (hoek zonder sprong bij +-pi)."""
    return (old[0] + alpha * (new[0] - old[0]), old[1] + alpha * (new[1] - old[1]),
            wrap(old[2] + alpha * wrap(new[2] - old[2])))


def cart_legs(leg_x: float, leg_y: float):
    """De vier poten in het kar-frame: rechthoek van 2*leg_x lang en 2*leg_y breed."""
    return [(leg_x, leg_y), (leg_x, -leg_y), (-leg_x, leg_y), (-leg_x, -leg_y)]


def find_leg_candidates(points, sensor, leg_r: float, max_gap: float = 0.04,
                        max_width: float = 0.08, min_points: int = 3):
    """Middelpunten van smalle objecten (mogelijke poten) in een scan.

    points: scanpunten in volgorde van de hoek, sensor: positie van de lidar, beide in
    hetzelfde frame. Een cluster stopt bij een sprong > max_gap. Het middelpunt volgt uit
    een cirkelfit met bekende straal leg_r (de punten liggen alleen op de zichtkant).
    """
    clusters, current = [], []
    for p in points:
        if current and math.dist(current[-1], p) > max_gap:
            clusters.append(current)
            current = []
        current.append(p)
    if current:
        clusters.append(current)

    centers = []
    for c in clusters:
        if len(c) < min_points or math.dist(c[0], c[-1]) > max_width:
            continue
        centers.append(_circle_center(c, sensor, leg_r))
    return centers


def _circle_center(points, sensor, r, iterations=10):
    """Middelpunt van een cirkel met straal r door de punten: elk punt wijst r verder
    (van zichzelf af, richting het huidige middelpunt) naar het middelpunt; herhalen."""
    cx, cy = sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points)
    d = math.dist((cx, cy), sensor)
    k = math.pi / 4 * r / d       # start: zwaartepunt van de zichtkant ligt pi/4 * r voor
    cx, cy = cx + k * (cx - sensor[0]), cy + k * (cy - sensor[1])
    for _ in range(iterations):
        sx = sy = 0.0
        for px, py in points:
            dist = math.hypot(cx - px, cy - py) or 1e-9
            sx += px + r * (cx - px) / dist
            sy += py + r * (cy - py) / dist
        cx, cy = sx / len(points), sy / len(points)
    return cx, cy


def fit_pose(model, measured):
    """Pose die de modelpunten (kar-frame) zo goed mogelijk op de metingen legt (kleinste
    kwadraten, 2D). Geeft (pose, rms-fout in m)."""
    n = len(model)
    mx, my = sum(p[0] for p in model) / n, sum(p[1] for p in model) / n
    dx, dy = sum(p[0] for p in measured) / n, sum(p[1] for p in measured) / n
    sxx = sxy = 0.0
    for (ax, ay), (bx, by) in zip(model, measured):
        ax, ay, bx, by = ax - mx, ay - my, bx - dx, by - dy
        sxx += ax * bx + ay * by
        sxy += ax * by - ay * bx
    t = math.atan2(sxy, sxx)
    c, s = math.cos(t), math.sin(t)
    pose = (dx - (c * mx - s * my), dy - (s * mx + c * my), t)
    rms = math.sqrt(sum(math.dist(transform(pose, a), b) ** 2
                        for a, b in zip(model, measured)) / n)
    return pose, rms


def match_cart(candidates, prior, legs, gate: float, max_rms: float = 0.02):
    """Zoek de kar rond de verwachte pose prior. Elke poot mag een kandidaat binnen gate
    van zijn verwachte plek krijgen; de toewijzing met de meeste poten (minstens 2) en
    dan de kleinste fout wint. Geeft (pose, aantal poten, rms) of None."""
    expected = [transform(prior, leg) for leg in legs]
    options = [[j for j, c in enumerate(candidates) if math.dist(c, e) <= gate] + [None]
               for e in expected]
    best = None
    for choice in itertools.product(*options):
        used = [j for j in choice if j is not None]
        if len(used) < 2 or len(set(used)) != len(used):
            continue
        model = [legs[i] for i, j in enumerate(choice) if j is not None]
        pose, rms = fit_pose(model, [candidates[j] for j in used])
        if rms > max_rms:
            continue
        key = (-len(used), rms)
        if best is None or key < best[0]:
            best = (key, (pose, len(used), rms))
    return best[1] if best else None


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


def point_cmd(robot, point, tol: float = 0.005):
    """(v, w) om de as van de robot recht naar point te rijden; None = klaar.
    Eerst draaien tot het punt voor de robot ligt, dan rijden met koerscorrectie."""
    dx, dy = point[0] - robot[0], point[1] - robot[1]
    dist = math.hypot(dx, dy)
    bearing = wrap(math.atan2(dy, dx) - robot[2])
    ahead = dist * math.cos(bearing)
    if dist <= tol or (dist < 0.05 and ahead <= tol):
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
