#!/usr/bin/env python3
"""Rekenwerk voor het docken onder de kar, zonder ROS.

Poses zijn (x, y, theta) in een plat frame; punten zijn (x, y). Het kar-frame heeft
de oorsprong midden onder de kar en de x-as in de rijrichting van de gedockte robot.
De lidar zit op wielhoogte en ziet van de kar alleen de vier zwenkwielen, als strepen.
Per streep volgt uit de bekende wielmaat het midden en de richting van het wiel. De
draai-as van een zwenkwiel ligt `trail` voor of achter het wielmidden (in de rijrichting
van het wiel); de draai-assen liggen vast op een rechthoek, dus uit twee of meer wielen
volgt de pose van de kar.
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
ANGLE_TOL = math.radians(2.5)   # minimaal; ruim, uiteinden van echte wielstrepen zijn rafelig
MISSING_COST = 0.01  # m: gewicht van een niet gebruikt wiel bij het kiezen van de kar
FLIP_MARGIN = 0.025  # m: gespiegeld zo weinig slechter = niet te onderscheiden (gemeten:
                     # uitgelijnde wielen <= 18 mm, verschillend gedraaide >= 33 mm)


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


def cart_castors(castor_x: float, castor_y: float):
    """De draai-assen van de vier zwenkwielen in het kar-frame: rechthoek van 2*castor_x
    lang en 2*castor_y breed."""
    return [(castor_x, castor_y), (castor_x, -castor_y), (-castor_x, castor_y),
            (-castor_x, -castor_y)]


def points_in_area(points, prior, half_x: float, half_y: float):
    """Alleen de punten binnen de rechthoek van 2*half_x bij 2*half_y rond prior (kar-frame)."""
    inv = inverse(prior)
    kept = []
    for p in points:
        x, y = transform(inv, p)
        if abs(x) <= half_x and abs(y) <= half_y:
            kept.append(p)
    return kept


def find_wheels(points, sensor, length: float, width: float, max_gap: float = 0.04,
                min_points: int = 3, max_rms: float = 0.015, angle_tol: float = ANGLE_TOL):
    """Wielen in een scan: [(midden, richting in rad)].

    points: scanpunten in volgorde van de hoek, sensor: positie van de lidar, beide in
    hetzelfde frame. Een cluster stopt bij een sprong > max_gap; clusters die niet als
    rechthoek van length x width (het wiel op lidarhoogte) passen, vallen af. Ligt het
    buurpunt van een cluster dichterbij, dan staat er iets voor die kant van het wiel; aan
    het begin of eind van points kan het wiel buiten het blikveld doorlopen.
    """
    clusters, start = [], 0
    for i in range(1, len(points) + 1):
        if i == len(points) or math.dist(points[i - 1], points[i]) > max_gap:
            clusters.append((start, i))
            start = i

    def hidden(edge, neighbour):
        if not 0 <= neighbour < len(points):
            return True    # rand van de scan: het wiel kan buiten het blikveld doorlopen
        a, b = points[edge], points[neighbour]
        gap = abs(wrap(math.atan2(a[1] - sensor[1], a[0] - sensor[0]) -
                       math.atan2(b[1] - sensor[1], b[0] - sensor[0])))
        return gap < angle_tol and math.dist(b, sensor) < math.dist(a, sensor)

    wheels = []
    for i0, i1 in clusters:
        c = points[i0:i1]
        extent = math.dist(c[0], c[-1])
        # Een wiel is van elke kant minstens zo breed als zijn breedte: dunnere strepen
        # (stoel- en tafelpoten) en langere (muren) zijn geen wiel.
        if len(c) < min_points or not 0.75 * width <= extent <= math.hypot(length, width) + 0.03:
            continue
        center, angle, rms = fit_wheel(c, sensor, length, width,
                                       (hidden(i0, i0 - 1), hidden(i1 - 1, i1)), angle_tol)
        if rms <= max_rms:
            wheels.append((center, angle))
    return wheels


def fit_wheel(points, sensor, length: float, width: float, hidden=(False, False),
              angle_tol: float = ANGLE_TOL):
    """Rechthoek van length x width die het best op de punten past: (midden, richting, rms).

    De lidar ziet alleen de kant(en) naar zich toe. Per richting volgt het midden uit de
    zichtbare zijden. De rechthoek moet vanuit de lidar even breed zijn (in hoek) als de
    streep, want de stralen ernaast gingen verder; behalve aan een kant waar iets voor
    het wiel staat (hidden: begin, eind van de streep). Grof zoeken, dan verfijnen.
    """
    best = None
    for step, span in ((math.radians(3), math.pi), (math.radians(0.5), math.radians(3)),
                       (math.radians(0.1), math.radians(0.5))):
        center = 0.0 if best is None else best[2]
        n = int(round(span / step))
        for k in range(-n, n + 1) if best else range(n):
            a = center + k * step
            fit = _fit_box(points, sensor, a, length, width, hidden, angle_tol)
            if best is None or fit[0] < best[0]:
                best = (fit[0], fit[1], a, fit[2])
    _, mid, angle, rms = best
    return mid, angle % math.pi, rms


def _fit_box(points, sensor, a, length, width, hidden, angle_tol):
    ux, uy = math.cos(a), math.sin(a)
    pu = [x * ux + y * uy for x, y in points]
    pv = [-x * uy + y * ux for x, y in points]
    su, sv = sensor[0] * ux + sensor[1] * uy, -sensor[0] * uy + sensor[1] * ux
    cu, cv = _axis_center(pu, su, length), _axis_center(pv, sv, width)
    hl, hw = length / 2, width / 2
    # Verfijnen: elk punt hoort bij zijn dichtstbijzijnde zijde en legt die zijde vast; het
    # gemiddelde per as is minder gevoelig voor ruis en kronkels dan het uiterste punt.
    for _ in range(2):
        on_u, on_v = [], []
        for u, v in zip(pu, pv):
            du, dv = u - cu, v - cv
            if hl - abs(du) < hw - abs(dv):
                on_u.append(u - math.copysign(hl, du))
            else:
                on_v.append(v - math.copysign(hw, dv))
        cu = sum(on_u) / len(on_u) if on_u else cu
        cv = sum(on_v) / len(on_v) if on_v else cv
    sq = 0.0
    for u, v in zip(pu, pv):
        du, dv = abs(u - cu) - hl, abs(v - cv) - hw
        sq += min(max(du, dv), 0.0) ** 2 + max(du, 0.0) ** 2 + max(dv, 0.0) ** 2
    rms = math.sqrt(sq / len(points))
    mid = (cu * ux - cv * uy, cu * uy + cv * ux)

    # Hoekbreedte van de rechthoek tegenover die van de streep (begin en eind van de scan).
    ref = math.atan2(mid[1] - sensor[1], mid[0] - sensor[0])
    dist = math.dist(mid, sensor)

    def bearing(p):
        return wrap(math.atan2(p[1] - sensor[1], p[0] - sensor[0]) - ref)

    corners = [bearing((mid[0] + ux * du - uy * dv, mid[1] + uy * du + ux * dv))
               for du in (-hl, hl) for dv in (-hw, hw)]
    seen_lo, seen_hi = bearing(points[0]), bearing(points[-1])
    lo = max(0.0, min(corners) - seen_lo) if hidden[0] else abs(min(corners) - seen_lo)
    hi = max(0.0, seen_hi - max(corners)) if hidden[1] else abs(max(corners) - seen_hi)
    penalty = (dist * max(0.0, lo - angle_tol)) ** 2 + (dist * max(0.0, hi - angle_tol)) ** 2
    return rms ** 2 + penalty, mid, rms


def _axis_center(values, sensor, size):
    """Midden langs een as: de zijde aan de kant van de lidar ligt op het uiterste punt."""
    lo, hi = min(values), max(values)
    if sensor < lo and hi - lo < size:
        return lo + size / 2
    if sensor > hi and hi - lo < size:
        return hi - size / 2
    return (lo + hi) / 2


def axis_candidates(wheel, trail: float):
    """De twee mogelijke plekken van de draai-as: trail voor of achter het wielmidden."""
    (x, y), a = wheel
    if trail <= 0:
        return [(x, y)]
    dx, dy = trail * math.cos(a), trail * math.sin(a)
    return [(x + dx, y + dy), (x - dx, y - dy)]


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


def match_cart(wheels, prior, castors, gate: float, trail: float, trust_prior: bool = False,
               max_rms: float = 0.02):
    """Zoek de kar rond de verwachte pose prior. Elke draai-as mag een as-kandidaat van een
    wiel binnen gate van zijn verwachte plek krijgen (elk wiel hooguit één keer, minstens
    2 wielen). De kleinste fout wint, met MISSING_COST per ontbrekend wiel: een slecht
    herkend wiel weglaten is beter dan het meenemen.

    Past ook de gespiegelde toewijzing (van elk wiel de andere as-kandidaat) bijna even
    goed (hooguit FLIP_MARGIN slechter), dan staan
    de wielen (bijna) allemaal dezelfde kant op en is niet te zien of de assen voor of
    achter de wielen zitten. Dan het gemiddelde van beide (= midden van de wielen, hooguit
    trail ernaast), of met trust_prior de oplossing die duidelijk het dichtst bij prior
    ligt. Geeft (pose, aantal wielen, rms) of None.
    """
    cands = [(i, p) for i, w in enumerate(wheels) for p in axis_candidates(w, trail)]
    expected = [transform(prior, c) for c in castors]
    options = [[k for k, (_, p) in enumerate(cands) if math.dist(p, e) <= gate] + [None]
               for e in expected]

    def fit(choice):
        model = [castors[i] for i, k in enumerate(choice) if k is not None]
        return fit_pose(model, [cands[k][1] for k in choice if k is not None])

    best = None
    for choice in itertools.product(*options):
        used = [k for k in choice if k is not None]
        if len(used) < 2 or len({cands[k][0] for k in used}) != len(used):
            continue
        pose, rms = fit(choice)
        cost = rms + MISSING_COST * (len(castors) - len(used))
        if rms <= max_rms and (best is None or cost < best[0]):
            best = (cost, len(used), pose, rms, choice)
    if best is None:
        return None
    _, n, pose, rms, choice = best
    if trail > 0:
        flipped, flipped_rms = fit([None if k is None else k ^ 1 for k in choice])
        if flipped_rms <= rms + FLIP_MARGIN:
            closer = math.dist(pose[:2], prior[:2]) - math.dist(flipped[:2], prior[:2])
            if trust_prior and abs(closer) > trail / 2:
                pose = flipped if closer > 0 else pose
            else:
                pose = ((pose[0] + flipped[0]) / 2, (pose[1] + flipped[1]) / 2,
                        wrap(pose[2] + wrap(flipped[2] - pose[2]) / 2))
    return pose, n, rms


def wheels_in_lane(wheels, cart, length: float, width: float, half_lane: float):
    """Wielen (midden, richting) waarvan de rechthoek binnen half_lane van de as van de kar
    komt (kar-frame |y|), dus in de rijbaan van de robot staat."""
    blocking = []
    for (x, y), a in wheels:
        _, wy, wa = compose(inverse(cart), (x, y, a))
        reach = abs(length / 2 * math.sin(wa)) + abs(width / 2 * math.cos(wa))
        if abs(wy) - reach < half_lane:
            blocking.append(((x, y), a))
    return blocking


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
