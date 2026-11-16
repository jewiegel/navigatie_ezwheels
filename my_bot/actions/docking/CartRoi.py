#!/usr/bin/env python3
"""De kar meten in een ROI rond de plek waar hij hoort te staan, zonder wielherkenning.

1. ROI: rechthoek van de kar plus margin rond de verwachte pose. Alles daarbuiten (stoelen,
   banken, muren) doet niet mee.
2. Rijbaan: de lege strook dwars (y) door de voorste helft van de ROI, minstens lane_min
   breed, het dichtst bij het verwachte midden. Wat links ervan ligt is 'links', rechts is 'rechts'; wat het is
   (wiel, beugel) maakt niet uit.
3. Per kant twee groepjes (voor en achter, gesplitst op het grootste gat in x); hun midden.
4. Midden tussen de vier groepjes = middellijn van de rijbaan; richting = van de voorste naar
   de achterste groepjes. Diepte: midden van de groepjes + depth_offset (de lidar ziet alleen
   de voorkant van de wielen; één keer inmeten).

Punten (x, y) in odom, poses (x, y, theta). Kar-frame: x = rijrichting van de robot onder de
kar, dus de robot komt van -x.
"""
import math

import numpy as np

from .CartDocking import compose, inverse, transform

NEIGHBOUR = 0.03   # punt telt voor de rijbaan pas met een buur binnen 3 cm (los spookpunt niet)
MIN_SPLIT = 0.10   # voor- en achterwiel liggen minstens zover uit elkaar (in x)
WHEEL_SPAN = 0.22  # een wiel is hooguit zo lang (diameter): verder weg = muur/rommel


def stable_points(scans, cell: float = 0.01, min_fraction: float = 0.3):
    """Punten die in minstens min_fraction van de scans terugkomen (vakjes van cell).

    scans: lijst van puntlijsten (odom) van een stilstaande robot. Ruis die maar af en toe
    opduikt valt weg; wat slecht weerkaatst maar wel vaak terugkomt blijft. Geeft het
    gemiddelde punt per vakje.
    """
    hits, sums = {}, {}
    for points in scans:
        seen = set()
        for x, y in points:
            key = (math.floor(x / cell), math.floor(y / cell))
            s = sums.setdefault(key, [0.0, 0.0, 0])
            s[0] += x
            s[1] += y
            s[2] += 1
            if key not in seen:
                seen.add(key)
                hits[key] = hits.get(key, 0) + 1
    need = max(1, math.ceil(min_fraction * len(scans)))
    return [(s[0] / s[2], s[1] / s[2]) for key, s in sums.items() if hits[key] >= need]


def _split_x(side, min_points):
    """Punten van één kant in (voor, achter) op het grootste gat in x; None als dat niet lukt.
    Per wiel alleen WHEEL_SPAN vanaf de binnenkant (kant van het kar-midden): een muur vlak
    achter de kar hangt soms via losse punten aan het achterwiel vast (echte scans: ~55 cm
    achter het midden)."""
    side = side[np.argsort(side[:, 0])]
    if len(side) < 2 * min_points:
        return None
    gaps = np.diff(side[:, 0])
    i = int(np.argmax(gaps))
    front, back = side[:i + 1], side[i + 1:]
    front = front[front[:, 0] >= front[-1, 0] - WHEEL_SPAN]
    back = back[back[:, 0] <= back[0, 0] + WHEEL_SPAN]
    if gaps[i] < MIN_SPLIT or len(front) < min_points or len(back) < min_points:
        return None
    return front, back


def measure_cart(points, prior, half_x, half_y, margin, lane_min, refine_margin=(0.12, 0.06),
                 refine=2, side_band=0.25, depth_offset=0.0, min_points=3):
    """Pose van de kar (odom) uit de punten in de ROI rond prior: (dict, None) of (None, reden).

    half_x, half_y: halve buitenmaat van de kar; margin: zoveel mag de kar van prior staan;
    lane_min: smalste vrije rijbaan die telt (robotbreedte). Daarna refine keer opnieuw met een
    krappe ROI rond de uitkomst, refine_margin = (in de lengte, opzij): met een ruime ROI valt
    er soms een muur of rommel in, of een wiel half buiten. In de lengte niet te krap: de
    zwenkwielen steken voor en achter ~6 cm uit de kar, en een afgeknipt wiel trekt het midden
    mee (Gazebo: 4 cm / 2 graden bij 6 cm). Opzij krap: daar staat de rommel. side_band: per kant alleen punten tot zover buiten de rijbaan.

    dict: pose, groups (vier middens in odom: links-voor, links-achter, rechts-voor,
    rechts-achter), counts, lane (breedte rijbaan).
    """
    result, why = _measure_once(points, prior, half_x, half_y, (margin, margin), lane_min,
                                side_band, depth_offset, min_points)
    for _ in range(refine):
        if result is None:
            break
        result, why = _measure_once(points, result['pose'], half_x, half_y, refine_margin,
                                    lane_min, side_band, depth_offset, min_points)
    return result, why


def _measure_once(points, prior, half_x, half_y, margin, lane_min, side_band, depth_offset,
                  min_points):
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return None, 'geen punten in de ROI'
    inv = inverse(prior)
    c, s = math.cos(inv[2]), math.sin(inv[2])
    q = pts @ np.array([[c, s], [-s, c]]) + np.array(inv[:2])
    margin_x, margin_y = margin
    q = q[(np.abs(q[:, 0]) <= half_x + margin_x) & (np.abs(q[:, 1]) <= half_y + margin_y)]
    if len(q) < 4 * min_points:
        return None, f'te weinig punten in de ROI ({len(q)})'

    # Rijbaan: grootste lege strook in y, alleen punten met een buur (los spookpunt telt niet).
    # Alleen de voorste helft van de ROI (tot margin voorbij het verwachte midden): daar komt
    # de robot binnen; een muur vlak achter de kar valt zo buiten de rijbaan-zoektocht.
    front = q[q[:, 0] <= margin_x]
    d = np.hypot(front[:, None, 0] - front[None, :, 0], front[:, None, 1] - front[None, :, 1])
    solid = front[(d < NEIGHBOUR).sum(axis=1) >= 2]
    ys = np.sort(solid[:, 1])
    gaps = [(ys[i], ys[i + 1]) for i in range(len(ys) - 1) if ys[i + 1] - ys[i] >= lane_min]
    if not gaps:
        widest = float(np.max(np.diff(ys))) if len(ys) > 1 else 0.0
        return None, (f'geen vrije rijbaan van {100 * lane_min:.0f} cm in de ROI '
                      f'(breedste: {100 * widest:.0f} cm)')
    lo, hi = min(gaps, key=lambda g: abs((g[0] + g[1]) / 2))

    groups = []
    # Links/rechts van het midden van de rijbaan: staat de kar scheef t.o.v. de ROI, dan kan
    # een achterwiel verder naar binnen liggen dan de rand die voorin gevonden is.
    centre = (lo + hi) / 2
    for name, side in (('links', q[(q[:, 1] > centre) & (q[:, 1] <= hi + side_band)]),
                       ('rechts', q[(q[:, 1] < centre) & (q[:, 1] >= lo - side_band)])):
        split = _split_x(side, min_points)
        if split is None:
            return None, (f'{name} geen voor- en achterwiel te onderscheiden '
                          f'({len(side)} punten naast de rijbaan)')
        groups += list(split)
    lf, lb, rf, rb = (g.mean(axis=0) for g in groups)
    ahead = (lb + rb - lf - rf) / 2
    ahead /= np.linalg.norm(ahead)
    mid = (lf + lb + rf + rb) / 4 + depth_offset * ahead
    pose = compose(prior, (float(mid[0]), float(mid[1]), math.atan2(ahead[1], ahead[0])))
    return {'pose': pose,
            'groups': [transform(prior, (float(g[0]), float(g[1]))) for g in (lf, lb, rf, rb)],
            'counts': [len(g) for g in groups],
            'lane': float(hi - lo)}, None


__all__ = ['measure_cart', 'stable_points']
