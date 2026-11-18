"""PersonDetectorNode: scan voorbereiden, assen omzetten en DR-SPAAM op een kunstmatige scan."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot', 'nodes')))

from PersonDetectorNode import NO_RETURN, prepare_scan, to_laser_xy  # noqa: E402

CHECKPOINT = '/opt/dr_spaam/dr_spaam_e40.pth'
ANGLES = np.linspace(-math.radians(100), math.radians(100), 534)   # zoals onze lidar


def scan_with_legs(legs, radius=0.06, max_range=12.0):
    """Afstand per straal tot de dichtstbijzijnde beencirkel, inf als niets geraakt wordt."""
    ranges = np.full(len(ANGLES), np.inf)
    for cx, cy in legs:
        dx, dy = np.cos(ANGLES), np.sin(ANGLES)
        b = dx * cx + dy * cy
        disc = b ** 2 - (cx ** 2 + cy ** 2 - radius ** 2)
        hit = disc >= 0
        ranges[hit] = np.minimum(ranges[hit], b[hit] - np.sqrt(disc[hit]))
    ranges[ranges > max_range] = np.inf
    return ranges


def test_prepare_scan_vervangt_geen_echo():
    scan = prepare_scan([1.0, float('inf'), float('nan'), 0.0, 2.5])
    assert scan.tolist() == [1.0, NO_RETURN, NO_RETURN, NO_RETURN, 2.5]


def test_to_laser_xy_draait_assen_om():
    # DR-SPAAM: (r sin phi, r cos phi). Recht vooruit (phi = 0) op 2 m -> ROS x = 2, y = 0.
    assert to_laser_xy(np.array([[0.0, 2.0], [1.0, 0.0]])).tolist() == [[2.0, 0.0], [0.0, 1.0]]


@pytest.mark.skipif(not os.path.exists(CHECKPOINT), reason='DR-SPAAM zit alleen in de Docker-image')
def test_detector_vindt_benen_recht_vooruit():
    from dr_spaam.detector import Detector
    detector = Detector('DR-SPAAM', CHECKPOINT, gpu=False, stride=2)
    detector.set_laser_spec(ANGLES[1] - ANGLES[0], len(ANGLES))
    scan = prepare_scan(scan_with_legs([(2.0, 0.1), (2.0, -0.1)]))
    for _ in range(3):   # DR-SPAAM gebruikt ook de vorige scans
        dets_xy, dets_cls, _ = detector(scan)
    xy = to_laser_xy(dets_xy)
    best = int(np.argmax(dets_cls.reshape(-1)))
    assert dets_cls.reshape(-1)[best] >= 0.1
    assert math.dist(xy[best], (2.0, 0.0)) < 0.3
