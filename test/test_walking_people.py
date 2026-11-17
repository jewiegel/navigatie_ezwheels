"""Looproute van de actors uit de wereld lezen en interpoleren (WalkingPeopleNode)."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot', 'nodes')))

from WalkingPeopleNode import position, trajectories  # noqa: E402

WORLD = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'hospital_aws_people.sdf')
HEEN_EN_WEER = [(0.0, 0.0, 0.0), (4.0, 4.0, 0.0), (5.0, 4.0, 0.0), (9.0, 0.0, 0.0), (10.0, 0.0, 0.0)]


def test_position_interpoleert_en_loopt_rond():
    assert position(HEEN_EN_WEER, 2.0) == (2.0, 0.0)
    assert position(HEEN_EN_WEER, 4.5) == (4.0, 0.0)    # omdraaien op de plek
    assert position(HEEN_EN_WEER, 7.0) == (2.0, 0.0)    # terug
    assert position(HEEN_EN_WEER, 12.0) == (2.0, 0.0)   # tweede ronde


def test_trajectories_uit_wereld():
    routes = trajectories(WORLD)
    assert set(routes) == {'loper_lobby_dwars', 'loper_lobby_oost', 'loper_gang_liften'}
    for points in routes.values():
        assert points[0][1:] == points[-1][1:]          # eindigt waar hij begint: naadloze lus
        assert [t for t, _, _ in points] == sorted(t for t, _, _ in points)
