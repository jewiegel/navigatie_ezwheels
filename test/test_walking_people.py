"""Lopende paspoppen (WalkingPeopleNode): route, benen in loopritme en de te spawnen modellen."""
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot', 'nodes')))

from WalkingPeopleNode import (  # noqa: E402
    HIP_HALF_WIDTH, STEP_AMPLITUDE, leg_positions, models, pose, route, routes)

PEOPLE = os.path.join(os.path.dirname(__file__), '..', 'config', 'people.yaml')
HEEN_EN_WEER = route((0.0, 0.0), (4.0, 0.0), speed=1.0, turn_s=1.0)   # 4 m naar +x en terug


def test_route_heen_omdraaien_terug():
    assert HEEN_EN_WEER == [(0.0, 0.0, 0.0, 0.0), (4.0, 4.0, 0.0, 0.0), (5.0, 4.0, 0.0, math.pi),
                            (9.0, 0.0, 0.0, math.pi), (10.0, 0.0, 0.0, 0.0)]


def test_pose_interpoleert_en_loopt_rond():
    assert pose(HEEN_EN_WEER, 2.0) == (2.0, 0.0, 0.0)
    assert pose(HEEN_EN_WEER, 4.5) == (4.0, 0.0, math.pi / 2)   # halverwege het omdraaien
    assert pose(HEEN_EN_WEER, 7.0) == (2.0, 0.0, math.pi)       # terug
    assert pose(HEEN_EN_WEER, 12.0) == (2.0, 0.0, 0.0)          # tweede ronde


def test_benen_naast_elkaar_bij_stilstaan():
    legs = leg_positions(HEEN_EN_WEER, 4.5)   # omdraaien: kijkt naar +y, links is dan -x
    assert math.isclose(legs['been_links'][0], 4.0 - HIP_HALF_WIDTH)
    assert math.isclose(legs['been_rechts'][0], 4.0 + HIP_HALF_WIDTH)


def test_benen_om_en_om_bij_lopen():
    for t in (0.3, 1.0, 1.7, 2.6):
        legs = leg_positions(HEEN_EN_WEER, t)
        x = pose(HEEN_EN_WEER, t)[0]
        voor_links = legs['been_links'][0] - x
        assert math.isclose(voor_links, -(legs['been_rechts'][0] - x))   # tegengesteld
        assert abs(voor_links) <= STEP_AMPLITUDE + 1e-9
        assert math.isclose(legs['been_links'][1], HIP_HALF_WIDTH)       # links van +x is +y


def test_modellen_uit_config():
    with open(PEOPLE) as f:
        config = yaml.safe_load(f)
    spawn = models(config)
    assert len(spawn) == 3 * len(config['walkers'])
    for name, w in config['walkers'].items():
        assert spawn[f'{name}_romp'] == ('paspop_romp', *w['start'])
        assert spawn[f'{name}_been_links'][0] == 'paspop_been'
    for points in routes(config).values():
        assert points[0][1:3] == points[-1][1:3]        # eindigt waar hij begint: naadloze lus
