"""Unittests voor build_route/store_route uit RouteManagerServer (set_route en route_draft/save).

Importeert RouteManagerServer, dus rclpy en my_bot_interfaces moeten gesourced zijn
(install/setup.bash); anders worden deze tests overgeslagen.
"""
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

try:
    from RouteManagerServer import build_route, store_route  # noqa: E402
except ImportError as e:  # geen ROS-omgeving
    raise unittest.SkipTest(f'RouteManagerServer niet te importeren: {e}')
from route_manager import Route, Waypoint  # noqa: E402
from route_manager.repositories import YamlRouteRepository  # noqa: E402


def wp(name, x, y, yaw):
    return SimpleNamespace(name=name, x=x, y=y, yaw=yaw)   # zoals my_bot_interfaces/Waypoint


class TestBuildRoute(unittest.TestCase):

    def test_namen_behouden_of_genereren(self):
        route = build_route('gang_d', 'ziekenhuis', [wp('start', 1, 2, 0), wp('', 3, 4, 1.5)])
        self.assertEqual(route.map_id, 'ziekenhuis')
        self.assertEqual([(w.name, w.x, w.y, w.yaw) for w in route.waypoints],
                         [('start', 1, 2, 0), ('gang_d_2', 3, 4, 1.5)])

    def test_ongeldige_invoer_geweigerd(self):
        for name, waypoints in (('', [wp('', 0, 0, 0)]),
                                ('met spatie', [wp('', 0, 0, 0)]),
                                ('r', []),
                                ('r', [wp('', float('nan'), 0, 0)]),
                                ('r', [wp('', 0, 0, float('inf'))])):
            with self.subTest(name=name), self.assertRaises(ValueError):
                build_route(name, 'm', waypoints)


class TestStoreRoute(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.repo = YamlRouteRepository(os.path.join(self.dir.name, 'routes.yaml'))
        self.repo.save(Route('r', 'm', [Waypoint('a', 1, 1, 0)]))

    def tearDown(self):
        self.dir.cleanup()

    def test_bestaande_naam_zonder_overwrite_geweigerd(self):
        with self.assertRaisesRegex(ValueError, 'bestaat al'):
            store_route(self.repo, Route('r', 'm', [Waypoint('b', 2, 2, 0)]), False)
        self.assertEqual(self.repo.load('r').waypoints[0].name, 'a')

    def test_overwrite_vervangt(self):
        store_route(self.repo, build_route('r', 'm', [wp('b', 2, 2, 0)]), True)
        self.assertEqual(self.repo.load('r').waypoints[0].name, 'b')

    def test_nieuwe_route(self):
        store_route(self.repo, build_route('s', 'm', [wp('', 2, 2, 0)]), False)
        self.assertEqual(self.repo.list(), ['r', 's'])


if __name__ == '__main__':
    unittest.main()
