"""Unittests voor YamlRouteRepository en RouteDraft — zonder ROS.

Draaien:  python3 -m unittest discover -s test -p "test_route*" -v   (vanuit navigatie_ezwheels/)
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from route_manager import Route, RouteDraft, Waypoint  # noqa: E402
from route_manager.repositories import YamlRouteRepository  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'routes.yaml')


class TestYamlRouteRepository(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, 'routes.yaml')
        self.repo = YamlRouteRepository(self.path)

    def tearDown(self):
        self.dir.cleanup()

    def write(self, text):
        with open(self.path, 'w') as f:
            f.write(text)

    def test_ontbrekend_bestand_is_leeg(self):
        self.assertEqual(self.repo.list(), [])
        with self.assertRaises(KeyError):
            self.repo.load('gang_a')

    def test_save_load_list_delete(self):
        route = Route('gang_a', 'ziekenhuis', [Waypoint('a', 1.0, 2.0, 0.5), Waypoint('b', 3, 4, 0)])
        self.repo.save(route)
        self.repo.save(Route('gang_b', 'ziekenhuis', [Waypoint('c', 0, 0, 0)]))
        self.assertEqual(self.repo.list(), ['gang_a', 'gang_b'])

        loaded = self.repo.load('gang_a')
        self.assertEqual(loaded.map_id, 'ziekenhuis')
        self.assertEqual([(w.name, w.x, w.y, w.yaw) for w in loaded.waypoints],
                         [('a', 1.0, 2.0, 0.5), ('b', 3.0, 4.0, 0.0)])

        self.repo.delete('gang_a')
        self.assertEqual(self.repo.list(), ['gang_b'])
        with self.assertRaises(KeyError):
            self.repo.delete('gang_a')

    def test_opslaan_bewaart_commentaar_bovenaan(self):
        self.write('# uitleg\n# meer uitleg\n\nroutes:\n  r:\n    waypoints:\n      - {x: 1, y: 2, yaw: 0}\n')
        self.repo.save(Route('s', 'm', [Waypoint('a', 1, 2, 0)]))
        with open(self.path) as f:
            text = f.read()
        self.assertTrue(text.startswith('# uitleg\n# meer uitleg\n'))
        self.assertEqual(self.repo.list(), ['r', 's'])

    def test_opslaan_houdt_rechten(self):
        self.write('routes: {}\n')
        os.chmod(self.path, 0o644)
        self.repo.save(Route('s', 'm', [Waypoint('a', 1, 2, 0)]))
        self.assertEqual(os.stat(self.path).st_mode & 0o777, 0o644)

    def test_waypoint_naam_optioneel(self):
        self.write('routes:\n  r:\n    waypoints:\n      - {x: 1, y: 2, yaw: 0}\n')
        self.assertEqual(self.repo.load('r').waypoints[0].name, 'wp1')

    def test_ongeldig_waypoint_geeft_duidelijke_fout(self):
        self.write('routes:\n  r:\n    waypoints:\n      - {x: 1, y: 2, yaw: 0}\n      - {x: 1, y: abc, yaw: 0}\n')
        with self.assertRaisesRegex(ValueError, 'Route "r", waypoint 2'):
            self.repo.load('r')

    def test_route_zonder_waypoints_ongeldig(self):
        self.write('routes:\n  r:\n    map_id: m\n    waypoints: []\n')
        with self.assertRaisesRegex(ValueError, 'geen waypoints'):
            self.repo.load('r')

    def test_meegeleverd_routebestand_is_geldig(self):
        repo = YamlRouteRepository(CONFIG)
        for name in repo.list():
            self.assertTrue(repo.load(name).waypoints)


class TestRouteDraft(unittest.TestCase):

    def test_toevoegen_undo_en_bouwen(self):
        draft = RouteDraft()
        draft.add(1.0, 2.0, 0.0)
        draft.add(3.0, 4.0, 1.5)
        draft.add(9.0, 9.0, 0.0)
        self.assertEqual(draft.undo().x, 9.0)
        route = draft.build('gang_c', 'ziekenhuis')
        self.assertEqual(route.map_id, 'ziekenhuis')
        self.assertEqual([(w.name, w.x, w.y, w.yaw) for w in route.waypoints],
                         [('gang_c_1', 1.0, 2.0, 0.0), ('gang_c_2', 3.0, 4.0, 1.5)])

    def test_undo_en_clear_op_lege_draft(self):
        draft = RouteDraft()
        self.assertIsNone(draft.undo())
        draft.add(1, 1, 0)
        draft.clear()
        self.assertEqual(draft.waypoints, [])

    def test_lege_draft_niet_te_bouwen(self):
        with self.assertRaisesRegex(ValueError, 'geen waypoints'):
            RouteDraft().build('r', 'm')

    def test_ongeldige_naam(self):
        draft = RouteDraft()
        draft.add(0, 0, 0)
        for name in ('', 'met spatie', 'a/b', 'x:y'):
            with self.assertRaises(ValueError):
                draft.build(name, 'm')

    def test_niet_eindige_waarden_geweigerd(self):
        with self.assertRaises(ValueError):
            RouteDraft().add(float('nan'), 0, 0)

    def test_opgeslagen_draft_is_te_laden(self):
        with tempfile.TemporaryDirectory() as d:
            repo = YamlRouteRepository(os.path.join(d, 'routes.yaml'))
            draft = RouteDraft()
            draft.add(1.0, 2.0, 0.5)
            repo.save(draft.build('nieuw', 'ziekenhuis'))
            self.assertEqual(repo.load('nieuw').waypoints[0].name, 'nieuw_1')


if __name__ == '__main__':
    unittest.main()
