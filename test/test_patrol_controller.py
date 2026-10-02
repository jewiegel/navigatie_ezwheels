"""Unittests voor de patrouille-logica — zonder ROS, Nav2 of robot.

Draaien:  python3 -m unittest discover -s test -v    (vanuit navigatie_ezwheels/)
     of:  python3 -m pytest test
"""
import os
import tempfile
import unittest

from fakes import Rig

from patrol import PatrolConfig, load_routes


class TestStartStop(unittest.TestCase):

    def test_begint_in_idle(self):
        rig = Rig()
        self.assertEqual(rig.state, 'IdleState')
        self.assertEqual(rig.signals.states, ['idle'])

    def test_start_gaat_naar_opstarten_en_selecteert_planner(self):
        rig = Rig()
        rig.ctl.start(1)
        self.assertEqual(rig.state, 'StartingState')
        self.assertEqual(rig.signals.states[-1], 'planning')
        self.assertEqual(rig.nav.planners, ['GridBased'])
        self.assertEqual(len(rig.nav.prepare_callbacks), 1)

    def test_vertrekt_na_controller_reset_vertraging(self):
        rig = Rig()
        rig.ctl.start(1)
        rig.nav.prepare_callbacks[0](True)
        rig.clock.advance(7.9)
        self.assertEqual(rig.state, 'StartingState')
        rig.clock.advance(0.2)
        self.assertEqual(rig.state, 'DrivingState')
        self.assertEqual(rig.nav.goal.waypoints, rig.config.routes[0].waypoints)

    def test_vertrekt_sneller_zonder_controller_reset(self):
        rig = Rig()
        rig.ctl.start(1)
        rig.nav.prepare_callbacks[0](False)
        rig.clock.advance(1.0)
        self.assertEqual(rig.state, 'DrivingState')

    def test_stop_tijdens_opstarten_wordt_niet_genegeerd(self):
        """Bug 1 uit de review: stop in de ~8 s opstartfase werd genegeerd."""
        rig = Rig()
        rig.ctl.start(1)
        rig.ctl.stop()
        self.assertEqual(rig.state, 'StoppedState')
        rig.nav.prepare_callbacks[0](True)       # late callback van Nav2
        rig.clock.advance(60)
        self.assertEqual(rig.state, 'StoppedState')
        self.assertEqual(rig.nav.goals, [])

    def test_tweede_start_tijdens_opstarten_genegeerd(self):
        rig = Rig()
        rig.ctl.start(1)
        rig.ctl.start(2)
        self.assertEqual(len(rig.nav.prepare_callbacks), 1)

    def test_start_genegeerd_tijdens_rijden(self):
        rig = Rig()
        rig.start_and_drive()
        rig.ctl.start(1)
        self.assertEqual(rig.state, 'DrivingState')
        self.assertEqual(len(rig.nav.prepare_callbacks), 1)

    def test_onbekende_route_genegeerd(self):
        rig = Rig()
        rig.ctl.start(99)
        self.assertEqual(rig.state, 'IdleState')

    def test_route_2_gebruikt_eigen_waypoints(self):
        rig = Rig()
        rig.start_and_drive(2)
        self.assertEqual(rig.nav.goal.waypoints, rig.config.routes[1].waypoints)

    def test_stop_tijdens_rijden_annuleert_en_staat_stil(self):
        rig = Rig()
        rig.start_and_drive()
        rig.ctl.stop()
        self.assertEqual(rig.state, 'StoppedState')
        self.assertTrue(rig.nav.goal.cancelled)
        self.assertEqual(rig.signals.indicators[-1], 'uit')

    def test_stop_genegeerd_in_idle(self):
        rig = Rig()
        rig.ctl.stop()
        self.assertEqual(rig.state, 'IdleState')

    def test_herstart_na_voltooid_begint_bij_waypoint_1(self):
        rig = Rig()
        rig.start_and_drive()
        rig.nav.feedback(1)
        rig.nav.succeed()
        rig.start_and_drive()
        self.assertEqual(rig.ctl.current_index, 0)
        self.assertEqual(len(rig.nav.goal.waypoints), 3)


class TestRijden(unittest.TestCase):

    def test_feedback_boekt_voortgang(self):
        rig = Rig()
        rig.start_and_drive()
        rig.nav.feedback(2)    # 3 gestuurd, 2 resterend → waypoint 1 gepasseerd
        self.assertEqual(rig.ctl.current_index, 1)
        self.assertEqual(rig.ctl.last_successful_idx, 0)

    def test_succes_geeft_voltooid_met_gevarenlichten(self):
        rig = Rig()
        rig.start_and_drive()
        rig.nav.succeed()
        self.assertEqual(rig.state, 'CompletedState')
        self.assertEqual(rig.signals.states[-1], 'voltooid')
        self.assertEqual(rig.signals.indicators[-1], 'gevaar')

    def test_rijden_zet_lampen_uit(self):
        rig = Rig()
        rig.start_and_drive()
        self.assertEqual(rig.signals.indicators[-1], 'uit')

    def test_nav2_onbereikbaar_geeft_fout(self):
        rig = Rig()
        rig.nav.available = False
        rig.ctl.start(1)
        rig.nav.prepare_callbacks[0](True)
        rig.clock.advance(8)
        self.assertEqual(rig.state, 'ErrorState')

    def test_geweigerd_doel_eerst_opnieuw_proberen(self):
        rig = Rig()
        rig.ctl.start(1)
        rig.nav.prepare_callbacks[0](True)
        rig.clock.advance(8)
        rig.nav.reject()
        self.assertEqual(rig.state, 'DrivingState')
        rig.clock.advance(4)
        self.assertEqual(len(rig.nav.goals), 2)
        rig.nav.reject()       # ook na retry geweigerd → blokkade
        self.assertEqual(rig.state, 'WaitingState')

    def test_timeout_zonder_voortgang_is_blokkade(self):
        rig = Rig()
        rig.start_and_drive()
        rig.clock.advance(59)
        self.assertEqual(rig.state, 'DrivingState')
        rig.clock.advance(2)
        self.assertEqual(rig.state, 'WaitingState')

    def test_voortgang_reset_timeout(self):
        rig = Rig()
        rig.start_and_drive()
        rig.clock.advance(50)
        rig.nav.feedback(2)
        rig.clock.advance(50)
        self.assertEqual(rig.state, 'DrivingState')

    def test_laat_resultaat_van_geannuleerd_doel_genegeerd(self):
        rig = Rig()
        rig.start_and_drive()
        old_goal = rig.nav.goal
        rig.ctl.stop()
        old_goal.cb['result'](False, 5)   # CANCELED komt nog binnen
        self.assertEqual(rig.state, 'StoppedState')


class TestEscalatieladder(unittest.TestCase):

    def test_volledige_ladder(self):
        rig = Rig()
        rig.start_and_drive()

        # 1. Blokkade → wachten
        rig.nav.fail()
        self.assertEqual(rig.state, 'WaitingState')
        self.assertTrue(rig.nav.goal.cancelled)
        rig.clock.advance(119)
        self.assertEqual(rig.state, 'WaitingState')

        # 2. Na 120 s → piep + achteruit, gevarenlichten aan
        rig.clock.advance(1)
        self.assertEqual(rig.state, 'BackingUpState')
        self.assertEqual(rig.signals.beeps, 1)
        self.assertEqual(rig.signals.indicators[-1], 'gevaar')
        rig.clock.advance(4.5)
        self.assertTrue(rig.signals.drive_cmds)
        self.assertTrue(all(v == -0.1 for v in rig.signals.drive_cmds))

        # ... na 0,5 m (5 s) → vers pad vanaf hetzelfde waypoint
        rig.clock.advance(0.6)
        self.assertEqual(rig.state, 'DrivingState')
        self.assertEqual(len(rig.nav.goal.waypoints), 3)
        rig.nav.accept()

        # 3. Weer geblokkeerd → alternatief via volgend waypoint (na korte pauze)
        rig.nav.fail()
        self.assertEqual(rig.state, 'WaitingState')
        rig.clock.advance(2)
        self.assertEqual(rig.state, 'DrivingState')
        self.assertEqual(rig.ctl.current_index, 1)
        self.assertEqual(len(rig.nav.goal.waypoints), 2)
        rig.nav.accept()

        # 4. Weer geblokkeerd → terugkeren
        rig.nav.fail()
        rig.clock.advance(2)
        self.assertEqual(rig.state, 'ReturningState')
        self.assertEqual(rig.nav.goal.kind, 'to')

        # ... terugkeren lukt → altijd fout (operator moet ingrijpen)
        rig.nav.succeed()
        self.assertEqual(rig.state, 'ErrorState')
        self.assertEqual(rig.signals.states[-1], 'fout')

    def test_geen_alternatief_bij_laatste_waypoint_dan_terugkeren(self):
        rig = Rig()
        rig.start_and_drive()
        rig.nav.feedback(1)
        rig.nav.feedback(0)        # current_index = 2 (laatste)
        rig.ctl.ladder_index = 2   # wacht + achteruit al gehad
        rig.ctl.on_blocked()
        rig.clock.advance(2)
        self.assertEqual(rig.state, 'ReturningState')

    def test_gepasseerd_waypoint_reset_ladder(self):
        rig = Rig()
        rig.start_and_drive()
        rig.nav.fail()
        rig.clock.advance(120 + 5.1)       # wachten + achteruit
        rig.nav.accept()
        self.assertEqual(rig.ctl.ladder_index, 2)
        rig.nav.feedback(2)
        self.assertEqual(rig.ctl.ladder_index, 0)
        rig.nav.fail()
        self.assertEqual(rig.state, 'WaitingState')   # weer bij stap 1

    def test_terugkeer_timeout_geeft_fout(self):
        """Bug 2 uit de review: terugkeren herhaalde zich eindeloos."""
        rig = Rig()
        rig.start_and_drive()
        rig.ctl.ladder_index = 3
        rig.ctl.on_blocked()
        rig.clock.advance(2)
        self.assertEqual(rig.state, 'ReturningState')
        rig.clock.advance(rig.config.return_timeout_sec + 1)
        self.assertEqual(rig.state, 'ErrorState')
        self.assertEqual(sum(1 for g in rig.nav.goals if g.kind == 'to'), 1)

    def test_stop_tijdens_achteruit(self):
        rig = Rig()
        rig.start_and_drive()
        rig.nav.fail()
        rig.clock.advance(121)
        self.assertEqual(rig.state, 'BackingUpState')
        rig.ctl.stop()
        n = len(rig.signals.drive_cmds)
        rig.clock.advance(30)
        self.assertEqual(rig.state, 'StoppedState')
        self.assertEqual(len(rig.signals.drive_cmds), n)   # niet meer achteruit

    def test_stop_tijdens_wachten_annuleert_wachttimer(self):
        rig = Rig()
        rig.start_and_drive()
        rig.nav.fail()
        rig.ctl.stop()
        rig.clock.advance(200)
        self.assertEqual(rig.state, 'StoppedState')
        self.assertEqual(rig.signals.beeps, 0)


class TestVeiligheidsstop(unittest.TestCase):

    def _stilgevallen(self):
        rig = Rig(safety=True)
        rig.start_and_drive()
        rig.monitor.on_motion(True)
        rig.monitor.on_motion(False)
        rig.clock.advance(16)
        rig.monitor.check()
        return rig

    def test_lange_stilstand_meldt_wachten(self):
        rig = self._stilgevallen()
        self.assertTrue(rig.ctl.safety_stopped)
        self.assertEqual(rig.signals.states[-1], 'wachten')
        self.assertEqual(rig.state, 'DrivingState')

    def test_weer_rijden_heft_veiligheidsstop_op(self):
        rig = self._stilgevallen()
        rig.monitor.on_motion(True)
        self.assertFalse(rig.ctl.safety_stopped)
        self.assertEqual(rig.signals.states[-1], 'rijdend')

    def test_persoon_blijft_staan_slaat_wachtstap_over(self):
        config = PatrolConfig(nav_timeout_sec=1000)   # timeout mag hier niet eerst vuren
        rig = Rig(config=config, safety=True)
        rig.start_and_drive()
        rig.monitor.on_motion(True)
        rig.monitor.on_motion(False)
        rig.clock.advance(16)
        rig.monitor.check()
        rig.clock.advance(120)
        self.assertEqual(rig.state, 'BackingUpState')


class TestRoutesBestand(unittest.TestCase):

    def test_routes_uit_yaml(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'config', 'patrol_routes.yaml')
        routes = load_routes(path)
        self.assertEqual([r.route_id for r in routes], [1, 2])
        self.assertEqual(routes[0].start_topic, '/start_patrol')
        self.assertEqual(routes[0].waypoints[0], (7.66, -3.49, 0.0))
        self.assertEqual(routes, PatrolConfig().routes)   # yaml == ingebouwde defaults

    def test_fout_waypoint_geeft_duidelijke_melding(self):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
            f.write('routes:\n  - id: 1\n    start_topic: /x\n    waypoints:\n      - [1.0, 2.0]\n')
        try:
            with self.assertRaisesRegex(ValueError, r'\[x, y, yaw\]'):
                load_routes(f.name)
        finally:
            os.unlink(f.name)


if __name__ == '__main__':
    unittest.main()
