"""Unittests voor de HMI-bridge (inputs, outputs, buzzer, watchdog) — zonder broker; de
inputs/outputs krijgen een nep-node als ctx.

Draaien:  source ~/ros2_ws/install/setup.bash && /usr/bin/python3 -m unittest discover -s test -p "test_hmi*" -v   (vanuit navigatie_ezwheels/)
"""
import json
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from sensor_msgs.msg import BatteryState  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from hmi_bridge.BuzzerPlayer import BuzzerPlayer  # noqa: E402
from hmi_bridge.CommandWatchDog import CommandWatchDog  # noqa: E402
from hmi_bridge.inputs.ModeInput import ModeInput  # noqa: E402
from hmi_bridge.inputs.StartButtonInput import StartButtonInput  # noqa: E402
from hmi_bridge.inputs.StopButtonInput import StopButtonInput  # noqa: E402
from hmi_bridge.outputs.BatteryOutput import BatteryOutput  # noqa: E402
from hmi_bridge.outputs.BuzzerOutput import BuzzerOutput  # noqa: E402
from hmi_bridge.outputs.LampsOutput import LampsOutput  # noqa: E402
from hmi_bridge.outputs.StateOutput import StateOutput  # noqa: E402


class FakeNode:
    """Wat de inputs/outputs van de HMIBridgeNode gebruiken (ctx), zonder ROS."""

    def __init__(self, **params):
        self.params, self.t = params, 0.0
        self.subs, self.calls, self.published, self.logs = {}, [], [], []

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.params.get(name, default))

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs[topic] = callback

    def create_client(self, srv_type, name):
        return name

    def call(self, client, request):
        self.calls.append((client, request))

    def mqtt_publish(self, topic, payload):
        self.published.append((topic, payload))

    def now(self):
        return self.t

    def get_logger(self):
        log = lambda m: self.logs.append(m)  # noqa: E731
        return SimpleNamespace(info=log, warn=log, error=log)


def setup(part, **params):
    node = FakeNode(**params)
    part.setup(node)
    return node


class TestOutputs(unittest.TestCase):

    def test_lampen_links_en_gevaar_knipperen(self):
        lamps = LampsOutput()
        node = setup(lamps)
        self.assertEqual(lamps.payloads(0.0), {'robot/outputs/lamp_links': b'0',
                                               'robot/outputs/lamp_rechts': b'0'})
        node.t = 10.0
        node.subs['indicators'](String(data='links'))
        self.assertEqual(lamps.payloads(10.1), {'robot/outputs/lamp_links': b'1',
                                                'robot/outputs/lamp_rechts': b'0'})
        self.assertEqual(lamps.payloads(10.6)['robot/outputs/lamp_links'], b'0')  # knipperfase
        node.subs['indicators'](String(data='gevaar'))
        self.assertEqual(set(lamps.payloads(10.1).values()), {b'1'})

    def test_buzzer(self):
        buzzer = BuzzerOutput()
        node = setup(buzzer)
        node.subs['buzzer_pattern'](String(data='enkel'))
        self.assertEqual(buzzer.payloads(0.1), {'robot/outputs/buzzer': b'1'})
        self.assertEqual(buzzer.payloads(0.5), {'robot/outputs/buzzer': b'0'})
        node.subs['buzzer_pattern'](String(data='toeter'))
        self.assertIn('onbekend', node.logs[-1])

    def test_statuswoorden_en_state_id(self):
        state = StateOutput()
        node = setup(state)
        self.assertEqual(state.payloads(0), {'robot/patrol_state': b'idle',
                                             'robot/outputs/state_id': b'0'})
        for mission, word, bit in (('DRIVING gang_a', b'rijdend', b'0'),
                                   ('ARRIVED gang_a', b'voltooid', b'0'),
                                   ('BLOCKED x', b'wachten', b'1'),
                                   ('WAITING_OPERATOR', b'fout', b'0'),
                                   ('IETS_NIEUWS', b'iets_nieuws', b'0')):
            node.subs['mission_status'](String(data=mission))
            self.assertEqual(state.payloads(0), {'robot/patrol_state': word,
                                                 'robot/outputs/state_id': bit})
        node.subs['mission_status'](String(data='DRIVING gang_a'))
        node.subs['navigation/blocked'](Bool(data=True))
        self.assertEqual(state.payloads(0)['robot/outputs/state_id'], b'1')

    def test_batterij(self):
        battery = BatteryOutput()
        node = setup(battery)
        self.assertEqual(battery.payloads(0), {})       # nog onbekend: niets sturen
        node.subs['battery_state'](BatteryState(voltage=27.3))
        self.assertEqual(battery.payloads(0), {'robot/outputs/batterij': b'90'})
        node.subs['battery_state'](BatteryState(voltage=30.0))
        self.assertEqual(battery.payloads(0), {'robot/outputs/batterij': b'100'})
        battery = BatteryOutput()
        node = setup(battery, battery_empty_v=0.0)      # percentage van de driver
        node.subs['battery_state'](BatteryState(percentage=0.87))
        self.assertEqual(battery.payloads(0), {'robot/outputs/batterij': b'87'})


class TestInputs(unittest.TestCase):

    def test_startknop(self):
        start = StartButtonInput()
        node = setup(start, start_route_name='gang_a')
        start.on_message(b'0')                          # losgelaten
        self.assertEqual(node.calls, [])
        start.on_message(b' 1\n')
        self.assertEqual(node.calls[0][0], 'start_route')
        self.assertEqual(node.calls[0][1].route_name, 'gang_a')

    def test_startknop_zonder_route(self):
        start = StartButtonInput()
        node = setup(start)
        start.on_message(b'1')
        self.assertEqual(node.calls, [])
        self.assertIn('leeg', node.logs[-1])

    def test_stopknop(self):
        stop = StopButtonInput()
        node = setup(stop)
        stop.on_message(b'1')
        self.assertEqual(node.calls[0][0], 'cancel_mission')

    def test_modus_verzoek_volgens_ezway_api(self):
        mode = ModeInput('robot/service', 'SERVICE')
        node = setup(mode)
        mode.on_message(b'1')
        mode.on_message(b'1')
        (topic, first), (_, second) = node.published
        self.assertEqual(topic, 'amr_manager/request')
        req = json.loads(first)
        self.assertEqual(req['cmd'], 'operating_mode_cmd')
        self.assertEqual(req['cmd_params'], {'state': 'SERVICE'})
        self.assertTrue(req['timestamp'].endswith('Z'))
        self.assertNotEqual(req['requestId'], json.loads(second)['requestId'])


class TestBuzzerPlayer(unittest.TestCase):

    def test_enkel_en_dubbel(self):
        player = BuzzerPlayer()
        self.assertFalse(player.is_on(0.0))
        player.play('enkel', 10.0)
        self.assertTrue(player.is_on(10.1))
        self.assertFalse(player.is_on(10.4))
        self.assertFalse(player.is_on(20.0))        # niet herhalen
        player.play('dubbel', 0.0)
        self.assertEqual([player.is_on(t) for t in (0.1, 0.2, 0.35, 0.5)],
                         [True, False, True, False])

    def test_elke_5s_tot_uit(self):
        player = BuzzerPlayer()
        player.play('elke_5s', 0.0)
        self.assertTrue(player.is_on(5.1))
        self.assertFalse(player.is_on(7.0))
        self.assertTrue(player.is_on(10.2))
        player.play('uit', 11.0)
        self.assertFalse(player.is_on(15.1))

    def test_onbekend_patroon_stopt_en_meldt(self):
        player = BuzzerPlayer()
        player.play('elke_5s', 0.0)
        with self.assertRaises(ValueError):
            player.play('toeter', 1.0)
        self.assertFalse(player.is_on(5.1))


class TestCommandWatchDog(unittest.TestCase):

    def test_verloopt_zonder_voeding(self):
        dog = CommandWatchDog(timeout_s=3.0)
        self.assertTrue(dog.expired(0.0))
        dog.feed(10.0)
        self.assertFalse(dog.expired(13.0))
        self.assertTrue(dog.expired(13.1))


if __name__ == '__main__':
    unittest.main()
