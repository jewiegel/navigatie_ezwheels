"""Unittests voor de HMI-bridge (codec, buzzer, watchdog, in-memory backend) — zonder ROS
of MQTT.

Draaien:  python3 -m unittest discover -s test -p "test_hmi*" -v   (vanuit navigatie_ezwheels/)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from hmi_bridge.BuzzerPlayer import BuzzerPlayer  # noqa: E402
from hmi_bridge.CommandWatchDog import CommandWatchDog  # noqa: E402
from hmi_bridge.HMIMessages import HMIStatus, SignalCommand  # noqa: E402
from hmi_bridge.InMemoryBackend import InMemoryBackend  # noqa: E402
from hmi_bridge.PayloadCodec import PayloadCodec  # noqa: E402


class TestPayloadCodec(unittest.TestCase):

    def test_uitgangen_zoals_de_oude_bridge(self):
        out = PayloadCodec.encode_command(SignalCommand('links', True, 'DRIVING gang_a'))
        self.assertEqual(out, {
            'robot/outputs/lamp_links': b'1', 'robot/outputs/lamp_rechts': b'0',
            'robot/outputs/buzzer': b'1', 'robot/patrol_state': b'rijdend'})

    def test_gevaar_is_beide_lampen(self):
        out = PayloadCodec.encode_command(SignalCommand(blinkers='gevaar'))
        self.assertEqual(out['robot/outputs/lamp_links'], b'1')
        self.assertEqual(out['robot/outputs/lamp_rechts'], b'1')
        self.assertEqual(out['robot/patrol_state'], b'idle')    # nog geen missiestatus

    def test_statuswoorden(self):
        for mission, word in (('ARRIVED gang_a', b'voltooid'), ('BLOCKED x', b'wachten'),
                              ('WAITING_OPERATOR', b'fout'), ('IETS_NIEUWS', b'iets_nieuws')):
            out = PayloadCodec.encode_command(SignalCommand(mission=mission))
            self.assertEqual(out['robot/patrol_state'], word)

    def test_knoppen(self):
        self.assertEqual(PayloadCodec.decode_status('robot/start_patrol', b'1'),
                         HMIStatus(start=True))
        self.assertEqual(PayloadCodec.decode_status('robot/stop_patrol', b' 1\n'),
                         HMIStatus(stop=True))
        self.assertIsNone(PayloadCodec.decode_status('robot/start_patrol', b'0'))
        self.assertIsNone(PayloadCodec.decode_status('robot/iets', b'1'))


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


class TestInMemoryBackend(unittest.TestCase):

    def test_versturen_en_ontvangen(self):
        backend = InMemoryBackend()
        backend.connect()
        self.assertTrue(backend.is_connected())
        backend.send(SignalCommand(blinkers='rechts'))
        self.assertEqual(backend.sent, [SignalCommand(blinkers='rechts')])
        backend.receive(HMIStatus(start=True))
        self.assertTrue(backend.latest_status().start)


if __name__ == '__main__':
    unittest.main()
