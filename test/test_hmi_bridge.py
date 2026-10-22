"""Unittests voor de HMI-bridge (codec, watchdog, in-memory backend) — zonder ROS of MQTT.

Draaien:  python3 -m unittest discover -s test -p "test_hmi*" -v   (vanuit navigatie_ezwheels/)
"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from hmi_bridge.CommandWatchDog import CommandWatchDog  # noqa: E402
from hmi_bridge.HMIMessages import HMIStatus, SignalCommand  # noqa: E402
from hmi_bridge.InMemoryBackend import InMemoryBackend  # noqa: E402
from hmi_bridge.PayloadCodec import PayloadCodec  # noqa: E402


class TestPayloadCodec(unittest.TestCase):

    def test_command_als_json(self):
        cmd = SignalCommand('links', 'enkel', 3, 'DRIVING gang_a')
        self.assertEqual(json.loads(PayloadCodec.encode_command(cmd, 42)), {
            'v': 1, 'seq': 42, 'blinkers': 'links', 'buzzer': 'enkel', 'buzzer_id': 3,
            'mission': 'DRIVING gang_a'})

    def test_status_lezen(self):
        status = PayloadCodec.decode_status(b'{"v": 1, "ack": 42, "start": true}')
        self.assertEqual(status, HMIStatus(ack=42, start=True, stop=False))

    def test_ongeldige_status(self):
        for payload in (b'geen json', b'[1]', b'{"v": 2}', b'{"ack": 1}',
                        b'{"v": 1, "ack": "42"}', b'{"v": 1, "ack": true}',
                        b'{"v": 1, "start": "ja"}', b'\xff'):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                PayloadCodec.decode_status(payload)


class TestCommandWatchDog(unittest.TestCase):

    def test_verloopt_zonder_voeding(self):
        dog = CommandWatchDog(timeout_s=3.0)
        self.assertTrue(dog.expired(0.0))        # nog nooit iets ontvangen
        dog.feed(10.0)
        self.assertFalse(dog.expired(13.0))
        self.assertTrue(dog.expired(13.1))


class TestInMemoryBackend(unittest.TestCase):

    def test_versturen_en_ontvangen(self):
        backend = InMemoryBackend()
        backend.connect()
        self.assertTrue(backend.is_connected())
        backend.send(SignalCommand(blinkers='rechts'), 1)
        self.assertEqual(backend.sent, [(SignalCommand(blinkers='rechts'), 1)])
        self.assertIsNone(backend.latest_status())
        backend.receive(HMIStatus(ack=1))
        self.assertEqual(backend.latest_status().ack, 1)


if __name__ == '__main__':
    unittest.main()
