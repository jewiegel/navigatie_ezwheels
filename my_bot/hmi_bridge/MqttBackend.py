#!/usr/bin/env python3

import logging
from typing import Optional

import paho.mqtt.client as mqtt

from hmi_bridge.HMIMessages import HMIStatus, SignalCommand
from hmi_bridge.PayloadCodec import TOPIC_START, TOPIC_STOP, PayloadCodec
from hmi_bridge.SignalBackend import SignalBackend


class MqttBackend(SignalBackend):
    """Verbindt op de achtergrond en opnieuw na een onderbreking (paho loop-thread)."""

    def __init__(self, host: str, port: int, logger=None):
        self.host, self.port = host, port
        self.log = logger or logging.getLogger('hmi_bridge')
        self._status: Optional[HMIStatus] = None
        # paho 2.x wil een callback-API-versie; VERSION1 = dezelfde callbacks als paho 1.x
        kwargs = ({'callback_api_version': mqtt.CallbackAPIVersion.VERSION1}
                  if hasattr(mqtt, 'CallbackAPIVersion') else {})
        self._client = mqtt.Client(client_id='hmi_bridge', **kwargs)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def connect(self):
        self._client.connect_async(self.host, self.port, keepalive=10)
        self._client.loop_start()

    def disconnect(self):
        self._client.disconnect()
        self._client.loop_stop()

    def is_connected(self):
        return self._client.is_connected()

    def send(self, cmd: SignalCommand):
        for topic, payload in PayloadCodec.encode_command(cmd).items():
            self._client.publish(topic, payload)

    def latest_status(self):
        return self._status

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.log.error(f'MQTT-verbinding geweigerd (rc={rc})')
            return
        self.log.info(f'MQTT verbonden met {self.host}:{self.port}')
        for topic in (TOPIC_START, TOPIC_STOP):    # opnieuw na elke (her)verbinding
            client.subscribe(topic)

    def _on_message(self, client, userdata, msg):
        status = PayloadCodec.decode_status(msg.topic, msg.payload)
        if status is None:
            if msg.payload.strip() != b'0':     # "0" (knop los) is normaal
                self.log.warning(f'HMI-bericht genegeerd: {msg.topic} = {msg.payload!r}')
        else:
            self._status = status
