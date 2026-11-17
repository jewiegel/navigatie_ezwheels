#!/usr/bin/env python3

import logging
import queue

import paho.mqtt.client as mqtt


class MqttBackend:
    """MQTT-verbinding: verbindt op de achtergrond en opnieuw na een onderbreking (paho
    loop-thread). Ontvangen berichten wachten in een rij tot receive() (ROS-thread)."""

    def __init__(self, host: str, port: int, topics, logger=None):
        self.host, self.port, self.topics = host, port, list(topics)
        self.log = logger or logging.getLogger('hmi_bridge')
        self._inbox = queue.SimpleQueue()
        # paho 2.x wil een callback-API-versie; VERSION1 = dezelfde callbacks als paho 1.x
        kwargs = ({'callback_api_version': mqtt.CallbackAPIVersion.VERSION1}
                  if hasattr(mqtt, 'CallbackAPIVersion') else {})
        self._client = mqtt.Client(client_id='hmi_bridge', **kwargs)
        self._client.on_connect = self._on_connect
        self._client.on_message = lambda client, userdata, msg: self._inbox.put(
            (msg.topic, msg.payload))

    def connect(self):
        self._client.connect_async(self.host, self.port, keepalive=10)
        self._client.loop_start()

    def disconnect(self):
        self._client.disconnect()
        self._client.loop_stop()

    def is_connected(self) -> bool:
        return self._client.is_connected()

    def publish(self, topic: str, payload: bytes):
        self._client.publish(topic, payload)

    def receive(self):
        """Alle (topic, payload) die sinds de vorige keer binnen zijn."""
        out = []
        while not self._inbox.empty():
            out.append(self._inbox.get())
        return out

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.log.error(f'MQTT-verbinding geweigerd (rc={rc})')
            return
        self.log.info(f'MQTT verbonden met {self.host}:{self.port}')
        for topic in self.topics:     # opnieuw na elke (her)verbinding
            client.subscribe(topic)
