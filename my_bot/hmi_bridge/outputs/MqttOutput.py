#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Dict


class MqttOutput(ABC):
    """Eén of meer MQTT-topics van de robot naar de HMI.

    De HMIBridgeNode vraagt elke 50 ms payloads(now) op en stuurt een topic als het
    veranderd is, en alles opnieuw elke heartbeat_s."""

    def setup(self, ctx):
        """ctx = de HMIBridgeNode: abonneer hier op ROS-topics en lees parameters."""
        self.ctx = ctx

    @abstractmethod
    def payloads(self, now: float) -> Dict[str, bytes]:
        """Payload per MQTT-topic; een topic weglaten = (nu) niets te sturen."""


def bit(on: bool) -> bytes:
    """Uitgang van de HMI: "1" = aan, "0" = uit."""
    return b'1' if on else b'0'
