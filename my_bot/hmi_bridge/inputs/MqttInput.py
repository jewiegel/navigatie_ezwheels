#!/usr/bin/env python3

from abc import ABC, abstractmethod


class MqttInput(ABC):
    """Eén MQTT-topic naar de robot (knop van de HMI, antwoord van EZ-Way).

    De HMIBridgeNode abonneert op `topic` en geeft elk bericht aan on_message (in de
    ROS-thread, dus ROS-aanroepen mogen daar)."""

    topic: str = ''

    def setup(self, ctx):
        """ctx = de HMIBridgeNode: maak hier ROS-clients/publishers en lees parameters."""
        self.ctx = ctx

    @abstractmethod
    def on_message(self, payload: bytes) -> None:
        pass


def pressed(payload: bytes) -> bool:
    """Knop van de HMI: "1" = ingedrukt, "0" = losgelaten (doet niets)."""
    return payload.strip() == b'1'
