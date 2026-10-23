#!/usr/bin/env python3

from typing import Optional

from hmi_bridge.HMIMessages import HMIStatus, SignalCommand
from hmi_bridge.SignalBackend import SignalBackend


class InMemoryBackend(SignalBackend):
    """Zonder broker, voor tests en de simulatie: onthoudt wat verstuurd is; een knopdruk
    zet je met receive()."""

    def __init__(self):
        self.connected = False
        self.sent: list = []        # SignalCommand's
        self._status: Optional[HMIStatus] = None

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def is_connected(self):
        return self.connected

    def send(self, cmd: SignalCommand):
        self.sent.append(cmd)

    def latest_status(self):
        return self._status

    def receive(self, status: HMIStatus):
        self._status = status
