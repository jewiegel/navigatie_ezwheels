#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Optional

from hmi_bridge.HMIMessages import HMIStatus, SignalCommand


class SignalBackend(ABC):
    """Verbinding met de HMI. latest_status geeft per knopdruk een nieuw object, zodat de
    aanroeper met `is` kan zien of er een nieuwe druk is."""

    @abstractmethod
    def connect(self) -> None:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @abstractmethod
    def send(self, cmd: SignalCommand) -> None:
        pass

    @abstractmethod
    def latest_status(self) -> Optional[HMIStatus]:
        pass
