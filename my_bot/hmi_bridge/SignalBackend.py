from abc import ABC, abstractmethod

class SignalBackend(ABC):

    @abstractmethod
    def connect():
        pass

    @abstractmethod
    def disconnect():
        pass

    @abstractmethod
    def is_connected() -> bool:
        pass

    @abstractmethod
    def send (cmd: SignalCommand, seq: int):
        pass

    @abstractmethod
    def latest_status() -> HMIStatus:
        pass