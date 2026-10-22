from abc import ABC, abstractmethod

class SignalSource(ABC):

    @abstractmethod
    def update(msg, now):
        pass

    @abstractmethod
    def is_fresh(now) -> bool:
        pass