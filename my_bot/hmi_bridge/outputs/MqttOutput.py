from abc import ABC, abstractmethod

class MqttOutput(ABC):

    def setup(self, ctx):
        pass

    @abstractmethod
    def payloads(self, now):
        pass