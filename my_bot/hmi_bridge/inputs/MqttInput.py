#!/usr/bin/env python3

from abc import ABC, abstractmethod

class MqttInput(ABC):

    def __init__(self):
        self.topic

    def setup(self, ctx):
        pass

    @abstractmethod
    def on_message(self, payload):
        pass