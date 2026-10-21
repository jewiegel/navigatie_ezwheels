#!/usr/bin/env python3

from abc import ABC, abstractmethod


class UINotifier(ABC):

    @abstractmethod
    def notify(self, message: str) -> None:
        pass
