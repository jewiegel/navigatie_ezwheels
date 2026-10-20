#!/usr/bin/env python3

from abc import ABC, abstractmethod

class NavigationClient(ABC):

    @abstractmethod
    def send_goal():
        pass

    @abstractmethod
    def cancel():
        pass