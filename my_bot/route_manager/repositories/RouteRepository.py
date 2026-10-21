#!/usr/bin/env python3

from abc import ABC, abstractmethod

from ..Route import Route


class RouteRepository(ABC):
    """Opslag van routes op naam. load() geeft KeyError bij een onbekende route."""

    @abstractmethod
    def list(self) -> list[str]:
        pass

    @abstractmethod
    def load(self, name: str) -> Route:
        pass

    @abstractmethod
    def save(self, route: Route) -> None:
        pass

    @abstractmethod
    def delete(self, name: str) -> None:
        pass
