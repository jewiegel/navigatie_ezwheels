#!/usr/bin/env python3

class ActionSpecification():
    """Actie op een waypoint. type is een naam uit de TaskRegistry, params gaan naar het actiedoel.
    max_retries: zo vaak opnieuw proberen na mislukken; daarna rijdt de route toch door."""

    def __init__(self, type: str, params: dict = None, on_failure=None, max_retries: int = 1):
        self.type = type
        self.params = params or {}
        self.on_failure = on_failure   # nog niet gebruikt: de route gaat na een actie altijd door
        self.max_retries = max_retries
