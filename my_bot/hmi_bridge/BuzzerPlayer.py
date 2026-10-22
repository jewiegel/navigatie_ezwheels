#!/usr/bin/env python3
"""Speelt een buzzerpatroon van de signal_manager af op een uitgang die alleen aan/uit kan."""

from typing import Optional

# Afwisselend aan/uit in seconden, beginnend met aan; daarna uit.
PATTERNS = {'uit': (), 'enkel': (0.3,), 'dubbel': (0.15, 0.15, 0.15)}
# Herhalend: (stappen, periode in s), tot er een ander patroon komt.
REPEATING = {'elke_5s': ((0.3,), 5.0)}


class BuzzerPlayer:

    def __init__(self):
        self._steps = ()
        self._period: Optional[float] = None
        self._start: Optional[float] = None

    def play(self, pattern: str, now: float) -> None:
        """ValueError bij een onbekend patroon; het vorige patroon stopt dan."""
        self._steps, self._period, self._start = (), None, now
        if pattern in REPEATING:
            self._steps, self._period = REPEATING[pattern]
        elif pattern in PATTERNS:
            self._steps = PATTERNS[pattern]
        else:
            raise ValueError(f'onbekend buzzerpatroon "{pattern}"')

    def is_on(self, now: float) -> bool:
        if self._start is None:
            return False
        t = now - self._start
        if self._period:
            t %= self._period
        for i, duration in enumerate(self._steps):
            if t < duration:
                return i % 2 == 0
            t -= duration
        return False
