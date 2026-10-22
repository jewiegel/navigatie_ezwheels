#!/usr/bin/env python3

from typing import Optional


class CommandWatchDog:
    """Verlopen als feed() langer dan timeout_s geleden is (of nog nooit)."""

    def __init__(self, timeout_s: float):
        self.timeout_s = timeout_s
        self._last: Optional[float] = None

    def feed(self, now: float) -> None:
        self._last = now

    def expired(self, now: float) -> bool:
        return self._last is None or now - self._last > self.timeout_s
