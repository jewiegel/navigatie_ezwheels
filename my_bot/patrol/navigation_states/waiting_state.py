from .navigation_state import NavigationState


class WaitingState(NavigationState):
    """Stilstaan gedurende `seconds` en daarna `then()` aanroepen.

    Gebruikt voor het wachten op vrije doorgang én voor de korte herplan-pauze.
    """
    name = 'wachten'
    can_stop = True

    def __init__(self, ctl, seconds, then):
        super().__init__(ctl)
        self._seconds = seconds
        self._then = then

    def on_enter(self):
        self.ctl.signals.stop_wheels()
        self.call_later(self._seconds, self._then)
