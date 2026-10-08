from .navigation_state import NavigationState


class ErrorState(NavigationState):
    """Operator moet ingrijpen; een nieuw startsignaal mag."""
    name = 'fout'
    can_start = True

    def on_enter(self):
        self.ctl.navigator.cancel()
        self.ctl.signals.stop_wheels()
