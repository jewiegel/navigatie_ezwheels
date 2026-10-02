from .navigation_state import NavigationState


class StoppedState(NavigationState):
    name = 'gestopt'
    can_start = True

    def on_enter(self):
        self.ctl.navigator.cancel()
        self.ctl.signals.stop_wheels()
