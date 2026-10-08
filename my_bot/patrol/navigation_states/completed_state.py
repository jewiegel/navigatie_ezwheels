from .navigation_state import NavigationState


class CompletedState(NavigationState):
    """Route klaar: stilstaan (gevarenlichten via IndicatorPolicy)."""
    name = 'voltooid'
    can_start = True

    def on_enter(self):
        self.ctl.signals.stop_wheels()
