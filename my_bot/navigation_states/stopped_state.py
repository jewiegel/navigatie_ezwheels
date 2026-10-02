from .navigation_state import NavigationState


class StoppedState(NavigationState):
    name = 'gestopt'

    def on_enter(self):
        self.ctx._reset_timers_and_flags()
        self.ctx._cancel_goal()

    def can_start(self):
        return True
