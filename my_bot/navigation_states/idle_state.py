from .navigation_state import NavigationState


class IdleState(NavigationState):
    name = 'idle'

    def can_start(self):
        return True
