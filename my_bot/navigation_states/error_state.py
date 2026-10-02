from .navigation_state import NavigationState


class ErrorState(NavigationState):
    name = 'fout'

    def can_start(self):
        return True
