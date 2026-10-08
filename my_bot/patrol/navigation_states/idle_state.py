from .navigation_state import NavigationState


class IdleState(NavigationState):
    name = 'idle'
    can_start = True
