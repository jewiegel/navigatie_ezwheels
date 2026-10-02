from geometry_msgs.msg import Twist

from .navigation_state import NavigationState


class WaitingState(NavigationState):
    """Robot staat stil: wachten op vrije doorgang, achteruit of herplan-pauze."""
    name = 'wachten'

    def on_enter(self):
        self.ctx._cmdvel_pub.publish(Twist())   # echt stilstaan

    def can_stop(self):
        return True
