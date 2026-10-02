from geometry_msgs.msg import Twist

from .navigation_state import NavigationState


class CompletedState(NavigationState):
    name = 'voltooid'

    def on_enter(self):
        # Geen herstart: stilstaan en gevarenlichten aan (indicator_node blijft knipperen).
        self.ctx._cmdvel_pub.publish(Twist())
        self.ctx._set_indicator('gevaar')

    def can_start(self):
        return True
