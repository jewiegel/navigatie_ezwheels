from action_msgs.msg import GoalStatus

from .navigation_state import NavigationState
from .waiting_state import WaitingState
from .completed_state import CompletedState


class DrivingState(NavigationState):
    name = 'rijdend'

    def on_enter(self):
        self.ctx._driving_since = self.ctx.get_clock().now()

    def on_leave(self):
        self.ctx._driving_since = None

    def can_stop(self):
        return True

    def on_nav_result(self, status):
        ctx = self.ctx
        if ctx._backing_up:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            ctx.get_logger().info('Volledige route voltooid — robot stopt, gevarenlichten aan.')
            ctx._last_successful_idx = len(ctx._waypoints) - 1
            ctx._trying_alternative  = False
            ctx._blockage_level      = 0
            ctx._set_state(CompletedState)
        else:
            ctx.get_logger().warn(
                f'Nav2 route mislukt — status: {status} '
                f'(4=SUCCEEDED, 5=CANCELED, 6=ABORTED)')
            ctx._on_blocked()

    def on_tick(self):
        """Per-waypoint timeout: _driving_since wordt gereset via _on_feedback
        telkens als een waypoint gepasseerd wordt."""
        ctx = self.ctx
        if ctx._backing_up or ctx._driving_since is None:
            return
        timeout = ctx._nav_timeout_sec
        elapsed = (ctx.get_clock().now() - ctx._driving_since).nanoseconds / 1e9
        if elapsed > timeout:
            ctx.get_logger().error(
                f'Nav2 timeout na {timeout}s — geen voortgang bij '
                f'waypoint {ctx._current_index + 1}')
            ctx._driving_since = None
            ctx._on_blocked()

    def published_name(self):
        # EZ-Wheel veiligheidsstop tijdens het rijden → naar buiten toe 'wachten'
        return WaitingState.name if self.ctx._safety_stopped else self.name
