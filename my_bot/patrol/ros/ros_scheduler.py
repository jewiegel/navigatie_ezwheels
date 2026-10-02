"""Scheduler-adapter op basis van rclpy-timers en de node-klok."""


class _RosTimer:

    def __init__(self, node, period, fn, oneshot):
        self._fn = fn
        self._oneshot = oneshot
        self._timer = node.create_timer(period, self._fire)

    def _fire(self):
        if self._timer is None:
            return
        if self._oneshot:
            self.cancel()
        self._fn()

    def cancel(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None


class RosScheduler:

    def __init__(self, node):
        self._node = node

    def now(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def call_later(self, delay, fn):
        return _RosTimer(self._node, delay, fn, oneshot=True)

    def call_every(self, period, fn):
        return _RosTimer(self._node, period, fn, oneshot=False)
