#!/usr/bin/env python3

from action_msgs.msg import GoalStatus

from .TaskExecutor import TaskExecutor


class Ros2TaskExecutor(TaskExecutor):
    """Voert een waypoint-actie uit als ROS 2-actiedoel via de TaskRegistry.
    on_result(succeeded: bool) volgt altijd precies één keer per execute()."""

    def __init__(self, node, registry, on_result):
        self._node = node
        self._registry = registry
        self._on_result = on_result
        self._seq = 0
        self._goal_handle = None

    def execute(self, task):
        self.cancel()
        log = self._node.get_logger()
        try:
            client, make_goal = self._registry.get(task.type)
            goal = make_goal(task.params)
        except KeyError:
            log.error(f'Onbekende actie "{task.type}"; bekend: {", ".join(self._registry.names())}')
            return self._on_result(False)
        except Exception as e:  # o.a. ValueError/TypeError door verkeerde params
            log.error(f'Actie "{task.type}": ongeldige params {task.params}: {e}')
            return self._on_result(False)
        if not client.server_is_ready():
            log.error(f'Action server voor "{task.type}" niet beschikbaar')
            return self._on_result(False)
        log.info(f'Actie "{task.type}" gestart {task.params}')
        self._seq += 1
        seq = self._seq
        client.send_goal_async(goal).add_done_callback(lambda f: self._on_goal_response(f, seq))

    def cancel(self):
        self._seq += 1   # late callbacks van het oude doel vervallen
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self._seq:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._node.get_logger().error('Actiedoel geweigerd')
            return self._on_result(False)
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result_done(f, seq))

    def _on_result_done(self, future, seq):
        if seq != self._seq:
            return
        self._goal_handle = None
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().warn(f'Actie mislukt (status {status})')
        self._on_result(status == GoalStatus.STATUS_SUCCEEDED)
