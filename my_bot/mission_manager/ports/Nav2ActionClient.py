#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

from .NavigationClient import NavigationClient


class Nav2ActionClient(NavigationClient):
    """Stuurt een route als één NavigateThroughPoses-doel naar Nav2.
    Het blokkade-herstel zit in de behavior tree, niet hier."""

    def __init__(self, node, on_result, callback_group=None, frame_id: str = 'map'):
        self._node = node
        self._on_result = on_result   # krijgt de get_result-future
        self._frame_id = frame_id
        self._client = ActionClient(
            node, NavigateThroughPoses, 'navigate_through_poses', callback_group=callback_group)
        self._seq = 0
        self._goal_handle = None

    def send_goal(self, waypoints):
        self.cancel()
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._to_pose(wp) for wp in waypoints]
        if not self._client.server_is_ready():
            # Een doel naar een afwezige server krijgt nooit antwoord -> direct als mislukt melden.
            self._node.get_logger().error('Nav2 navigate_through_poses niet beschikbaar')
            self._on_result(None)
            return
        self._seq += 1
        seq = self._seq
        self._client.send_goal_async(goal).add_done_callback(
            lambda f: self._on_goal_response(f, seq))

    def cancel(self):
        self._seq += 1   # late callbacks van het oude doel vervallen
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self._seq:
            # Al geannuleerd of vervangen voordat Nav2 antwoordde: niet laten doorrijden.
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._node.get_logger().error('Nav2 heeft het doel geweigerd')
            self._on_result(None)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result_done(f, seq))

    def _on_result_done(self, future, seq):
        if seq != self._seq:
            return
        self._goal_handle = None
        self._on_result(future)

    def _to_pose(self, wp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        # stamp blijft 0 = nieuwste TF; een echte tijd faalt zodra die uit de TF-buffer valt
        pose.pose.position.x = float(wp.x)
        pose.pose.position.y = float(wp.y)
        pose.pose.orientation.z = math.sin(wp.yaw / 2.0)
        pose.pose.orientation.w = math.cos(wp.yaw / 2.0)
        return pose
