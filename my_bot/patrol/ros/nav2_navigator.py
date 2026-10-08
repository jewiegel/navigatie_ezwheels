"""Adapter naar Nav2: action clients, cancel-services en controller-reset.

Elk doel krijgt een volgnummer. Callbacks van een doel dat niet meer het
actuele is (na cancel() of een nieuw doel) worden hier al weggefilterd, zodat
een laat CANCELED/ABORTED-resultaat nooit een escalatiestap kan triggeren.
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from std_msgs.msg import String


class Nav2Navigator:

    def __init__(self, node, server_timeout_sec: float = 5.0, frame_id: str = 'map'):
        self._node = node
        self._log = node.get_logger()
        self._server_timeout = server_timeout_sec
        self._frame_id = frame_id

        self._through_client = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
        self._to_client      = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._cancel_through = node.create_client(
            CancelGoal, '/navigate_through_poses/_action/cancel_goal')
        self._cancel_to      = node.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._ctrl_lifecycle = node.create_client(
            ChangeState, '/controller_server/change_state')

        planner_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._planner_pub = node.create_publisher(String, '/planner_selector', planner_qos)

        self._seq = 0
        self._goal_handle = None

    # ── Planner ───────────────────────────────────────────────────────────────

    def select_planner(self, planner_id: str):
        msg = String()
        msg.data = planner_id
        self._planner_pub.publish(msg)

    # ── Voorbereiden: zombie-doelen weg + controller_server resetten ──────────

    def prepare(self, on_ready):
        self.cancel()
        pending = [0]

        def _on_cancelled(future, name):
            try:
                n = len(future.result().goals_canceling)
                self._log.info(f'{name}: {n} zombie-doel(en) geannuleerd')
            except Exception as exc:
                self._log.warn(f'CancelGoal {name} fout: {exc}')
            pending[0] -= 1
            if pending[0] <= 0:
                self._reset_controller(on_ready)

        req = CancelGoal.Request()   # leeg verzoek = alle doelen annuleren
        for client, name in ((self._cancel_through, 'NavigateThroughPoses'),
                             (self._cancel_to, 'NavigateToPose')):
            if client.service_is_ready():
                pending[0] += 1
                client.call_async(req).add_done_callback(
                    lambda fut, name=name: _on_cancelled(fut, name))

        if pending[0] == 0:
            self._log.warn('Geen cancel-services beschikbaar — sla cancel over')
            self._reset_controller(on_ready)

    def _reset_controller(self, on_ready):
        if not self._ctrl_lifecycle.service_is_ready():
            self._log.warn('controller_server lifecycle-service niet bereikbaar — sla reset over')
            on_ready(False)
            return
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self._log.info('controller_server deactiveren om interne staat te wissen...')
        self._ctrl_lifecycle.call_async(req).add_done_callback(
            lambda fut: self._on_controller_reset(fut, on_ready))

    def _on_controller_reset(self, future, on_ready):
        try:
            if future.result().success:
                self._log.info('controller_server gedeactiveerd — lifecycle_manager herstart hem')
            else:
                self._log.warn('controller_server deactivate mislukt — toch doorgaan')
        except Exception as exc:
            self._log.warn(f'controller_server lifecycle fout: {exc}')
        on_ready(True)

    # ── Doelen sturen ─────────────────────────────────────────────────────────

    def drive_through(self, waypoints, on_accepted, on_rejected, on_feedback, on_result):
        if not self._through_client.wait_for_server(timeout_sec=self._server_timeout):
            return False
        seq = self._next_seq()
        stamp = self._node.get_clock().now().to_msg()
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._to_pose(wp, stamp) for wp in waypoints]

        def _feedback(msg):
            if seq == self._seq:
                on_feedback(msg.feedback.number_of_poses_remaining)

        future = self._through_client.send_goal_async(goal, feedback_callback=_feedback)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, seq, on_accepted, on_rejected, on_result))
        return True

    def drive_to(self, waypoint, on_rejected, on_result):
        if not self._to_client.wait_for_server(timeout_sec=self._server_timeout):
            return False
        seq = self._next_seq()
        goal = NavigateToPose.Goal()
        goal.pose = self._to_pose(waypoint, self._node.get_clock().now().to_msg())
        future = self._to_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, seq, None, on_rejected, on_result))
        return True

    def cancel(self):
        self._seq += 1   # alle late callbacks van het oude doel vervallen
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def shutdown(self):
        """Bij afsluiten: actief doel netjes annuleren (blokkeert max. 3 s)."""
        if self._goal_handle is None:
            return
        self._log.info('Afsluiten — annuleer actief Nav2-doel...')
        try:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        except Exception as exc:
            self._log.warn(f'Kon doel niet netjes annuleren: {exc}')
        finally:
            self._goal_handle = None

    # ── Intern ────────────────────────────────────────────────────────────────

    def _next_seq(self):
        self._seq += 1
        return self._seq

    def _on_goal_response(self, future, seq, on_accepted, on_rejected, on_result):
        if seq != self._seq:
            return   # verouderd doel — er is al een nieuwer doel gestuurd of geannuleerd
        handle = future.result()
        if not handle.accepted:
            on_rejected()
            return
        self._goal_handle = handle
        if on_accepted is not None:
            on_accepted()
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, seq, on_result))

    def _on_result(self, future, seq, on_result):
        if seq != self._seq:
            return
        self._goal_handle = None
        status = future.result().status
        on_result(status == GoalStatus.STATUS_SUCCEEDED, status)

    def _to_pose(self, waypoint, stamp):
        x, y, yaw = waypoint
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose
