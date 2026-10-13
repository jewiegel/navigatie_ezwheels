#!/usr/bin/env python3

import math
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.declare_parameter('behavior_tree', '')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.done = False

    def send_goal(self, x, y, yaw):
        self.get_logger().info('Wachten op nav2 server')
        self._client.wait_for_server()

        # Het doel bouwen: een positie + oriëntatie in het 'map' frame
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Hoek (yaw) omzetten naar een quaternion (alleen rotatie om z-as)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        bt = self.get_parameter('behavior_tree').value
        if bt:
            if not os.path.isabs(bt):
                bt = os.path.join(get_package_share_directory('my_bot'), 'behavior_trees', bt)
            goal.behavior_tree = bt
            self.get_logger().info(f'Loaded behavior tree: {bt}')

        self.get_logger().info(f'Doel verstuurd: x={x}, y={y}, yaw={yaw:.2f}')

        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        # Nav2 laat eerst weten of het doel geaccepteerd wordt
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Doel geweigerd door Nav2')
            self.done = True
            return
        self.get_logger().info('Doel geaccepteerd, robot rijdt...')
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        # Wordt herhaaldelijk aangeroepen tijdens het rijden
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Resterende afstand: {fb.distance_remaining:.2f} m',
            throttle_duration_sec=1.0)

    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Doel bereikt!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigatie afgebroken (bijv. geen pad gevonden)')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigatie geannuleerd')
        self.done = True

def main():
    rclpy.init()
    node = GoalSender()
    node.send_goal(8.0, 6.5, 0.0)

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()