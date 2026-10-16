#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.declare_parameter('behavior_tree', '')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Eigen topic: op /goal_pose luistert bt_navigator zelf al (met de standaardboom)
        self.create_subscription(PoseStamped, 'goal_pose_bt', self.send_goal, 10)
        self.get_logger().info('Klaar: stuur doelen met 2D Goal Pose in RViz (/goal_pose_bt)')

    def send_goal(self, pose):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server niet bereikbaar, doel genegeerd')
            return

        # Een nieuw doel vervangt in Nav2 automatisch het lopende doel
        goal = NavigateToPose.Goal()
        goal.pose = pose
        # Tijd 0 = nieuwste TF. Met de kliktijd faalt het herplannen zodra die uit de TF-buffer valt.
        goal.pose.header.stamp = Time().to_msg()

        bt = self.get_parameter('behavior_tree').value
        if bt:
            if not os.path.isabs(bt):
                bt = os.path.join(get_package_share_directory('my_bot'), 'behavior_trees', bt)
            goal.behavior_tree = bt
            self.get_logger().info(f'Loaded behavior tree: {bt}')

        p = pose.pose.position
        self.get_logger().info(f'Doel verstuurd: x={p.x:.2f}, y={p.y:.2f}')

        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        # Nav2 laat eerst weten of het doel geaccepteerd wordt
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Doel geweigerd door Nav2')
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
            self.get_logger().warn('Navigatie geannuleerd (of vervangen door nieuw doel)')

def main():
    rclpy.init()
    node = GoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()