#!/usr/bin/env python3
"""TurnDetectorNode: herkent bochten op tijd, voor de signal_manager (knipperlichten).

Interface:
  sub  plan  (nav_msgs/Path)            gepland pad van Nav2, begint bij de robot
  sub  odom  (nav_msgs/Odometry)        gemeten draaisnelheid
  sub  route_preview (geometry_msgs/PoseArray)  gestarte route (route_manager): bochten
       bij waypoints, want Nav2 plant per waypoint en het pad eindigt daar
  pub  turn  (my_bot_interfaces/Turn)   komende bocht + huidige draai, met rate_hz

Parameters:
  lookahead_m     hoe ver vooruit langs het pad naar bochten gezocht wordt   (3.0)
  turn_angle_deg  minimale richtingsverandering die als bocht telt            (30.0)
  turn_rate       draaisnelheid (rad/s) waarboven de robot "draait"          (0.25)
  plan_timeout_s  ouder pad wordt genegeerd (Nav2 plant elke seconde opnieuw) (2.0)
  rate_hz         publicatiefrequentie van turn                               (5.0)
"""
import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from my_bot_interfaces.msg import Turn
from turn_detection import LEFT, NONE, RIGHT, extend_with_route, find_turn, turning_direction

NAMES = {NONE: 'geen', LEFT: 'LINKS', RIGHT: 'RECHTS'}


class TurnDetectorNode(Node):

    def __init__(self):
        super().__init__('turn_detector')
        self.lookahead_m = self.declare_parameter('lookahead_m', 3.0).value
        self.turn_angle = math.radians(self.declare_parameter('turn_angle_deg', 30.0).value)
        self.turn_rate = self.declare_parameter('turn_rate', 0.25).value
        self.plan_timeout_s = self.declare_parameter('plan_timeout_s', 2.0).value
        rate_hz = self.declare_parameter('rate_hz', 5.0).value

        self._upcoming = None        # UpcomingTurn uit het laatste pad
        self._plan_time = None       # ontvangsttijd van het laatste pad
        self._turning = NONE
        self._route = []             # waypoints (x, y) van de gestarte route
        self._last_logged = (NONE, NONE)

        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(
            PoseArray, 'route_preview', self.on_route,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.turn_pub = self.create_publisher(Turn, 'turn', 10)
        self.create_timer(1.0 / rate_hz, self.publish_turn)
        self.get_logger().info(
            f'TurnDetector: bochten >= {math.degrees(self.turn_angle):.0f}° '
            f'binnen {self.lookahead_m:.1f} m')

    def on_plan(self, msg: Path):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        points = extend_with_route(points, self._route)
        self._upcoming = find_turn(points, self.lookahead_m, self.turn_angle)
        self._plan_time = self.get_clock().now()

    def on_route(self, msg: PoseArray):
        self._route = [(p.position.x, p.position.y) for p in msg.poses]

    def on_odom(self, msg: Odometry):
        self._turning = turning_direction(msg.twist.twist.angular.z, self.turn_rate)

    def publish_turn(self):
        msg = Turn(direction=Turn.NONE, turning=self._turning)
        # Na het laatste doel komt er geen nieuw pad meer; een oud pad meldt dan een
        # bocht die er niet meer is.
        fresh = (self._plan_time is not None and
                 (self.get_clock().now() - self._plan_time).nanoseconds / 1e9
                 < self.plan_timeout_s)
        if fresh and self._upcoming is not None:
            # shortcut: afstand geldt voor het moment van plannen (max. ~1 s rijden oud);
            # trek de gereden afstand af als de signal_manager preciezer moet zijn.
            msg.direction = self._upcoming.direction
            msg.distance = float(self._upcoming.distance)
            msg.angle = float(self._upcoming.angle)
        self.turn_pub.publish(msg)

        # Registreren: alleen loggen als er iets verandert, niet elke cyclus.
        state = (msg.direction, msg.turning)
        if state != self._last_logged:
            self._last_logged = state
            ahead = (f'bocht {NAMES[msg.direction]} over {msg.distance:.1f} m '
                     f'({math.degrees(msg.angle):+.0f}°)' if msg.direction else 'geen bocht')
            self.get_logger().info(f'{ahead}; draait nu: {NAMES[msg.turning]}')


def main():
    rclpy.init()
    node = TurnDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
