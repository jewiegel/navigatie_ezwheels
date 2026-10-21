#!/usr/bin/env python3
"""TurnDetectorNode: herkent bochten op tijd, voor de signal_manager (knipperlichten).

Interface:
  sub  plan  (nav_msgs/Path)            gepland pad van Nav2, begint bij de robot
  sub  odom  (nav_msgs/Odometry)        gemeten draaisnelheid
  sub  route_preview (geometry_msgs/PoseArray)  gestarte route (route_manager): bochten
       bij waypoints, want Nav2 plant per waypoint en het pad eindigt daar
  pub  turn  (my_bot_interfaces/Turn)   komende bocht + huidige draai, met rate_hz
  pub  turn_markers (visualization_msgs/MarkerArray)  voor RViz/Foxglove: knipperlichten
       op de robot en het bochtpunt op het pad

Parameters:
  lookahead_m     hoe ver vooruit langs het pad naar bochten gezocht wordt   (3.0)
  turn_angle_deg  minimale richtingsverandering die als bocht telt            (30.0)
  turn_rate       draaisnelheid (rad/s) waarboven de robot "draait"          (0.25)
  plan_timeout_s  ouder pad wordt genegeerd (Nav2 plant elke seconde opnieuw) (2.0)
  rate_hz         publicatiefrequentie van turn                               (5.0)
"""
import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from my_bot_interfaces.msg import Turn
from turn_detection import LEFT, NONE, RIGHT, extend_with_route, find_turn, turning_direction

NAMES = {NONE: 'geen', LEFT: 'LINKS', RIGHT: 'RECHTS'}

# Knipperlichten op de voorhoeken van de chassisdoos (0,5 x 0,4 m) in base_link
LIGHT_POSITIONS = {LEFT: (0.25, 0.2, 0.12), RIGHT: (0.25, -0.2, 0.12)}
ORANGE = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
DARK = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.6)


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
        self._plan_frame = 'map'
        self._turning = NONE
        self._route = []             # waypoints (x, y) van de gestarte route
        self._last_logged = (NONE, NONE)

        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(
            PoseArray, 'route_preview', self.on_route,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.turn_pub = self.create_publisher(Turn, 'turn', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'turn_markers', 10)
        self.create_timer(1.0 / rate_hz, self.publish_turn)
        self.get_logger().info(
            f'TurnDetector: bochten >= {math.degrees(self.turn_angle):.0f}° '
            f'binnen {self.lookahead_m:.1f} m')

    def on_plan(self, msg: Path):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        points = extend_with_route(points, self._route)
        self._upcoming = find_turn(points, self.lookahead_m, self.turn_angle)
        self._plan_time = self.get_clock().now()
        self._plan_frame = msg.header.frame_id or 'map'

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
        self.marker_pub.publish(self._markers(msg))

        # Registreren: alleen loggen als er iets verandert, niet elke cyclus.
        state = (msg.direction, msg.turning)
        if state != self._last_logged:
            self._last_logged = state
            ahead = (f'bocht {NAMES[msg.direction]} over {msg.distance:.1f} m '
                     f'({math.degrees(msg.angle):+.0f}°)' if msg.direction else 'geen bocht')
            self.get_logger().info(f'{ahead}; draait nu: {NAMES[msg.turning]}')

    # ── Visualisatie ──────────────────────────────────────────────────────────

    def _markers(self, turn: Turn) -> MarkerArray:
        """Knipperlicht aan de kant van de bocht (komend of nu), bolletje + tekst op
        het bochtpunt. Alleen voor RViz/Foxglove; de signal_manager gebruikt turn."""
        now = self.get_clock().now()
        blink_on = (now.nanoseconds // 500_000_000) % 2 == 0   # 1 Hz knipperen
        side = turn.turning or turn.direction
        markers = []
        for i, (direction, (x, y, z)) in enumerate(LIGHT_POSITIONS.items()):
            light = self._marker('turn_lights', i, 'base_link', Marker.SPHERE, 0.1)
            light.pose.position.x, light.pose.position.y, light.pose.position.z = x, y, z
            light.color = ORANGE if direction == side and blink_on else DARK
            markers.append(light)

        point = self._marker('turn_point', 0, self._plan_frame, Marker.SPHERE, 0.3)
        label = self._marker('turn_point', 1, self._plan_frame, Marker.TEXT_VIEW_FACING, 0.3)
        if turn.direction:
            x, y = self._upcoming.point
            point.pose.position.x, point.pose.position.y = x, y
            point.color = ORANGE
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.6
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'{NAMES[turn.direction]} {turn.distance:.1f} m'
        else:
            point.action = label.action = Marker.DELETE
        return MarkerArray(markers=markers + [point, label])

    def _marker(self, ns, marker_id, frame, marker_type, size) -> Marker:
        m = Marker(ns=ns, id=marker_id, type=marker_type, action=Marker.ADD)
        m.header.frame_id = frame     # stamp 0 = nieuwste TF
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = size
        m.lifetime = Duration(sec=1)  # verdwijnt vanzelf als de node stopt
        return m


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
