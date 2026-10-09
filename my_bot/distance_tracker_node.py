#!/usr/bin/env python3
"""
Houdt de afgelegde afstand en de tijd sinds de start bij (batterijtest).

Leest de wielodometrie (/odom) en telt de gereden afstand op. Publiceert:
    /distance_travelled  (std_msgs/Float64)  meter
    /time_since_start    (std_msgs/Float64)  seconden

De tijd start bij het eerste /odom-bericht. Opnieuw beginnen bij nul:
    ros2 topic pub --once /distance_tracker/reset std_msgs/msg/Bool "{data: true}"
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


class DistanceTrackerNode(Node):

    def __init__(self):
        super().__init__('distance_tracker_node')

        # Parameters — aan te passen met -p naam:=waarde
        self.declare_parameter('min_step', 0.005)       # m — kleinere verplaatsing telt (nog) niet: filtert ruis bij stilstand
        self.declare_parameter('max_step', 0.5)         # m — grotere sprong per bericht = odom-reset, niet meetellen
        self.declare_parameter('publish_rate', 1.0)     # Hz voor /distance_travelled en /time_since_start
        self.declare_parameter('log_interval', 10.0)    # s tussen statusregels in de terminal

        self._min_step = self.get_parameter('min_step').value
        self._max_step = self.get_parameter('max_step').value
        publish_rate   = self.get_parameter('publish_rate').value
        log_interval   = self.get_parameter('log_interval').value

        self._distance   = 0.0    # opgetelde afstand in m
        self._last_xy    = None   # laatst meegetelde positie (x, y)
        self._start_time = None   # tijdstip eerste /odom; None = nog niet gestart

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/distance_tracker/reset', self._on_reset, 10)

        self._distance_pub = self.create_publisher(Float64, '/distance_travelled', 10)
        self._time_pub     = self.create_publisher(Float64, '/time_since_start', 10)

        self.create_timer(1.0 / publish_rate, self._publish)
        self.create_timer(log_interval, self._log_status)

        self.get_logger().info('DistanceTrackerNode gestart — wacht op /odom')


    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Eerste bericht (of na reset): beginpunt en starttijd vastleggen
        if self._start_time is None:
            self._start_time = self.get_clock().now()
            self._last_xy = (x, y)
            self.get_logger().info('Eerste /odom ontvangen — meting gestart')
            return

        step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])

        # Kleine stap: last_xy NIET bijwerken, zodat langzaam rijden wel
        # optelt zodra de drempel bereikt is, maar trillen op de plek niet
        if step < self._min_step:
            return

        if step > self._max_step:
            self.get_logger().warn(f'Sprong van {step:.2f} m in /odom genegeerd (odom-reset?)')
        else:
            self._distance += step

        self._last_xy = (x, y)


    def _on_reset(self, msg: Bool):
        if not msg.data:
            return
        self._distance   = 0.0
        self._last_xy    = None
        self._start_time = None   # start opnieuw bij het volgende /odom-bericht
        self.get_logger().info('Reset — afstand en tijd op nul')


    def _elapsed(self) -> float:
        """Seconden sinds de start; 0 zolang er nog geen /odom binnen is.
        Met use_sim_time:=true is dit simulatietijd."""
        if self._start_time is None:
            return 0.0
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9


    def _publish(self):
        self._distance_pub.publish(Float64(data=self._distance))
        self._time_pub.publish(Float64(data=self._elapsed()))


    def _log_status(self):
        if self._start_time is None:
            return
        self.get_logger().info(self.summary())


    def summary(self) -> str:
        """Bv. 'Afstand: 123.45 m | Tijd: 00:12:34 | Gem.: 0.16 m/s'"""
        elapsed = self._elapsed()
        h, rest = divmod(int(elapsed), 3600)
        m, s    = divmod(rest, 60)
        avg     = self._distance / elapsed if elapsed > 0 else 0.0
        return f'Afstand: {self._distance:.2f} m | Tijd: {h:02d}:{m:02d}:{s:02d} | Gem.: {avg:.2f} m/s'


def main():
    # Eigen Ctrl+C-afhandeling, zodat de eindstand nog gelogd kan worden
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DistanceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Eindstand — {node.summary()}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
