#!/usr/bin/env python3
"""ROS-schil van de patrouille.

Knoopt de ROS-adapters (patrol/ros/) aan de ROS-vrije PatrolController
(patrol/controller.py) en vertaalt topics naar commando's. Alle logica —
toestanden, escalatieladder, timeouts — staat in het package `patrol`.

Instellingen: config/patrol_params.yaml (tijden, afstanden)
              config/patrol_routes.yaml (routes + start-topics)
"""
import math
import signal

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from patrol import (
    HmiStatePublisher, IndicatorPolicy, PatrolController, SafetyMonitor, TransitionLogger)
from patrol.ros import Nav2Navigator, RobotSignals, RosScheduler, load_config

SAFETY_CHECK_PERIOD = 0.5   # s


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')
        config = load_config(self)
        log = self.get_logger()

        scheduler = RosScheduler(self)
        signals = RobotSignals(self, scheduler, config.cmd_vel_topic)
        self._navigator = Nav2Navigator(self, config.server_timeout_sec)

        self.controller = PatrolController(config, self._navigator, signals, scheduler, log)
        self.controller.add_listener(TransitionLogger(log))
        self.controller.add_listener(HmiStatePublisher(signals))
        self.controller.add_listener(IndicatorPolicy(signals))

        for route in config.routes:
            self.create_subscription(
                Bool, route.start_topic,
                lambda msg, rid=route.route_id: msg.data and self.controller.start(rid), 10)
            log.info(f'Route {route.route_id}: {len(route.waypoints)} waypoints, '
                     f'start via {route.start_topic}')
        self.create_subscription(
            Bool, '/stop_patrol', lambda msg: msg.data and self.controller.stop(), 10)

        if config.safety_monitor_enabled:
            monitor = SafetyMonitor(self.controller)
            self.controller.add_listener(monitor)
            self.create_subscription(
                Odometry, '/odom', lambda msg: monitor.on_motion(_is_moving(msg)), 10)
            self.create_timer(SAFETY_CHECK_PERIOD, monitor.check)
        else:
            log.info('EZ-Wheel veiligheidsstop-detectie uit (safety_monitor_enabled=false)')

        self._navigator.select_planner(config.planner_id)
        self.controller.begin()
        log.info('PatrolNode klaar — wacht op startsignaal')

    def shutdown_cleanly(self):
        self._navigator.shutdown()


def _is_moving(msg: Odometry) -> bool:
    twist = msg.twist.twist
    speed = math.hypot(twist.linear.x, twist.linear.y)
    return speed > 0.01 or abs(twist.angular.z) > 0.03


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    node = PatrolNode()
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanly()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
