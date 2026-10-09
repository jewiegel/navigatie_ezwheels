#!/usr/bin/env python3
"""
Rijdt de robot in rondjes voor de batterijtest.
Stuurt een vaste lineaire + draaisnelheid naar /cmd_vel; daardoor rijdt de
robot een cirkel met straal = linear_speed / angular_speed. Geen kaart of
Nav2 nodig.

Starten / stoppen:
    ros2 topic pub --once /battery_test/start std_msgs/msg/Bool "{data: true}"
    ros2 topic pub --once /battery_test/start std_msgs/msg/Bool "{data: false}"
"""
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CircleDrivingNode(Node):

    def __init__(self):
        super().__init__('circle_driving_node')

        # Parameters — aan te passen met -p naam:=waarde
        # 0.3 m/s / 0.4 rad/s = straal 0.75 m: past ruim in de testruimte van 3x3 m
        self.declare_parameter('linear_speed', 0.25)     # m/s vooruit
        self.declare_parameter('angular_speed', 0.5)    # rad/s; positief = linksom, negatief = rechtsom
        self.declare_parameter('autostart', False)      # true = meteen rijden, zonder start-topic
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('rate', 10.0)            # Hz waarmee het commando herhaald wordt

        self._linear  = self.get_parameter('linear_speed').value
        self._angular = self.get_parameter('angular_speed').value
        self._driving = self.get_parameter('autostart').value

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        rate          = self.get_parameter('rate').value

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Bool, '/battery_test/start', self._on_start, 10)

        # Commando continu herhalen: veel robotbases stoppen vanzelf als er
        # even geen /cmd_vel binnenkomt (watchdog)
        self.create_timer(1.0 / rate, self._publish_cmd)

        radius = abs(self._linear / self._angular) if self._angular else float('inf')
        self.get_logger().info(
            f'CircleDrivingNode gestart — {self._linear} m/s, {self._angular} rad/s '
            f'(straal {radius:.2f} m) op {cmd_vel_topic}. '
            f'{"Rijdt direct (autostart)." if self._driving else "Wacht op /battery_test/start."}')


    def _on_start(self, msg: Bool):
        """true = rondjes rijden, false = stoppen."""
        if msg.data == self._driving:
            return
        self._driving = msg.data
        if self._driving:
            self.get_logger().info('Start: rondjes rijden')
        else:
            self.get_logger().info('Stop: robot staat stil')
            self.stop()


    def _publish_cmd(self):
        # Alleen publiceren tijdens het rijden; zo blijft /cmd_vel vrij voor
        # controller/toetsenbord wanneer de test niet loopt
        if not self._driving:
            return
        twist = Twist()
        twist.linear.x  = self._linear
        twist.angular.z = self._angular
        self._cmd_pub.publish(twist)


    def stop(self):
        """Eén nulcommando sturen. Nodig omdat Gazebo's DiffDrive het laatste
        commando blijft uitvoeren; zonder dit rijdt hij door na stoppen."""
        self._cmd_pub.publish(Twist())


def main():
    # Eigen Ctrl+C-afhandeling: anders is ROS al afgesloten voordat we het
    # stopcommando kunnen sturen
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CircleDrivingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.get_logger().info('Afgesloten — stopcommando verstuurd')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
