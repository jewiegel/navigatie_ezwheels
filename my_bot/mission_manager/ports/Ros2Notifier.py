#!/usr/bin/env python3

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .UINotifier import UINotifier


class Ros2Notifier(UINotifier):
    """Publiceert missie-status op /mission_status (voor HMI / mqtt-bridge / dashboard).

    transient_local: een later verbonden dashboard ziet meteen de huidige toestand.
    """

    def __init__(self, node, topic: str = 'mission_status'):
        self._pub = node.create_publisher(
            String, topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def notify(self, message: str):
        self._pub.publish(String(data=message))
