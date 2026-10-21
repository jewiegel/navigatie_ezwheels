#!/usr/bin/env python3

from std_msgs.msg import String

from .UINotifier import UINotifier


class Ros2Notifier(UINotifier):
    """Publiceert missie-status op /mission_status (voor HMI / mqtt-bridge)."""

    def __init__(self, node, topic: str = 'mission_status'):
        self._pub = node.create_publisher(String, topic, 10)

    def notify(self, message: str):
        self._pub.publish(String(data=message))
