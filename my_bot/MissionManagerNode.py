#!/usr/bin/env python3

from rclpy import Node
from mission_manager import MissionStateMachine
from mission_manager.ports import MissionPorts

class MissionManagerNode(Node):

    def __init__():
        stateMachine = MissionStateMachine()
        missionPorts = MissionPorts()

    def on_configure():
        pass

    def on_activate():
        pass

    def on_start_route():
        pass

    def on_cancel(req):
        pass

    def on_bt_state(msg):
        pass

    def on_nav_result(future):
        pass

