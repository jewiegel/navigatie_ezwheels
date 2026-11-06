#!/usr/bin/env python3
"""WaypointTaskNode: biedt de waypoint-acties aan als action servers.

Elke WaypointTaskServerBase krijgt een action server met zijn task_name
(my_bot_interfaces/action/WaypointTask); de mission_manager roept ze aan na het bereiken
van een waypoint (TaskRegistry, zie register_tasks in MissionManagerNode.py).

Interface:
  action  dock_cart  (my_bot_interfaces/action/WaypointTask)  robot midden onder de kar
          gebruikt: navigate_to_pose (Nav2), /laser_1/scan_filtered, tf, pub cmd_vel
          pub docking/active (std_msgs/Bool, transient_local): True tijdens het koppelen
          pub dock_cart/markers (visualization_msgs/MarkerArray, odom): zoekgebied, gevonden
              wielen en gemeten kar, voor RViz/Foxglove. Mislukt meten: scan in ~/.ros

Parameters (dock_cart.*, standaard de echte kar, die ook in de simulatie staat):
  cart_length, cart_width  buitenmaat van de kar (zoekgebied)                (0.70, 0.845)
  search_margin       marge rond de kar en rond elk verwacht draaipunt        (0.25)
  castor_x, castor_y  halve lengte/breedte van de rechthoek van de draai-assen van de
                      zwenkwielen van de kar                                  (0.31, 0.35)
  wheel_l, wheel_w    wiel op lidarhoogte: lengte (koorde) en dikte           (0.19, 0.03)
  trail               naloop: horizontale afstand draai-as tot wielmidden     (0.03)
  wheel_gap           grootste gat in de streep van één wiel (niet-weerkaatsend
                      ijzer); groter = gaten overbruggen, maar wielen kleven aan
                      dingen ernaast                                          (0.08)
  wheel_rms           hoeveel de streep gemiddeld van de wielvorm mag afwijken
                      (kronkels)                                              (0.02)
  axle_x              wielas van de robot t.o.v. base_link (draaipunt)        (0.13)
  robot_half_width    halve breedte van de robot (rijbaancontrole)            (0.245)
"""
import json

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from actions.DockCarTask import DockCarTask
from my_bot_interfaces.action import WaypointTask


class WaypointTaskNode(Node):

    def __init__(self):
        super().__init__('waypoint_tasks')
        # Reentrant: een actie wacht op scans, tf en Nav2 terwijl hij zelf nog loopt.
        cb = ReentrantCallbackGroup()
        tasks = [DockCarTask(self, cb)]
        self._servers = [
            ActionServer(self, WaypointTask, task.task_name,
                         execute_callback=lambda handle, task=task: self._execute(task, handle),
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
            for task in tasks]
        self.get_logger().info(f'Acties: {", ".join(t.task_name for t in tasks)}')

    def _execute(self, task, handle):
        try:
            params = json.loads(handle.request.params_json or '{}')
        except json.JSONDecodeError as e:
            handle.abort()
            return WaypointTask.Result(success=False, message=f'params geen geldige JSON: {e}')
        self.get_logger().info(f'{task.task_name} gestart {params}')
        result = task.execute(
            params, lambda status: handle.publish_feedback(WaypointTask.Feedback(status=status)),
            lambda: handle.is_cancel_requested)
        if handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return WaypointTask.Result(success=result.success, message=result.message)


def main():
    rclpy.init()
    node = WaypointTaskNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
