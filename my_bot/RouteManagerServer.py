#!/usr/bin/env python3
"""RouteManagerServer: levert routes op naam aan de MissionManagerNode.

Interface:
  service  get_route  (my_bot_interfaces/srv/GetRoute)

Parameter:
  routes_file  YAML met routes (standaard share/my_bot/config/routes.yaml)
"""
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from my_bot_interfaces.msg import Waypoint as WaypointMsg
from my_bot_interfaces.srv import GetRoute
from route_manager.repositories import YamlRouteRepository


class RouteManagerServer(Node):

    def __init__(self):
        super().__init__('route_manager')
        default_file = os.path.join(
            get_package_share_directory('my_bot'), 'config', 'routes.yaml')
        routes_file = self.declare_parameter('routes_file', default_file).value
        self.repository = YamlRouteRepository(routes_file)
        self.create_service(GetRoute, 'get_route', self.on_get_route)

        try:
            names = self.repository.list()
            self.get_logger().info(f'Routes uit {routes_file}: {", ".join(names) or "(geen)"}')
        except Exception as e:  # o.a. OSError, yaml.YAMLError, ValueError
            self.get_logger().error(f'Routebestand {routes_file} niet leesbaar: {e}')

    def on_get_route(self, req, res):
        try:
            route = self.repository.load(req.name)
        except KeyError:
            res.success = False
            res.message = f'Onbekende route "{req.name}"; beschikbaar: {self._available()}'
            self.get_logger().warn(res.message)
            return res
        except Exception as e:  # o.a. OSError, yaml.YAMLError, ValueError
            res.success = False
            res.message = f'Route "{req.name}" niet te laden: {e}'
            self.get_logger().error(res.message)
            return res

        res.success = True
        res.map_id = route.map_id
        res.waypoints = [
            WaypointMsg(name=w.name, x=w.x, y=w.y, yaw=w.yaw) for w in route.waypoints]
        self.get_logger().info(f'Route "{route.name}" geleverd ({len(route.waypoints)} waypoints)')
        return res

    def _available(self) -> str:
        try:
            return ', '.join(self.repository.list()) or '(geen)'
        except Exception:
            return '(routebestand niet leesbaar)'


def main():
    rclpy.init()
    node = RouteManagerServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
