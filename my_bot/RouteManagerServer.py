#!/usr/bin/env python3
"""RouteManagerServer: levert routes op naam aan de MissionManagerNode en
maakt nieuwe routes door waypoints aan te klikken (concept-route).

Interface:
  service  get_route             (my_bot_interfaces/srv/GetRoute)   route op naam
  topic    route_preview         (geometry_msgs/PoseArray)          laatst opgevraagde route tonen
  topic    route_draft/add_pose  (geometry_msgs/PoseStamped, map)   waypoint toevoegen
  topic    route_draft/poses     (geometry_msgs/PoseArray)          concept-route tonen
  service  route_draft/undo      (std_srvs/srv/Trigger)             laatste waypoint weg
  service  route_draft/clear     (std_srvs/srv/Trigger)             concept-route leeg
  service  route_draft/save      (my_bot_interfaces/srv/SaveRoute)  opslaan onder een naam

Parameters:
  routes_file  YAML met routes (standaard share/my_bot/config/routes.yaml)
  map_id       kaart-id voor opgeslagen routes (standaard ziekenhuis)
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from my_bot_interfaces.msg import Waypoint as WaypointMsg
from my_bot_interfaces.srv import GetRoute, SaveRoute
from route_manager import RouteDraft
from route_manager.repositories import YamlRouteRepository


class RouteManagerServer(Node):

    def __init__(self):
        super().__init__('route_manager')
        default_file = os.path.join(
            get_package_share_directory('my_bot'), 'config', 'routes.yaml')
        routes_file = self.declare_parameter('routes_file', default_file).value
        self.map_id = self.declare_parameter('map_id', 'ziekenhuis').value
        self.repository = YamlRouteRepository(routes_file)
        self.create_service(GetRoute, 'get_route', self.on_get_route)
        # transient_local: een later geopend RViz/Foxglove ziet de laatste waarde meteen
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.preview_pub = self.create_publisher(PoseArray, 'route_preview', latched)

        self.draft = RouteDraft()
        self.draft_pub = self.create_publisher(PoseArray, 'route_draft/poses', latched)
        self.create_subscription(PoseStamped, 'route_draft/add_pose', self.on_add_pose, 10)
        self.create_service(Trigger, 'route_draft/undo', self.on_undo)
        self.create_service(Trigger, 'route_draft/clear', self.on_clear)
        self.create_service(SaveRoute, 'route_draft/save', self.on_save)
        self._publish_draft()

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

        self.preview_pub.publish(self._pose_array(route.waypoints))
        res.success = True
        res.map_id = route.map_id
        res.waypoints = [
            WaypointMsg(name=w.name, x=w.x, y=w.y, yaw=w.yaw) for w in route.waypoints]
        self.get_logger().info(f'Route "{route.name}" geleverd ({len(route.waypoints)} waypoints)')
        return res

    # ── Concept-route ─────────────────────────────────────────────────────────

    def on_add_pose(self, msg: PoseStamped):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'Waypoint genegeerd: frame "{msg.header.frame_id}", verwacht "map" '
                '(zet in RViz/Foxglove het display frame op map)')
            return
        q = msg.pose.orientation
        try:
            wp = self.draft.add(msg.pose.position.x, msg.pose.position.y,
                                2.0 * math.atan2(q.z, q.w))
        except ValueError as e:
            self.get_logger().warn(f'Waypoint genegeerd: {e}')
            return
        self.get_logger().info(
            f'Concept-route: waypoint {len(self.draft.waypoints)} '
            f'({wp.x:.2f}, {wp.y:.2f}, yaw {wp.yaw:.2f})')
        self._publish_draft()

    def on_undo(self, req, res):
        removed = self.draft.undo()
        res.success = removed is not None
        res.message = (f'Waypoint verwijderd, {len(self.draft.waypoints)} over'
                       if removed else 'Concept-route is al leeg')
        self._publish_draft()
        return res

    def on_clear(self, req, res):
        self.draft.clear()
        res.success, res.message = True, 'Concept-route leeg'
        self._publish_draft()
        return res

    def on_save(self, req, res):
        try:
            route = self.draft.build(req.name, self.map_id)
            if not req.overwrite and req.name in self.repository.list():
                raise ValueError(
                    f'Route "{req.name}" bestaat al; kies een andere naam of overwrite: true')
            self.repository.save(route)
        except Exception as e:  # o.a. ValueError, OSError
            res.success, res.message = False, str(e)
            self.get_logger().warn(f'Opslaan mislukt: {e}')
            return res
        self.draft.clear()
        self._publish_draft()
        res.success = True
        res.message = f'Route "{route.name}" opgeslagen ({len(route.waypoints)} waypoints)'
        self.get_logger().info(res.message)
        return res

    def _publish_draft(self):
        self.draft_pub.publish(self._pose_array(self.draft.waypoints))

    def _pose_array(self, waypoints) -> PoseArray:
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for w in waypoints:
            pose = Pose()
            pose.position.x, pose.position.y = w.x, w.y
            pose.orientation.z, pose.orientation.w = math.sin(w.yaw / 2), math.cos(w.yaw / 2)
            msg.poses.append(pose)
        return msg

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
