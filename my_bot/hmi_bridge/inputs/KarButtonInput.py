#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from my_bot_interfaces.srv import StartRoute
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import String

from hmi_bridge.inputs.MqttInput import MqttInput, pressed


class KarButtonInput(MqttInput):
    """Karknop: AMCL terug naar zijn ingestelde startpositie (parameters initial_pose.* van
    /amcl), daarna route kar_route_name; alleen als er geen missie loopt (IDLE of ARRIVED)."""

    topic = 'robot/kar'

    def setup(self, ctx):
        super().setup(ctx)
        self.route = ctx.declare_parameter('kar_route_name', 'kaas').value
        self.mission = ''
        ctx.create_subscription(String, 'mission_status', self._on_mission, 10)
        self.start_client = ctx.create_client(StartRoute, 'start_route')
        self.initialpose_pub = ctx.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.amcl_params = AsyncParameterClient(ctx, 'amcl')
        self._timer = None

    def _on_mission(self, msg):
        self.mission = msg.data

    def on_message(self, payload):
        if not pressed(payload):
            return
        log = self.ctx.get_logger()
        if self.mission.split(' ', 1)[0] not in ('', 'IDLE', 'ARRIVED'):
            log.warn(f'HMI: karknop genegeerd, missie loopt ({self.mission})')
            return
        if not self.amcl_params.services_are_ready():
            log.error('HMI: karknop, maar /amcl is niet bereikbaar')
            return
        log.info(f'HMI: karknop -> startpositie terug, dan route "{self.route}"')
        self.amcl_params.get_parameters(
            ['initial_pose.x', 'initial_pose.y', 'initial_pose.yaw'], self._reset_pose)

    def _reset_pose(self, future):
        try:
            x, y, yaw = (float(parameter_value_to_python(v)) for v in future.result().values)
        except (TypeError, ValueError):    # parameter ontbreekt (NOT_SET) of niet numeriek
            self.ctx.get_logger().error('Karknop: initial_pose.* van /amcl niet te lezen')
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.ctx.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25  # zoals RViz' 2D Pose Estimate
        msg.pose.covariance[35] = 0.0685
        self.initialpose_pub.publish(msg)
        self.ctx.get_logger().info(f'Startpositie teruggezet: x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
        # AMCL zet map->odom pas bij de volgende scans om; wacht even zodat Nav2 vanaf de
        # nieuwe positie plant
        self._timer = self.ctx.create_timer(1.0, self._start_route)

    def _start_route(self):
        self.ctx.destroy_timer(self._timer)
        self.ctx.call(self.start_client, StartRoute.Request(route_name=self.route))
