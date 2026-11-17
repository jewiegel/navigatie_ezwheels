#!/usr/bin/env python3

from my_bot_interfaces.srv import StartRoute

from hmi_bridge.inputs.MqttInput import MqttInput, pressed


class StartButtonInput(MqttInput):
    """Startknop -> route start_route_name (leeg = startknop doet niets)."""

    topic = 'robot/start'

    def setup(self, ctx):
        super().setup(ctx)
        self.route = ctx.declare_parameter('start_route_name', '').value
        self.client = ctx.create_client(StartRoute, 'start_route')

    def on_message(self, payload):
        if not pressed(payload):
            return
        if not self.route:
            self.ctx.get_logger().warn('HMI: startknop, maar start_route_name is leeg')
            return
        self.ctx.get_logger().info(f'HMI: startknop -> route "{self.route}"')
        self.ctx.call(self.client, StartRoute.Request(route_name=self.route))
