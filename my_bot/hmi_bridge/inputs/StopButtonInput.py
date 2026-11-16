#!/usr/bin/env python3

from std_srvs.srv import Trigger

from hmi_bridge.inputs.MqttInput import MqttInput, pressed


class StopButtonInput(MqttInput):
    """Stopknop -> missie afbreken."""

    topic = 'robot/stop'

    def setup(self, ctx):
        super().setup(ctx)
        self.client = ctx.create_client(Trigger, 'cancel_mission')

    def on_message(self, payload):
        if pressed(payload):
            self.ctx.get_logger().info('HMI: stopknop')
            self.ctx.call(self.client, Trigger.Request())
