#!/usr/bin/env python3

from hmi_bridge.inputs.MqttInput import MqttInput


class EzWayResultInput(MqttInput):
    """Antwoord van EZ-Way (ook op verzoeken van anderen): alleen loggen."""

    topic = 'amr_manager/result'

    def on_message(self, payload):
        self.ctx.get_logger().info(f'EZ-Way antwoord: {payload.decode(errors="replace")}')
