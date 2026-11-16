#!/usr/bin/env python3

import math

from sensor_msgs.msg import BatteryState

from hmi_bridge.outputs.MqttOutput import MqttOutput

TOPIC = 'robot/outputs/batterij'


class BatteryOutput(MqttOutput):
    """battery_state -> batterij als tekst 0-100, bv. "87" (niets zolang onbekend).

    Uit de spanning als battery_empty_v/battery_full_v gezet zijn, anders uit percentage
    (0-1) van de driver; remap battery_state naar het topic van de batterijdriver."""

    def setup(self, ctx):
        super().setup(ctx)
        self.empty_v = ctx.declare_parameter('battery_empty_v', 21.0).value
        self.full_v = ctx.declare_parameter('battery_full_v', 28.0).value
        self.percent = None
        ctx.create_subscription(BatteryState, 'battery_state', self._on_battery, 10)

    def _on_battery(self, msg):
        if self.full_v > self.empty_v > 0:
            # shortcut: lineair tussen leeg en vol; de echte ontlaadcurve is dat niet, gebruik
            # een tabel als het percentage te veel verspringt
            p = (msg.voltage - self.empty_v) / (self.full_v - self.empty_v)
        else:
            p = msg.percentage      # 0-1, NaN als de driver het niet weet
        self.percent = None if math.isnan(p) else round(min(max(p, 0.0), 1.0) * 100)

    def payloads(self, now):
        return {} if self.percent is None else {TOPIC: str(self.percent).encode('utf-8')}
