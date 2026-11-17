#!/usr/bin/env python3

from std_msgs.msg import String

from hmi_bridge.outputs.MqttOutput import MqttOutput, bit

TOPIC_LEFT = 'robot/outputs/lamp_links'
TOPIC_RIGHT = 'robot/outputs/lamp_rechts'


class LampsOutput(MqttOutput):
    """Knipperlichten van de signal_manager (indicators: 'uit', 'links', 'rechts', 'gevaar')
    -> lamp_links/lamp_rechts "1"/"0"; de bridge laat ze knipperen (blink_period_s: helft aan,
    helft uit, begint met aan). 'gevaar' = beide."""

    def setup(self, ctx):
        super().setup(ctx)
        self.period = ctx.declare_parameter('blink_period_s', 1.0).value
        self.blinkers, self.since = 'uit', 0.0
        ctx.create_subscription(String, 'indicators', self._on_indicators, 10)

    def _on_indicators(self, msg):
        if msg.data != self.blinkers:
            self.blinkers, self.since = msg.data, self.ctx.now()

    def payloads(self, now):
        on = (now - self.since) % self.period < self.period / 2
        return {TOPIC_LEFT: bit(on and self.blinkers in ('links', 'gevaar')),
                TOPIC_RIGHT: bit(on and self.blinkers in ('rechts', 'gevaar'))}
