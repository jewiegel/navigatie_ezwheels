#!/usr/bin/env python3

from std_msgs.msg import String

from hmi_bridge.BuzzerPlayer import BuzzerPlayer
from hmi_bridge.outputs.MqttOutput import MqttOutput, bit

TOPIC = 'robot/outputs/buzzer'


class BuzzerOutput(MqttOutput):
    """Buzzerpatroon van de signal_manager (buzzer_pattern), afgespeeld op de aan/uit-uitgang."""

    def setup(self, ctx):
        super().setup(ctx)
        self.player = BuzzerPlayer()
        ctx.create_subscription(String, 'buzzer_pattern', self._on_pattern, 10)

    def _on_pattern(self, msg):
        try:
            self.player.play(msg.data, self.ctx.now())
        except ValueError as e:
            self.ctx.get_logger().warn(f'Buzzer: {e}')

    def payloads(self, now):
        return {TOPIC: bit(self.player.is_on(now))}
