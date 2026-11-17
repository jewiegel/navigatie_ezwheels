#!/usr/bin/env python3

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from hmi_bridge.outputs.MqttOutput import MqttOutput, bit

TOPIC_STATE = 'robot/patrol_state'
TOPIC_STATE_ID = 'robot/outputs/state_id'

# MissionState -> statuswoorden die de HMI van de oude patrol_node kent
PATROL_STATES = {
    'IDLE': 'idle', 'DRIVING': 'rijdend', 'BLOCKED': 'wachten', 'EXECUTING_TASKS': 'rijdend',
    'WAITING_OPERATOR': 'fout', 'ARRIVED': 'voltooid', 'ERROR': 'fout',
}


class StateOutput(MqttOutput):
    """Missietoestand (mission_status) -> patrol_state als tekst; geblokkeerd
    (navigation/blocked van de BT, of BLOCKED) -> state_id "1" (gevarenlicht + elke 5 s piep)."""

    def setup(self, ctx):
        super().setup(ctx)
        self.mission, self.blocked = '', False
        ctx.create_subscription(String, 'mission_status', self._on_mission, 10)
        ctx.create_subscription(
            Bool, 'navigation/blocked', self._on_blocked,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_mission(self, msg):
        self.mission = msg.data

    def _on_blocked(self, msg):
        self.blocked = msg.data

    def payloads(self, now):
        state = self.mission.split(' ', 1)[0]
        return {TOPIC_STATE: PATROL_STATES.get(state, state.lower() or 'idle').encode('utf-8'),
                TOPIC_STATE_ID: bit(self.blocked or state == 'BLOCKED')}
