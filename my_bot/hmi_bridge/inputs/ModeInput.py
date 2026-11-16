#!/usr/bin/env python3

import json
import uuid
from datetime import datetime, timezone

from hmi_bridge.inputs.MqttInput import MqttInput, pressed

TOPIC_AMR_REQUEST = 'amr_manager/request'


class ModeInput(MqttInput):
    """Modusknop (robot/manual, robot/service) -> EZ-Way operating mode omzetten.

    EZ-Way zit op dezelfde broker (MQTT API, doc 'Operating Mode'); het antwoord komt op
    amr_manager/result (EzWayResultInput)."""

    def __init__(self, topic: str, mode: str):
        self.topic, self.mode = topic, mode

    def on_message(self, payload):
        if pressed(payload):
            self.ctx.get_logger().info(f'HMI: modusknop -> EZ-Way {self.mode}')
            self.ctx.mqtt_publish(TOPIC_AMR_REQUEST, mode_request(self.mode))


def mode_request(mode: str) -> bytes:
    """operating_mode_cmd van de EZ-Way MQTT API."""
    return json.dumps({
        'headerId': 0,
        'timestamp': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
        'requestId': str(uuid.uuid4()),
        'cmd': 'operating_mode_cmd',
        'cmd_params': {'state': mode},
    }).encode('utf-8')
