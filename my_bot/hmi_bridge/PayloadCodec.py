#!/usr/bin/env python3
"""MQTT-topics en payloads van de FT2J HMI (zelfde als de oude mqtt_hmi_bridge, die werkte
op de echte robot). Alles is tekst; uitgangen zijn "1" (aan) of "0" (uit).

Robot -> HMI:
  robot/outputs/lamp_links, robot/outputs/lamp_rechts   "1"/"0" ('gevaar' = beide),
                                                         knipperend via blink_on
  robot/outputs/buzzer                                   "1"/"0"
  robot/patrol_state     'idle', 'rijdend', 'wachten', 'voltooid', 'fout'
  robot/outputs/state_id                                 "1" als geblokkeerd (gevarenlicht + elke 5 s
                                                         piep), anders "0"
  robot/outputs/batterij                                 percentage als tekst, bv. "87"
                                                         (niet gestuurd zolang onbekend)
HMI -> robot:
  robot/start, robot/stop                                "1" = knop ingedrukt
"""

from typing import Dict, Optional

from hmi_bridge.HMIMessages import HMIStatus, SignalCommand

TOPIC_LAMP_LEFT = 'robot/outputs/lamp_links'
TOPIC_LAMP_RIGHT = 'robot/outputs/lamp_rechts'
TOPIC_BUZZER = 'robot/outputs/buzzer'
TOPIC_STATE = 'robot/patrol_state'
TOPIC_BATTERY = 'robot/outputs/batterij'
TOPIC_STATE_ID = 'robot/outputs/state_id'
TOPIC_START = 'robot/start'
TOPIC_STOP = 'robot/stop'

# MissionState -> statuswoorden die de HMI van de oude patrol_node kent
PATROL_STATES = {
    'IDLE': 'idle', 'DRIVING': 'rijdend', 'BLOCKED': 'wachten', 'EXECUTING_TASKS': 'rijdend',
    'WAITING_OPERATOR': 'fout', 'ARRIVED': 'voltooid', 'ERROR': 'fout',
}


def _bit(on: bool) -> bytes:
    return b'1' if on else b'0'


class PayloadCodec:

    @staticmethod
    def encode_command(cmd: SignalCommand) -> Dict[str, bytes]:
        """Payload per MQTT-topic."""
        state = cmd.mission.split(' ', 1)[0]
        out = {
            TOPIC_LAMP_LEFT: _bit(cmd.blink_on and cmd.blinkers in ('links', 'gevaar')),
            TOPIC_LAMP_RIGHT: _bit(cmd.blink_on and cmd.blinkers in ('rechts', 'gevaar')),
            TOPIC_BUZZER: _bit(cmd.buzzer_on),
            TOPIC_STATE: PATROL_STATES.get(state, state.lower() or 'idle').encode('utf-8'),
            TOPIC_STATE_ID: _bit(cmd.blocked or state == 'BLOCKED'),
        }
        if cmd.battery is not None:
            out[TOPIC_BATTERY] = str(cmd.battery).encode('utf-8')
        return out

    @staticmethod
    def decode_status(topic: str, payload: bytes) -> Optional[HMIStatus]:
        """Knopdruk, of None voor alles wat geen "1" op een knoptopic is (genegeerd)."""
        if payload.strip() != b'1':
            return None
        if topic == TOPIC_START:
            return HMIStatus(start=True)
        if topic == TOPIC_STOP:
            return HMIStatus(stop=True)
        return None
