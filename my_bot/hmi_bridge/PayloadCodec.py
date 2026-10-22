#!/usr/bin/env python3
"""JSON-berichten tussen robot en HMI.

Robot -> HMI (command):  {"v": 1, "seq": 42, "blinkers": "links", "buzzer": "enkel",
                          "buzzer_id": 3, "mission": "DRIVING gang_a"}
HMI -> robot (status):   {"v": 1, "ack": 42, "start": false, "stop": false}
"""

import json

from hmi_bridge.HMIMessages import HMIStatus, SignalCommand


class PayloadCodec:

    VERSION: int = 1

    @staticmethod
    def encode_command(cmd: SignalCommand, seq: int) -> bytes:
        return json.dumps({
            'v': PayloadCodec.VERSION, 'seq': seq, 'blinkers': cmd.blinkers,
            'buzzer': cmd.buzzer, 'buzzer_id': cmd.buzzer_id, 'mission': cmd.mission,
        }).encode('utf-8')

    @staticmethod
    def decode_status(payload: bytes) -> HMIStatus:
        """ValueError bij een bericht dat geen geldige status van deze versie is."""
        try:
            data = json.loads(payload)
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            raise ValueError(f'geen JSON: {e}') from None
        if not isinstance(data, dict):
            raise ValueError('geen JSON-object')
        if data.get('v') != PayloadCodec.VERSION:
            raise ValueError(f'versie {data.get("v")!r}, verwacht {PayloadCodec.VERSION}')
        ack = data.get('ack')
        if ack is not None and (not isinstance(ack, int) or isinstance(ack, bool)):
            raise ValueError(f'ack moet een geheel getal zijn, niet {ack!r}')
        for key in ('start', 'stop'):
            if not isinstance(data.get(key, False), bool):
                raise ValueError(f'{key} moet true of false zijn')
        return HMIStatus(ack, data.get('start', False), data.get('stop', False))
