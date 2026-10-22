#!/usr/bin/env python3
"""Berichten tussen robot en HMI, zonder ROS of MQTT."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class SignalCommand:
    """Robot -> HMI: wat de HMI moet laten zien en horen."""
    blinkers: str = 'uit'      # 'uit', 'links', 'rechts', 'gevaar' (signal_manager)
    buzzer: str = 'uit'        # 'uit', 'enkel', 'dubbel', 'elke_5s' (signal_manager)
    buzzer_id: int = 0         # +1 per nieuw buzzerpatroon: speel alleen af als dit verandert,
    #                            dan herhaalt de hartslag een 'enkel' niet
    mission: str = ''          # missietoestand, bv. 'DRIVING gang_a' (MissionManager)


@dataclass(frozen=True)
class HMIStatus:
    """HMI -> robot."""
    ack: Optional[int] = None  # laatst ontvangen seq van een SignalCommand
    start: bool = False        # startknop ingedrukt
    stop: bool = False         # stopknop ingedrukt
