#!/usr/bin/env python3
"""Berichten tussen robot en HMI, zonder ROS of MQTT."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SignalCommand:
    """Robot -> HMI: stand van de uitgangen en de status voor het scherm."""
    blinkers: str = 'uit'      # 'uit', 'links', 'rechts', 'gevaar' (signal_manager)
    buzzer_on: bool = False    # buzzer nu aan (BuzzerPlayer speelt het patroon af)
    mission: str = ''          # missietoestand, bv. 'DRIVING gang_a' (MissionManager)


@dataclass(frozen=True)
class HMIStatus:
    """HMI -> robot: één knopdruk."""
    start: bool = False
    stop: bool = False
