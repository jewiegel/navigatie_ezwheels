"""Observers op toestandswissels: elk regelt één neveneffect."""
from .navigation_states import (
    BackingUpState, CompletedState, DrivingState, ErrorState, IdleState,
    ReturningState, StoppedState)


class HmiStatePublisher:
    """Publiceert de toestandsnaam op /patrol_state (→ HMI en indicator_node)."""

    def __init__(self, signals):
        self._signals = signals

    def on_state_changed(self, old, new):
        self._signals.publish_state(new.published_name())


class IndicatorPolicy:
    """Eén tabel: welke toestand zet welke knipperlichten.
    Toestanden die er niet in staan laten de lampen zoals ze zijn."""

    MODES = {
        IdleState:      'uit',
        DrivingState:   'uit',
        BackingUpState: 'gevaar',
        ReturningState: 'gevaar',
        CompletedState: 'gevaar',
        StoppedState:   'uit',
        ErrorState:     'gevaar',
    }

    def __init__(self, signals):
        self._signals = signals

    def on_state_changed(self, old, new):
        if old is new:
            return
        mode = self.MODES.get(type(new))
        if mode is not None:
            self._signals.set_indicator(mode)


class TransitionLogger:
    def __init__(self, logger):
        self._log = logger

    def on_state_changed(self, old, new):
        if old is new:
            return
        self._log.info(f'[STATE] {old} → {new}')
