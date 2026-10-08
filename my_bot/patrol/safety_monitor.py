"""EZ-Wheel veiligheidsstop-detectie.

De EZ-Wheel stopt zelf bij een persoon/object. Wij zien dat als: de robot
stond net nog te rijden en staat nu langer dan `safety_still_seconds` stil
terwijl we in DrivingState zijn. Dan melden we 'wachten' aan de HMI, en duurt
het langer dan `person_wait_seconds`, dan escaleren we de blokkade.

Aan/uit via de parameter safety_monitor_enabled.
"""
from .navigation_states import DrivingState


class SafetyMonitor:

    def __init__(self, ctl):
        self._ctl = ctl
        self._was_moving = False
        self._still_since = None
        self._person_timer = None

    def on_motion(self, moving: bool) -> None:
        """Aanroepen bij elk odometrie-bericht."""
        if not moving:
            if self._was_moving and self._still_since is None:
                self._still_since = self._ctl.scheduler.now()
            return
        self._was_moving = True
        self._still_since = None
        if self._ctl.safety_stopped:
            self._cancel_person_timer()
            self._ctl.log.info('EZ-Wheel veiligheidsstop opgeheven — robot rijdt verder')
            self._ctl.set_safety_stopped(False)

    def check(self) -> None:
        """Periodiek aanroepen (bv. elke 0,5 s)."""
        ctl = self._ctl
        if not isinstance(ctl.state, DrivingState) or ctl.safety_stopped:
            return
        if self._still_since is None:
            return
        if ctl.scheduler.now() - self._still_since <= ctl.config.safety_still_seconds:
            return
        ctl.log.info('EZ-Wheel veiligheidsstop actief — robot gestopt door persoon/object')
        ctl.set_safety_stopped(True)
        if self._person_timer is None:
            self._person_timer = ctl.scheduler.call_later(
                ctl.config.person_wait_seconds, self._on_person_wait_over)

    def on_state_changed(self, old, new) -> None:
        if old is new:
            return
        self._was_moving = False
        self._still_since = None
        self._cancel_person_timer()

    def _on_person_wait_over(self):
        self._person_timer = None
        ctl = self._ctl
        if not ctl.safety_stopped or not isinstance(ctl.state, DrivingState):
            return
        ctl.log.warn(
            f'Persoon blokkeert al {ctl.config.person_wait_seconds:.0f}s — escaleren')
        ctl.on_person_blocked()

    def _cancel_person_timer(self):
        if self._person_timer is not None:
            self._person_timer.cancel()
            self._person_timer = None
