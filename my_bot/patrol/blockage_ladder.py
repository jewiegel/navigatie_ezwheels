"""Escalatieladder bij een blokkade (Strategy / Chain of Responsibility).

Elke nieuwe blokkade voert de volgende stap uit; een gepasseerd waypoint zet de
ladder terug naar de eerste stap. Volgorde aanpassen of een stap weglaten =
DEFAULT_LADDER wijzigen (of een eigen ladder aan PatrolController meegeven).
"""
from .navigation_states import BackingUpState, WaitingState


class BlockageStep:
    def execute(self, ctl) -> None:
        raise NotImplementedError


class WaitStep(BlockageStep):
    """Stilstaan en wachten op vrije doorgang; daarna direct de volgende stap."""

    def execute(self, ctl):
        seconds = ctl.config.wait_seconds
        ctl.log.warn(
            f'Blokkade bij waypoint {ctl.current_index + 1} — '
            f'wacht {seconds:.0f}s op vrije doorgang')
        ctl.transition(WaitingState(ctl, seconds, then=ctl.on_blocked))


class BackupStep(BlockageStep):
    """Piepen, stukje achteruit en daarna opnieuw proberen."""

    def execute(self, ctl):
        ctl.log.warn('Blokkade — piep, stukje achteruit en opnieuw proberen')
        ctl.signals.beep()
        ctl.transition(BackingUpState(ctl))


class AlternativeWaypointStep(BlockageStep):
    """Huidig waypoint overslaan en via het volgende verder; is er geen
    volgend waypoint, dan terugkeren."""

    def execute(self, ctl):
        next_index = ctl.current_index + 1
        if next_index >= len(ctl.waypoints):
            ctl.log.warn('Geen alternatief — terugkeren naar vorig waypoint')
            _pause_then(ctl, ctl.return_to_last)
            return
        ctl.log.warn(
            f'Nog steeds geblokkeerd — alternatieve route via waypoint {next_index + 1}')
        ctl.trying_alternative = True
        ctl.current_index = next_index
        _pause_then(ctl, ctl.drive_remaining)


class ReturnStep(BlockageStep):
    def execute(self, ctl):
        ctl.log.warn('Alternatieve route ook geblokkeerd — terugkeren')
        _pause_then(ctl, ctl.return_to_last)


def _pause_then(ctl, action):
    """Korte stilstand zodat Nav2 het oude doel kan afronden vóór het nieuwe."""
    ctl.transition(WaitingState(ctl, ctl.config.renav_pause, then=action))


DEFAULT_LADDER = (
    WaitStep(),
    BackupStep(),
    AlternativeWaypointStep(),
    ReturnStep(),
)
