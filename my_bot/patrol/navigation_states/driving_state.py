from .navigation_state import NavigationState


class DrivingState(NavigationState):
    """Rijdt de resterende waypoints af als één NavigateThroughPoses-doel."""
    name = 'rijdend'
    can_stop = True

    def __init__(self, ctl):
        super().__init__(ctl)
        self._start_index = ctl.current_index
        self._timeout = None

    def on_enter(self):
        ctl = self.ctl
        poses = ctl.waypoints[self._start_index:]
        label = 'alternatief' if ctl.trying_alternative else 'route'
        ctl.log.info(
            f'Stuur {label}: {len(poses)} waypoints '
            f'(waypoint {self._start_index + 1} t/m {len(ctl.waypoints)})')
        sent = ctl.navigator.drive_through(
            poses,
            on_accepted=self.guard(self._on_accepted),
            on_rejected=self.guard(self._on_rejected),
            on_feedback=self.guard(self._on_feedback),
            on_result=self.guard(self._on_result))
        if not sent:
            ctl.fail(f'Nav2 navigate_through_poses niet beschikbaar na '
                     f'{ctl.config.server_timeout_sec:.0f} seconden')
            return
        self._restart_timeout()

    def on_leave(self):
        self.ctl.navigator.cancel()

    def published_name(self):
        # EZ-Wheel veiligheidsstop tijdens het rijden → naar buiten toe 'wachten'
        return 'wachten' if self.ctl.safety_stopped else self.name

    # ── Per-waypoint timeout ──────────────────────────────────────────────────

    def _restart_timeout(self):
        if self._timeout is not None:
            self._timeout.cancel()
        self._timeout = self.call_later(self.ctl.config.nav_timeout_sec, self._on_timeout)

    def _on_timeout(self):
        self.ctl.log.error(
            f'Nav2 timeout na {self.ctl.config.nav_timeout_sec:.0f}s — geen voortgang bij '
            f'waypoint {self.ctl.current_index + 1}')
        self.ctl.on_blocked()

    # ── Nav2-callbacks ────────────────────────────────────────────────────────

    def _on_accepted(self):
        self.ctl.goal_retries = 0

    def _on_rejected(self):
        ctl = self.ctl
        cfg = ctl.config
        if ctl.goal_retries < cfg.max_goal_retries:
            ctl.goal_retries += 1
            ctl.log.warn(
                f'Nav2 heeft het doel geweigerd — retry over {cfg.goal_retry_delay:.0f}s '
                f'(poging {ctl.goal_retries}/{cfg.max_goal_retries})')
            self.call_later(cfg.goal_retry_delay, ctl.drive_remaining)
        else:
            ctl.goal_retries = 0
            ctl.log.error('Nav2 heeft het doel geweigerd (ook na retry)')
            ctl.on_blocked()

    def _on_feedback(self, poses_remaining: int):
        total_sent = len(self.ctl.waypoints) - self._start_index
        new_index = self._start_index + (total_sent - poses_remaining)
        if new_index > self.ctl.current_index:
            self.ctl.waypoint_passed(new_index, poses_remaining)
            self._restart_timeout()

    def _on_result(self, succeeded: bool, status: int):
        if succeeded:
            self.ctl.complete()
        else:
            self.ctl.log.warn(
                f'Nav2 route mislukt — status: {status} '
                f'(4=SUCCEEDED, 5=CANCELED, 6=ABORTED)')
            self.ctl.on_blocked()
