from .navigation_state import NavigationState

STEP_PERIOD = 0.1   # s tussen cmd_vel-berichten


class BackingUpState(NavigationState):
    """Stukje achteruit rijden en daarna een vers pad sturen."""
    name = 'wachten'   # HMI/indicator_node kennen geen aparte achteruit-stand
    can_stop = True

    def on_enter(self):
        self._start = self.ctl.scheduler.now()
        self.call_every(STEP_PERIOD, self._step)

    def on_leave(self):
        self.ctl.signals.stop_wheels()

    def _step(self):
        cfg = self.ctl.config
        if self.ctl.scheduler.now() - self._start < cfg.backup_duration:
            self.ctl.signals.drive(-cfg.backup_speed)
            return
        self.ctl.log.info('Achteruit klaar — vers pad sturen')
        self.ctl.drive_remaining()
