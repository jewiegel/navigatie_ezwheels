from .navigation_state import NavigationState


class StartingState(NavigationState):
    """Zombie-doelen annuleren en controller_server resetten vóór vertrek.

    Dit duurt ~8 s. Stoppen mag hier (anders werd een stopsignaal in deze fase
    genegeerd en reed de robot toch weg); een tweede startsignaal niet.
    """
    name = 'planning'
    can_stop = True

    def on_enter(self):
        self.ctl.navigator.prepare(self.guard(self._on_prepared))

    def _on_prepared(self, controller_reset: bool):
        cfg = self.ctl.config
        delay = cfg.controller_reset_delay if controller_reset else cfg.start_delay
        self.call_later(delay, self._go)

    def _go(self):
        self.ctl.log.info('Doelen verwerkt — route wordt gestart')
        self.ctl.drive_remaining()
