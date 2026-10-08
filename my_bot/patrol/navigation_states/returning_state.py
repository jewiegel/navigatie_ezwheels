from .navigation_state import NavigationState


class ReturningState(NavigationState):
    """Terugrijden naar het laatst gehaalde waypoint. Eindigt altijd in fout:
    de route is mislukt en een operator moet ingrijpen."""
    name = 'rijdend'
    can_stop = True

    def on_enter(self):
        ctl = self.ctl
        idx = ctl.last_successful_idx
        ctl.log.info(f'Terugkeren naar waypoint {idx + 1}')
        sent = ctl.navigator.drive_to(
            ctl.waypoints[idx],
            on_rejected=self.guard(self._on_rejected),
            on_result=self.guard(self._on_result))
        if not sent:
            ctl.fail('Nav2 navigate_to_pose niet beschikbaar — kan niet terugkeren')
            return
        self.call_later(ctl.config.return_timeout_sec, self._on_timeout)

    def on_leave(self):
        self.ctl.navigator.cancel()

    def _on_rejected(self):
        self.ctl.fail('FOUT: Kan niet terugkeren — operator ingrijpen vereist')

    def _on_result(self, succeeded: bool, status: int):
        if succeeded:
            self.ctl.fail('FOUT: Route mislukt — robot teruggekeerd — operator ingrijpen vereist')
        else:
            self.ctl.fail(
                'FOUT: Route mislukt — kon ook niet terugkeren — operator ingrijpen vereist')

    def _on_timeout(self):
        self.ctl.fail(
            f'FOUT: Terugkeren duurt langer dan {self.ctl.config.return_timeout_sec:.0f}s '
            f'— operator ingrijpen vereist')
