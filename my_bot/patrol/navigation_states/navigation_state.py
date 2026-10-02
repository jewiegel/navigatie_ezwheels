"""Basisklasse van het State-pattern.

Een toestand wordt bij elke wissel vers aangemaakt (zodat hij parameters kan
meekrijgen, bv. hoe lang te wachten) en is eigenaar van zijn eigen timers en
callbacks: zodra hij verlaten wordt, worden zijn timers geannuleerd en vallen
al zijn late callbacks (timers, Nav2-resultaten) automatisch weg via guard().
"""


class NavigationState:
    name = ''          # tekst op /patrol_state (HMI + indicator_node kennen deze)
    can_start = False  # mag een startsignaal hier een nieuwe route beginnen?
    can_stop = False   # mag een stopsignaal hier de route onderbreken?

    def __init__(self, ctl):
        self.ctl = ctl          # PatrolController (NavigationContext)
        self.active = False
        self._timers = []

    # ── Overschrijven in subklassen ───────────────────────────────────────────

    def on_enter(self):
        pass

    def on_leave(self):
        pass

    def published_name(self) -> str:
        return self.name

    # ── Hulpmiddelen voor subklassen ──────────────────────────────────────────

    def guard(self, fn):
        """Wikkel een callback zodat hij niets doet als deze toestand al verlaten is."""
        def guarded(*args, **kwargs):
            if self.active:
                fn(*args, **kwargs)
        return guarded

    def call_later(self, delay, fn):
        if not self.active:
            return None
        handle = self.ctl.scheduler.call_later(delay, self.guard(fn))
        self._timers.append(handle)
        return handle

    def call_every(self, period, fn):
        if not self.active:
            return None
        handle = self.ctl.scheduler.call_every(period, self.guard(fn))
        self._timers.append(handle)
        return handle

    # ── Aangeroepen door de controller ────────────────────────────────────────

    def enter(self):
        self.active = True
        self.on_enter()

    def leave(self):
        self.active = False
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
        self.on_leave()

    def __str__(self):
        return f'{type(self).__name__}({self.published_name()})'
