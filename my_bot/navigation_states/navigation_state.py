# Toestanden (State-pattern): elke toestand is een klasse met on_enter()/on_leave()
# en bepaalt zelf hoe hij op gebeurtenissen reageert. PatrolNode is de
# NavigationContext: die houdt de actieve toestand vast en wisselt via
# _set_state(<StateKlasse>).


class NavigationState:
    """Interface voor alle navigatietoestanden."""
    name = ''   # tekst die op /patrol_state gepubliceerd wordt (HMI)

    def __init__(self, ctx):
        self.ctx = ctx   # PatrolNode (NavigationContext)

    def on_enter(self):
        pass

    def on_leave(self):
        pass

    def can_start(self) -> bool:
        """Mag een startsignaal in deze toestand een nieuwe route beginnen?"""
        return False

    def can_stop(self) -> bool:
        """Mag een stopsignaal in deze toestand de route onderbreken?"""
        return False

    def on_nav_result(self, status: int):
        """Resultaat van het NavigateThroughPoses-doel. Standaard: negeren."""
        pass

    def on_tick(self):
        """Periodieke controle (elke 5 s). Standaard: niets."""
        pass

    def published_name(self) -> str:
        return self.name
