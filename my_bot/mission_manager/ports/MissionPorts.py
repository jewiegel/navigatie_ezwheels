#!/usr/bin/env python3

from .NavigationClient import NavigationClient
from .UINotifier import UINotifier


class MissionPorts():

    def __init__(self, nav: NavigationClient, ui: UINotifier):
        self.nav = nav
        self.ui = ui
