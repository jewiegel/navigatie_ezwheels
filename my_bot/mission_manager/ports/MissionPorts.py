#!/usr/bin/env python3

import my_bot.mission_manager.ports.NavigationClient as NavigationClient
import my_bot.mission_manager.ports.UINotifier as UINotifier

class MissionPorts():
    def __init__(self, nav: NavigationClient, ui: UINotifier):
        self.nav: NavigationClient = nav
        self.ui: UINotifier = ui

    