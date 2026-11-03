#!/usr/bin/env python3

from .NavigationClient import NavigationClient
from .TaskExecutor import TaskExecutor
from .UINotifier import UINotifier


class MissionPorts():

    def __init__(self, nav: NavigationClient, ui: UINotifier, tasks: TaskExecutor = None):
        self.nav = nav
        self.ui = ui
        self.tasks = tasks
