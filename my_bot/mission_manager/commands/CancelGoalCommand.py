#!/usr/bin/env python3

from .Command import Command


class CancelGoalCommand(Command):

    def execute(self, ports):
        ports.nav.cancel()
