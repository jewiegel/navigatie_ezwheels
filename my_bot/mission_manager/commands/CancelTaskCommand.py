#!/usr/bin/env python3

from .Command import Command


class CancelTaskCommand(Command):

    def execute(self, ports):
        ports.tasks.cancel()
