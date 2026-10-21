#!/usr/bin/env python3

from .Command import Command


class NotifyUICommand(Command):

    def __init__(self, message: str):
        self.message = message

    def execute(self, ports):
        ports.ui.notify(self.message)
