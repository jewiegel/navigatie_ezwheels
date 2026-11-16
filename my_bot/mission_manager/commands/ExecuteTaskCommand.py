#!/usr/bin/env python3

from route_manager import ActionSpecification
from .Command import Command


class ExecuteTaskCommand(Command):

    def __init__(self, task: ActionSpecification):
        self.task = task

    def execute(self, ports):
        ports.tasks.execute(self.task)
