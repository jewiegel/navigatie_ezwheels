"""Fakes voor de ports in patrol/ports.py: geen ROS, volledige controle over de tijd."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot')))

from patrol import (  # noqa: E402
    HmiStatePublisher, IndicatorPolicy, PatrolConfig, PatrolController, SafetyMonitor)


class FakeTimer:
    def __init__(self, due, fn, period):
        self.due, self.fn, self.period = due, fn, period
        self.active = True

    def cancel(self):
        self.active = False


class FakeScheduler:
    """Virtuele klok: advance(s) laat alle timers die in die tijd aflopen vuren."""

    def __init__(self):
        self.t = 0.0
        self._timers = []

    def now(self):
        return self.t

    def call_later(self, delay, fn):
        timer = FakeTimer(self.t + delay, fn, None)
        self._timers.append(timer)
        return timer

    def call_every(self, period, fn):
        timer = FakeTimer(self.t + period, fn, period)
        self._timers.append(timer)
        return timer

    def advance(self, seconds):
        target = self.t + seconds
        while True:
            due = [t for t in self._timers if t.active and t.due <= target + 1e-9]
            if not due:
                break
            timer = min(due, key=lambda t: t.due)
            self.t = max(self.t, timer.due)
            if timer.period is None:
                timer.active = False
            else:
                timer.due += timer.period
            timer.fn()
        self.t = target


class Goal:
    def __init__(self, kind, waypoints, callbacks):
        self.kind = kind            # 'through' of 'to'
        self.waypoints = waypoints
        self.cb = callbacks
        self.cancelled = False


class FakeNavigator:
    """Neemt doelen op; tests spelen Nav2 na via accept()/feedback()/succeed()/fail()."""

    def __init__(self):
        self.available = True
        self.goals = []
        self.prepare_callbacks = []
        self.planners = []
        self.cancel_count = 0

    @property
    def goal(self):
        return self.goals[-1]

    def select_planner(self, planner_id):
        self.planners.append(planner_id)

    def prepare(self, on_ready):
        self.prepare_callbacks.append(on_ready)

    def drive_through(self, waypoints, on_accepted, on_rejected, on_feedback, on_result):
        if not self.available:
            return False
        self.goals.append(Goal('through', list(waypoints), dict(
            accepted=on_accepted, rejected=on_rejected, feedback=on_feedback, result=on_result)))
        return True

    def drive_to(self, waypoint, on_rejected, on_result):
        if not self.available:
            return False
        self.goals.append(Goal('to', [waypoint], dict(rejected=on_rejected, result=on_result)))
        return True

    def cancel(self):
        self.cancel_count += 1
        for g in self.goals:
            g.cancelled = True

    # Nav2 naspelen
    def accept(self):
        self.goal.cb['accepted']()

    def reject(self):
        self.goal.cb['rejected']()

    def feedback(self, remaining):
        self.goal.cb['feedback'](remaining)

    def succeed(self):
        self.goal.cb['result'](True, 4)

    def fail(self):
        self.goal.cb['result'](False, 6)


class FakeSignals:
    def __init__(self):
        self.states = []
        self.indicators = []
        self.drive_cmds = []
        self.beeps = 0
        self.stops = 0

    def stop_wheels(self):
        self.stops += 1

    def drive(self, linear_x):
        self.drive_cmds.append(linear_x)

    def beep(self):
        self.beeps += 1

    def set_indicator(self, mode):
        self.indicators.append(mode)

    def publish_state(self, name):
        self.states.append(name)


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))

    def error(self, msg):
        self.lines.append(('error', msg))


class Rig:
    """Controller + fakes, opgebouwd zoals patrol_node.py dat doet."""

    def __init__(self, config=None, safety=False):
        self.config = config or PatrolConfig()
        self.nav = FakeNavigator()
        self.signals = FakeSignals()
        self.clock = FakeScheduler()
        self.log = FakeLogger()
        self.ctl = PatrolController(self.config, self.nav, self.signals, self.clock, self.log)
        self.ctl.add_listener(HmiStatePublisher(self.signals))
        self.ctl.add_listener(IndicatorPolicy(self.signals))
        self.monitor = None
        if safety:
            self.monitor = SafetyMonitor(self.ctl)
            self.ctl.add_listener(self.monitor)
        self.ctl.begin()

    @property
    def state(self):
        return type(self.ctl.state).__name__

    def start_and_drive(self, route_id=1):
        """Start → opstartfase (controller-reset) → rijdend, doel geaccepteerd."""
        self.ctl.start(route_id)
        self.nav.prepare_callbacks[-1](True)
        self.clock.advance(self.config.controller_reset_delay)
        self.nav.accept()
