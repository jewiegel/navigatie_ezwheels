#!/usr/bin/env python3

import threading
from typing import Optional

from my_bot_interfaces.srv import GetRoute
from route_manager import Route, Waypoint


class RouteServiceClient():
    """Vraagt routes op bij de route_manager (service /get_route).

    get() blokkeert tot het antwoord er is; roep hem alleen aan vanuit een
    callback van een MultiThreadedExecutor met een reentrant callback group,
    anders kan het antwoord nooit verwerkt worden (deadlock)."""

    def __init__(self, node, callback_group=None, timeout_sec: float = 5.0):
        self._client = node.create_client(GetRoute, 'get_route', callback_group=callback_group)
        self._log = node.get_logger()
        self._timeout = timeout_sec

    def get(self, name: str) -> Optional[Route]:
        if not self._client.wait_for_service(timeout_sec=self._timeout):
            self._log.error('Service /get_route niet beschikbaar (draait de route_manager?)')
            return None

        done = threading.Event()
        future = self._client.call_async(GetRoute.Request(name=name))
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout):
            future.cancel()
            self._log.error(f'Geen antwoord van /get_route binnen {self._timeout:.0f}s')
            return None

        res = future.result()
        if not res.success:
            self._log.warn(f'Route "{name}" niet gevonden: {res.message}')
            return None
        waypoints = [Waypoint(w.name, w.x, w.y, w.yaw) for w in res.waypoints]
        return Route(name, res.map_id, waypoints)
