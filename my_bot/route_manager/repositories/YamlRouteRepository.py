#!/usr/bin/env python3

import math
import os
import tempfile

import yaml

from ..Route import Route
from ..Waypoint import Waypoint
from .RouteRepository import RouteRepository


class YamlRouteRepository(RouteRepository):
    """Routes in één YAML-bestand:

        routes:
          gang_a:
            map_id: ziekenhuis
            waypoints:
              - {name: start, x: 3.19, y: 0.21, yaw: 0.0}   # m, m, rad in het map-frame

    Het bestand wordt bij elke aanroep opnieuw gelezen, dus wijzigingen gelden
    zonder herstart. Ongeldige inhoud geeft ValueError met de route en het waypoint.
    """

    def __init__(self, path: str):
        super().__init__()
        self._path = path

    def list(self) -> list[str]:
        return sorted(self._read())

    def load(self, name: str) -> Route:
        routes = self._read()
        if name not in routes:
            raise KeyError(name)
        return _parse_route(name, routes[name])

    def save(self, route: Route) -> None:
        routes = self._read()
        routes[route.name] = {
            'map_id': route.map_id,
            'waypoints': [
                {'name': w.name, 'x': w.x, 'y': w.y, 'yaw': w.yaw} for w in route.waypoints],
        }
        self._write(routes)

    def delete(self, name: str) -> None:
        routes = self._read()
        if name not in routes:
            raise KeyError(name)
        del routes[name]
        self._write(routes)

    # ── Intern ────────────────────────────────────────────────────────────────

    def _read(self) -> dict:
        if not os.path.exists(self._path):
            return {}
        with open(self._path) as f:
            data = yaml.safe_load(f) or {}
        routes = data.get('routes') or {}
        if not isinstance(routes, dict):
            raise ValueError(f"{self._path}: 'routes' moet een mapping naam -> route zijn")
        return routes

    def _write(self, routes: dict) -> None:
        # Eerst naar een tijdelijk bestand en dan vervangen: een crash halverwege
        # laat nooit een half geschreven routebestand achter.
        directory = os.path.dirname(os.path.abspath(self._path))
        os.makedirs(directory, exist_ok=True)
        with tempfile.NamedTemporaryFile('w', dir=directory, suffix='.tmp', delete=False) as f:
            f.write(self._header())
            # default_flow_style=None: elk waypoint op één regel, zoals met de hand geschreven
            yaml.safe_dump({'routes': routes}, f, sort_keys=False, default_flow_style=None)
        self._copy_permissions(f.name)
        os.replace(f.name, self._path)

    def _copy_permissions(self, tmp_path: str) -> None:
        """Rechten en eigenaar van het oude bestand overnemen. Een tijdelijk bestand is
        0600; draait de node als root (Docker), dan kan de gebruiker het anders niet meer lezen."""
        if not os.path.exists(self._path):
            os.chmod(tmp_path, 0o644)
            return
        st = os.stat(self._path)
        os.chmod(tmp_path, st.st_mode & 0o777)
        try:
            os.chown(tmp_path, st.st_uid, st.st_gid)
        except PermissionError:
            pass  # alleen root mag chown; als gewone gebruiker is de eigenaar al goed

    def _header(self) -> str:
        """Commentaar bovenaan het bestand; yaml.safe_dump zou het anders weggooien."""
        if not os.path.exists(self._path):
            return ''
        header = []
        with open(self._path) as f:
            for line in f:
                if line.strip() and not line.lstrip().startswith('#'):
                    break
                header.append(line)
        return ''.join(header)


def _parse_route(name: str, entry) -> Route:
    if not isinstance(entry, dict):
        raise ValueError(f'Route "{name}": moet map_id en waypoints hebben')
    raw_waypoints = entry.get('waypoints') or []
    if not raw_waypoints:
        raise ValueError(f'Route "{name}": heeft geen waypoints')
    waypoints = []
    for i, wp in enumerate(raw_waypoints, 1):
        try:
            x, y, yaw = (float(wp[k]) for k in ('x', 'y', 'yaw'))
        except (TypeError, KeyError, ValueError):
            raise ValueError(
                f'Route "{name}", waypoint {i}: moet x, y en yaw als getallen hebben') from None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError(f'Route "{name}", waypoint {i}: x, y en yaw moeten eindig zijn')
        waypoints.append(Waypoint(str(wp.get('name', f'wp{i}')), x, y, yaw))
    return Route(name, str(entry.get('map_id', '')), waypoints)
