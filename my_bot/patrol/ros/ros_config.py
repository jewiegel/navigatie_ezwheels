"""PatrolConfig vullen vanuit ROS-parameters en het routebestand.

Elk veld van PatrolConfig is een ROS-parameter met dezelfde naam (standaard =
de waarde in config.py). Routes komen uit `routes_file`; leeg = het
patrol_routes.yaml in de share-map van my_bot. Is dat er niet, dan gelden de
ingebouwde DEFAULT_ROUTES.
"""
import os
from dataclasses import fields

from rcl_interfaces.msg import ParameterDescriptor

from ..config import PatrolConfig, load_routes

PACKAGE = 'my_bot'
ROUTES_FILENAME = 'patrol_routes.yaml'


def load_config(node) -> PatrolConfig:
    defaults = PatrolConfig()
    values = {}
    # dynamic_typing: '120' in de YAML mag ook als het veld een float is
    descriptor = ParameterDescriptor(dynamic_typing=True)
    for f in fields(PatrolConfig):
        if f.name == 'routes':
            continue
        default = getattr(defaults, f.name)
        node.declare_parameter(f.name, default, descriptor)
        values[f.name] = type(default)(node.get_parameter(f.name).value)

    node.declare_parameter('routes_file', '')
    routes_file = node.get_parameter('routes_file').value or _default_routes_file()
    log = node.get_logger()
    if routes_file and os.path.isfile(routes_file):
        values['routes'] = load_routes(routes_file)
        log.info(f'Routes geladen uit {routes_file}')
    else:
        log.warn(f'Routebestand niet gevonden ({routes_file or "-"}) — ingebouwde routes gebruikt')
    return PatrolConfig(**values)


def _default_routes_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(PACKAGE), 'config', ROUTES_FILENAME)
    except Exception:
        return ''
