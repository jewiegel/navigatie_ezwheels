#!/usr/bin/env python3
"""WalkingPeopleNode: lopende paspoppen in Gazebo, voor het testen van beendetectie.

Elke loper uit people_file (config/people.yaml) bestaat uit drie modellen die de launch
spawnt (people:=true): <naam>_been_links, <naam>_been_rechts (models/paspop_been) en
<naam>_romp (models/paspop_romp). De lidar ziet alleen de benen; de romp zit erboven.
Dit node laat ze op simulatietijd heen en weer lopen, de benen om en om naar voren zoals bij
lopen. Geen actors: die ziet de gpu_lidar van Fortress maar soms (alleen als er verder iets
beweegt), en dan op een verkeerde plek.

Interface (per model, via ros_gz_bridge):
  sub  /model/<model>/odometry (nav_msgs/Odometry)  positie (wereldframe)
  pub  /model/<model>/cmd_vel  (geometry_msgs/Twist)  snelheid

Parameters:
  people_file  lopers: start, eind, snelheid (verplicht)
  gain         terugkoppeling op positiefout (1/s)   (4.0)
  max_speed    maximale snelheid per model (m/s)      (3.0)
  rate_hz      stuurfrequentie                        (20.0)
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

LEGS = {'been_links': 1, 'been_rechts': -1}   # kant t.o.v. de looprichting
HIP_HALF_WIDTH = 0.1     # beenmidden naast het lichaamsmidden (m)
STEP_AMPLITUDE = 0.25    # been max zoveel voor/achter het lichaamsmidden (m)
STEP_PERIOD = 1.3        # duur van een volledige loopcyclus (twee stappen) (s)
RAMP_S = 0.4             # stapgrootte loopt in zoveel seconden op/af bij starten en stoppen


def route(start, end, speed, turn_s):
    """Heen, omdraaien, terug, omdraaien: [(t, x, y, yaw), ...]; het laatste punt = het eerste."""
    there = math.atan2(end[1] - start[1], end[0] - start[0])
    t1 = math.dist(start, end) / speed
    return [(0.0, *start, there), (t1, *end, there), (t1 + turn_s, *end, there + math.pi),
            (2 * t1 + turn_s, *start, there + math.pi), (2 * t1 + 2 * turn_s, *start, there)]


def routes(config):
    """{lopernaam: route} uit de inhoud van people.yaml."""
    return {name: route(w['start'], w['end'], config['speed'], config['turn_s'])
            for name, w in config['walkers'].items()}


def pose(points, t):
    """(x, y, yaw) op tijd t, lineair tussen de routepunten; de route loopt rond."""
    t %= points[-1][0]
    for a, b in zip(points, points[1:]):
        if t <= b[0]:
            f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 1.0
            return tuple(a[i] + f * (b[i] - a[i]) for i in (1, 2, 3))
    return points[-1][1:]


def leg_positions(points, t):
    """{been: (x, y)}: naast het lichaamsmidden, en tijdens het lopen om en om naar voren."""
    x, y, yaw = pose(points, t)
    # Gemiddelde snelheid rond t: de stapgrootte loopt geleidelijk op en af, zonder sprong.
    (bx, by, _), (ax, ay, _) = pose(points, t - RAMP_S / 2), pose(points, t + RAMP_S / 2)
    speed = math.hypot(ax - bx, ay - by) / RAMP_S
    swing = STEP_AMPLITUDE * min(speed, 1.0) * math.sin(2 * math.pi * t / STEP_PERIOD)
    fwd, left = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
    return {leg: (x + side * HIP_HALF_WIDTH * left[0] + side * swing * fwd[0],
                  y + side * HIP_HALF_WIDTH * left[1] + side * swing * fwd[1])
            for leg, side in LEGS.items()}


def targets(points, t):
    """{onderdeel: (x, y)} voor één loper: beide benen en de romp (lichaamsmidden)."""
    x, y, _ = pose(points, t)
    return {**leg_positions(points, t), 'romp': (x, y)}


def models(config):
    """{modelnaam: (modelmap, x, y)} om te spawnen: alle onderdelen op hun plek op t = 0."""
    return {f'{name}_{part}': ('paspop_romp' if part == 'romp' else 'paspop_been', x, y)
            for name, points in routes(config).items()
            for part, (x, y) in targets(points, 0.0).items()}


class WalkingPeopleNode(Node):
    def __init__(self):
        super().__init__('walking_people')
        people_file = self.declare_parameter('people_file', '').value
        self.gain = self.declare_parameter('gain', 4.0).value
        self.max_speed = self.declare_parameter('max_speed', 3.0).value
        self.dt = 1.0 / self.declare_parameter('rate_hz', 20.0).value
        with open(people_file) as f:
            self.routes = routes(yaml.safe_load(f))
        self.positions = {}
        self.cmd = {}
        for name, points in self.routes.items():
            for part in targets(points, 0.0):
                model = f'{name}_{part}'
                self.create_subscription(
                    Odometry, f'/model/{model}/odometry',
                    lambda msg, m=model: self.positions.__setitem__(m, msg.pose.pose.position), 10)
                self.cmd[model] = self.create_publisher(Twist, f'/model/{model}/cmd_vel', 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f'{len(self.routes)} lopers uit {people_file}: {", ".join(self.routes)}')

    def step(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t == 0.0:            # nog geen /clock
            return
        for name, points in self.routes.items():
            now, ahead = targets(points, t), targets(points, t + self.dt)
            for part, (x, y) in now.items():
                model = f'{name}_{part}'
                current = self.positions.get(model)
                if current is None:
                    continue
                nx, ny = ahead[part]
                vx = (nx - x) / self.dt + self.gain * (x - current.x)
                vy = (ny - y) / self.dt + self.gain * (y - current.y)
                scale = min(1.0, self.max_speed / max(math.hypot(vx, vy), 1e-9))
                msg = Twist()
                # Het model draait niet (velocity-control houdt de hoeksnelheid op 0): body = wereld.
                msg.linear.x, msg.linear.y = vx * scale, vy * scale
                self.cmd[model].publish(msg)


def main():
    rclpy.init()
    node = WalkingPeopleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
