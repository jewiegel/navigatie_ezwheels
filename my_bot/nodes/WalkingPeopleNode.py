#!/usr/bin/env python3
"""WalkingPeopleNode: laat lopende mensen (actors) in Gazebo zichtbaar worden voor de lidar.

De gpu_lidar van Gazebo Fortress ziet geen actors. Daarom heeft elke actor <naam> in de
wereld een onzichtbare cilinder <naam>_lichaam (velocity-control + odometry-publisher). Dit node
leest de looproute van elke actor uit hetzelfde wereldbestand en stuurt de cilinder zo dat
hij op simulatietijd precies met de actor meeloopt.

Interface (per actor, via ros_gz_bridge):
  sub  /model/<naam>_lichaam/odometry (nav_msgs/Odometry)  positie cilinder (wereldframe)
  pub  /model/<naam>_lichaam/cmd_vel  (geometry_msgs/Twist)  snelheid cilinder

Parameters:
  world_file  SDF-wereld met de actors (verplicht)
  gain        terugkoppeling op positiefout (1/s)   (2.0)
  max_speed   maximale snelheid cilinder (m/s)       (2.0)
  rate_hz     stuurfrequentie                        (20.0)
"""
import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def trajectories(world_file):
    """{actornaam: [(t, x, y), ...]} uit de <actor>-scripts van een SDF-wereld."""
    result = {}
    for actor in ET.parse(world_file).getroot().iter('actor'):
        points = [(float(w.find('time').text), *map(float, w.find('pose').text.split()[:2]))
                  for w in actor.iter('waypoint')]
        if len(points) > 1:
            result[actor.get('name')] = points
    return result


def position(points, t):
    """Positie op tijd t, lineair tussen de waypoints; de route loopt rond zoals <loop>true</loop>."""
    t %= points[-1][0]
    for (t0, x0, y0), (t1, x1, y1) in zip(points, points[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
    return points[-1][1:]


class WalkingPeopleNode(Node):
    def __init__(self):
        super().__init__('walking_people')
        world_file = self.declare_parameter('world_file', '').value
        self.gain = self.declare_parameter('gain', 2.0).value
        self.max_speed = self.declare_parameter('max_speed', 2.0).value
        self.dt = 1.0 / self.declare_parameter('rate_hz', 20.0).value
        self.routes = trajectories(world_file)
        self.poses = {}
        self.cmd = {}
        for name in self.routes:
            body = f'/model/{name}_lichaam'
            self.create_subscription(
                Odometry, f'{body}/odometry',
                lambda msg, n=name: self.poses.__setitem__(n, msg.pose.pose.position), 10)
            self.cmd[name] = self.create_publisher(Twist, f'{body}/cmd_vel', 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f'{len(self.routes)} lopers uit {world_file}: {", ".join(self.routes)}')

    def step(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t == 0.0:            # nog geen /clock
            return
        for name, points in self.routes.items():
            current = self.poses.get(name)
            if current is None:
                continue
            x, y = position(points, t)
            nx, ny = position(points, t + self.dt)
            vx = (nx - x) / self.dt + self.gain * (x - current.x)
            vy = (ny - y) / self.dt + self.gain * (y - current.y)
            scale = min(1.0, self.max_speed / max(math.hypot(vx, vy), 1e-9))
            msg = Twist()
            # De cilinder draait niet (velocity-control houdt de hoeksnelheid op 0): body = wereld.
            msg.linear.x, msg.linear.y = vx * scale, vy * scale
            self.cmd[name].publish(msg)


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
