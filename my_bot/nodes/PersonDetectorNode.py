#!/usr/bin/env python3
"""PersonDetectorNode: personen (benen) in de 2D-lidarscan met DR-SPAAM, op CPU.

DR-SPAAM (https://github.com/VisualComputingInstitute/DR-SPAAM-Detector, GPL-3) is getraind
op de DROW-dataset: SICK-lidar op 37 cm, 225 graden. Onze lidar zit op 16,5 cm en kijkt +-100
graden; meet dus hoe goed hij hier werkt voordat je er gedrag op bouwt. Torch, dr_spaam en het
checkpoint zitten in de Docker-image (~/ros2_docker/Dockerfile).

Interface:
  sub  scan               (sensor_msgs/LaserScan)  /laser_1/scan_filtered
  pub  person_detections  (geometry_msgs/PoseArray)       personen in het lidarframe
  pub  person_markers     (visualization_msgs/Marker)    bolletjes voor RViz/Foxglove

Parameters:
  checkpoint   DR-SPAAM-gewichten            (/opt/dr_spaam/dr_spaam_e40.pth)
  conf_thresh  minimale zekerheid (0..1); 0.2 gemeten in de simulatie met paspoppen:
               binnen 2 m 91% gevonden, 2-4 m 75%, 4-8 m 34%; 0.11 vals alarm per scan   (0.2)
  stride       elk zoveelste scanpunt; 2 halveert de rekentijd   (2)
"""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

NO_RETURN = 29.99   # zo codeert DR-SPAAM "geen echo" (zoals de originele ROS 1-node)


def prepare_scan(ranges):
    """LaserScan.ranges -> numpy-array zoals DR-SPAAM die verwacht (geen inf/nan/0)."""
    scan = np.array(ranges, dtype=np.float64)
    scan[~np.isfinite(scan) | (scan <= 0.0)] = NO_RETURN
    return scan


def to_laser_xy(dets_xy):
    """DR-SPAAM geeft (r sin phi, r cos phi); ROS-lidarframe is x vooruit, y links."""
    return dets_xy[:, ::-1]


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector')
        checkpoint = self.declare_parameter('checkpoint', '/opt/dr_spaam/dr_spaam_e40.pth').value
        self.conf_thresh = self.declare_parameter('conf_thresh', 0.2).value
        stride = self.declare_parameter('stride', 2).value
        # dr_spaam gebruikt np.int/np.bool, weg sinds NumPy 1.24 (EZ-Way-container heeft 1.26)
        np.int, np.bool = getattr(np, 'int', int), getattr(np, 'bool', bool)
        from dr_spaam.detector import Detector   # zwaar (torch): pas laden als het node echt start
        self.detector = Detector('DR-SPAAM', checkpoint, gpu=False, stride=stride)
        self.detections_pub = self.create_publisher(PoseArray, 'person_detections', 10)
        self.markers_pub = self.create_publisher(Marker, 'person_markers', 10)
        # Diepte 1: is het node trager dan de lidar, dan altijd de nieuwste scan.
        qos = qos_profile_sensor_data
        qos.depth = 1
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos)
        self.times = []
        self.get_logger().info(f'DR-SPAAM geladen ({checkpoint}), stride {stride}')

    def on_scan(self, msg):
        if not self.detector.laser_spec_set():
            # DR-SPAAM neemt een scan symmetrisch rond x vooruit aan.
            if abs(msg.angle_min + msg.angle_max) > msg.angle_increment:
                self.get_logger().error(
                    f'scan niet symmetrisch ({msg.angle_min:.3f}..{msg.angle_max:.3f} rad)')
            self.detector.set_laser_spec(msg.angle_increment, len(msg.ranges))
        start = time.monotonic()
        dets_xy, dets_cls, _ = self.detector(prepare_scan(msg.ranges))
        self.report_time(time.monotonic() - start)
        keep = dets_cls.reshape(-1) >= self.conf_thresh
        xy = to_laser_xy(dets_xy[keep])

        detections = PoseArray(header=msg.header)
        for x, y in xy:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            detections.poses.append(pose)
        self.detections_pub.publish(detections)

        marker = Marker(header=msg.header, ns='personen', id=0, type=Marker.SPHERE_LIST,
                        action=Marker.ADD)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.g, marker.color.a = 0.8, 0.8
        marker.points = [Point(x=float(x), y=float(y), z=0.5) for x, y in xy]
        self.markers_pub.publish(marker)

    def report_time(self, seconds):
        self.times.append(seconds)
        if len(self.times) == 100:
            self.get_logger().info(
                f'rekentijd per scan: gemiddeld {1000 * np.mean(self.times):.0f} ms, '
                f'max {1000 * np.max(self.times):.0f} ms')
            self.times = []


def main():
    rclpy.init()
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
