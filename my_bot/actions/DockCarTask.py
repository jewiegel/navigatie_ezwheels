#!/usr/bin/env python3

import json
import math
import os
import statistics
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from docking import (
    axis_candidates, axis_cmd, cart_castors, compose, find_wheels, inverse, match_cart,
    point_cmd, points_in_area, rotate_cmd, search_area, transform, wheels_in_lane)
from .WaypointTaskServerBase import TaskResult, WaypointTaskServerBase

APPROACH_M = 1.4          # Nav2 brengt het midden van de robot zover voor het kar-midden
STAGE_M = 1.0             # daar precies op de as uitlijnen, dan recht naar binnen
# Rest stuurt het rechtdoor naar binnen rijden bij (1 m tot de kar, convergeert in ~0,3 m).
STAGE_TOL_Y, STAGE_TOL_TH = 0.02, math.radians(1.5)
FINAL_TOL_XY, FINAL_TOL_TH = 0.015, math.radians(1.5)
STAGE_ATTEMPTS = 3
GATE_SEARCH, GATE_CHECK = 0.4, 0.12   # m rond de verwachte draai-as: AMCL-fout / na meten
SCANS = 10                # mediaan per straal over zoveel scans (stilstaand): minder ruis
SETTLE_S = 0.5            # na stoppen even wachten voor de eerste scan
LANE_MARGIN = 0.02        # m vrij tussen robot en een naar binnen gedraaid wiel
NAV_TIMEOUT_S = 180.0
MAX_RANGE_M = 3.0
LOOP_S = 0.05


class _Abort(Exception):
    pass


def _pose_2d(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return (t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class DockCarTask(WaypointTaskServerBase):
    """Rijdt de robot nauwkeurig midden onder de kar.

    params: x, y, yaw = pose van het midden van de robot als hij gedockt is (map-frame):
    midden onder de kar, yaw = rijrichting onder de kar. De robot rijdt erin vanaf de
    kant tegenover yaw.

    1. Nav2 naar APPROACH_M voor de kar (grof: AMCL zit er decimeters naast).
    2. Stilstaan en de kar meten: de zwenkwielen rond de verwachte plek (zie CartDocking);
       vanaf dan geldt de gemeten kar-pose (odom-frame), niet meer AMCL.
    3. Precies op de as van de kar zetten, STAGE_M ervoor, met de juiste richting; daar
       opnieuw meten (dichterbij, nauwkeuriger) en zo nodig opnieuw uitlijnen.
    4. Recht naar binnen tot het midden op odometrie; richting bijdraaien. Onder de kar
       ziet de lidar de wielen maar half (rand van het blikveld): daar niet meten.

    Staan de vier wielen (bijna) dezelfde kant op, dan is niet te zien of de draai-assen
    voor of achter de wielen zitten; de fout is dan hooguit de naloop (trail).

    Vanaf stap 2 tot het einde staat docking/active op True (signal_manager: alarmlichten).
    """

    def __init__(self, node, callback_group=None):
        super().__init__('dock_cart')
        self._node = node
        self._log = node.get_logger()

        def param(name, default):
            return node.declare_parameter(f'dock_cart.{name}', default).value

        # Maten van de kar en de robot; standaard die van de simulatie.
        self._castors = cart_castors(param('castor_x', 0.29), param('castor_y', 0.3625))
        self._wheel_l = param('wheel_l', 0.205)   # wiel op lidarhoogte: lengte (koorde) ...
        self._wheel_w = param('wheel_w', 0.05)    # ... en breedte
        self._trail = param('trail', 0.04)        # naloop: draai-as tot wielmidden
        self._axle_x = param('axle_x', 0.13)   # wielas t.o.v. base_link: hier draait de robot om
        self._half_width = param('robot_half_width', 0.245)   # buitenkant van de robot
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node)
        self._cmd = node.create_publisher(Twist, 'cmd_vel', 10)
        self._docking = node.create_publisher(
            Bool, 'docking/active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._docking.publish(Bool(data=False))
        self._markers = node.create_publisher(
            MarkerArray, 'dock_cart/markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._nav = ActionClient(node, NavigateToPose, 'navigate_to_pose',
                                 callback_group=callback_group)
        node.create_subscription(LaserScan, '/laser_1/scan_filtered', self._on_scan, 10,
                                 callback_group=callback_group)
        self._lock = threading.Lock()
        self._scans = None         # lijst tijdens een meting, anders None
        self._cart = None          # gemeten pose van de kar (= doelpose) in odom
        self._cancelled = lambda: False

    # ── Actie ─────────────────────────────────────────────────────────────────

    def execute(self, params, feedback, cancelled):
        try:
            target = (float(params['x']), float(params['y']), float(params.get('yaw', 0.0)))
        except (KeyError, TypeError, ValueError):
            return TaskResult(False, 'dock_cart heeft params x, y (en yaw) nodig: kar in map')
        self._cancelled = cancelled
        try:
            feedback('naar de startpositie voor de kar')
            self._navigate(compose(target, (-APPROACH_M, 0.0, 0.0)))
            feedback('kar zoeken')
            self._docking.publish(Bool(data=True))
            try:
                odom_map = _pose_2d(self._tf.lookup_transform('odom', 'map', Time()))
            except TransformException as e:
                raise _Abort(f'geen map -> odom ({e})')
            self._cart, _ = self._measure(compose(odom_map, target), GATE_SEARCH, False)
            for attempt in range(1, STAGE_ATTEMPTS + 1):
                feedback(f'uitlijnen voor de kar (poging {attempt})')
                stage = (self._axle_x - STAGE_M, 0.0)   # wielas STAGE_M voor de gedockte plek
                self._run(lambda: point_cmd(self._axle(), transform(self._cart, stage)))
                self._run(lambda: self._rotate_cmd())
                self._cart, wheels = self._measure(self._cart, GATE_CHECK, True)
                _, y, th = self._rel(self._axle())
                self._log.info(f'Uitgelijnd voor de kar: y={1000 * y:+.1f} mm, '
                               f'hoek={math.degrees(th):+.2f} graden')
                if abs(y) <= STAGE_TOL_Y and abs(th) <= STAGE_TOL_TH:
                    break
            else:
                raise _Abort('lukt niet om recht voor de kar te komen')
            blocking = wheels_in_lane(wheels, self._cart, self._wheel_l, self._wheel_w,
                                      self._half_width + LANE_MARGIN)
            if blocking:
                where = ', '.join('({:+.2f}, {:+.2f})'.format(*self._rel((*c, a))[:2])
                                  for c, a in blocking)
                raise _Abort(f'zwenkwiel staat in de rijbaan (kar-frame x, y: {where}); '
                             'wiel rechtzetten en opnieuw proberen')
            feedback('onder de kar rijden')
            self._run(lambda: axis_cmd(self._rel(self._axle()), self._axle_x))
            self._run(lambda: self._rotate_cmd())
            x, y, th = self._rel(self._base())
            message = (f'gedockt: afwijking x={1000 * x:+.1f} mm, y={1000 * y:+.1f} mm, '
                       f'hoek={math.degrees(th):+.2f} graden (odometrie t.o.v. de laatste meting)')
            if math.hypot(x, y) > FINAL_TOL_XY or abs(th) > FINAL_TOL_TH:
                raise _Abort('te onnauwkeurig ' + message)
            self._log.info(message)
            return TaskResult(True, message)
        except _Abort as e:
            self._log.warn(f'dock_cart mislukt: {e}')
            return TaskResult(False, str(e))
        finally:
            self._stop()
            self._docking.publish(Bool(data=False))
            with self._lock:
                self._scans = None
            self._cart = None

    # ── Stappen ───────────────────────────────────────────────────────────────

    def _navigate(self, pose):
        if not self._nav.wait_for_server(timeout_sec=5.0):
            raise _Abort('Nav2 navigate_to_pose niet beschikbaar')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2)
        handle = self._wait(self._nav.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise _Abort('Nav2 weigert het doel voor de kar')
        result = handle.get_result_async()
        try:
            self._wait(result, NAV_TIMEOUT_S)
        except _Abort:
            handle.cancel_goal_async()
            raise
        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise _Abort(f'Nav2 kwam niet bij de kar (status {result.result().status})')

    def _run(self, command):
        """command() geeft (v, w) of None (klaar); herhalen tot klaar."""
        while True:
            self._check_cancel()
            out = command()
            if out is None:
                self._stop()
                return
            self._publish(*out)
            time.sleep(LOOP_S)

    def _rotate_cmd(self):
        w = rotate_cmd(self._rel(self._axle())[2])
        return None if w is None else (0.0, w)

    # ── Kar meten met de lidar ────────────────────────────────────────────────

    def _measure(self, prior, gate, trust_prior):
        """Stilstaand SCANS scans nemen, per straal de mediaan, en daarin de kar zoeken rond
        prior (odom). Geeft (pose van de kar, gevonden wielen), in odom."""
        self._stop()
        time.sleep(SETTLE_S)
        with self._lock:
            self._scans = []
        deadline = time.monotonic() + 5.0
        while True:
            self._check_cancel()
            with self._lock:
                if len(self._scans) >= SCANS:
                    scans, self._scans = self._scans, None
                    break
            if time.monotonic() > deadline:
                raise _Abort('geen lidarscans')
            time.sleep(LOOP_S)

        last = scans[-1]
        try:
            sensor = _pose_2d(self._tf.lookup_transform(
                'odom', last.header.frame_id, Time.from_msg(last.header.stamp),
                timeout=Duration(seconds=0.5)))
        except TransformException as e:
            raise _Abort(f'geen odom -> lidar ({e})')
        points = []
        for i in range(len(last.ranges)):
            valid = [s.ranges[i] for s in scans
                     if s.range_min < s.ranges[i] < min(s.range_max, MAX_RANGE_M)]
            if len(valid) > len(scans) // 2:
                r = statistics.median(valid)
                a = last.angle_min + i * last.angle_increment
                points.append(transform(sensor, (r * math.cos(a), r * math.sin(a))))
        # Alleen zoeken waar de wielen kunnen staan: de kar rond prior plus de zoekmarge.
        half_x, half_y = search_area(self._castors, self._trail, self._wheel_l, gate)
        near = points_in_area(points, prior, half_x, half_y)
        angle_tol = max(math.radians(1.0), 2.5 * abs(last.angle_increment))
        wheels = find_wheels(near, sensor[:2], self._wheel_l, self._wheel_w, angle_tol=angle_tol)
        match = match_cart(wheels, prior, self._castors, gate, self._trail, trust_prior)
        self._show(prior, half_x, half_y, wheels, match[0] if match and match[1] >= 3 else None)
        if match is None or match[1] < 3:
            nearest = []
            for c in self._castors:
                e = transform(prior, c)
                d = [math.dist(e, a) for w in wheels for a in axis_candidates(w, self._trail)]
                nearest.append(f'{100 * min(d):.0f}' if d else '-')
            path = self._save(sensor, prior, gate, points, wheels)
            raise _Abort(f'kar niet gevonden: {len(wheels)} wielen in het zoekgebied '
                         f'({2 * half_x:.1f} x {2 * half_y:.1f} m), geen drie op de verwachte '
                         f'plek; dichtstbijzijnde wiel per draaipunt: {", ".join(nearest)} cm '
                         f'(zoekmarge {100 * gate:.0f} cm). Scan bewaard: {path}')
        pose, n, rms = match
        shift = math.dist(pose[:2], prior[:2])
        self._log.info(f'Kar gemeten: {n} wielen, fout {1000 * rms:.1f} mm, '
                       f'{1000 * shift:.0f} mm van de verwachting')
        return pose, wheels

    def _show(self, prior, half_x, half_y, wheels, cart):
        """Markers op dock_cart/markers (odom): zoekgebied en verwachte draaipunten (geel),
        gevonden wielen (oranje), gemeten kar (groen)."""
        stamp = self._node.get_clock().now().to_msg()
        markers = [Marker(action=Marker.DELETEALL)]

        def marker(kind, rgb, scale, ns):
            m = Marker(type=kind, ns=ns, id=len(markers))
            m.header.frame_id, m.header.stamp = 'odom', stamp
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
            m.pose.orientation.w = 1.0
            markers.append(m)
            return m

        def outline(pose, hx, hy, rgb, ns):
            m = marker(Marker.LINE_STRIP, rgb, 0.01, ns)
            for x, y in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy), (hx, hy)):
                px, py = transform(pose, (x, y))
                m.points.append(Point(x=px, y=py, z=0.10))

        def dots(pose, rgb, ns):
            m = marker(Marker.SPHERE_LIST, rgb, 0.04, ns)
            for c in self._castors:
                px, py = transform(pose, c)
                m.points.append(Point(x=px, y=py, z=0.10))

        outline(prior, half_x, half_y, (1.0, 0.8, 0.0), 'zoekgebied')
        dots(prior, (1.0, 0.8, 0.0), 'verwacht')
        for (x, y), a in wheels:
            m = marker(Marker.CUBE, (1.0, 0.4, 0.0), 0.0, 'wielen')
            m.scale.x, m.scale.y, m.scale.z = self._wheel_l, self._wheel_w, 0.1
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.10
            m.pose.orientation.z, m.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
        if cart is not None:
            outline(cart, max(c[0] for c in self._castors), max(c[1] for c in self._castors),
                    (0.0, 0.9, 0.0), 'kar')
            dots(cart, (0.0, 0.9, 0.0), 'kar')
        self._markers.publish(MarkerArray(markers=markers))

    def _save(self, sensor, prior, gate, points, wheels):
        """Scan en verwachting als JSON in ~/.ros, om een mislukte meting na te kijken."""
        folder = os.path.join(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')))
        path = os.path.join(folder, time.strftime('dock_cart_%Y%m%d_%H%M%S.json'))
        try:
            os.makedirs(folder, exist_ok=True)
            with open(path, 'w') as f:
                json.dump({'sensor': sensor, 'prior': prior, 'gate': gate,
                           'castors': self._castors, 'wheel_l': self._wheel_l,
                           'wheel_w': self._wheel_w, 'trail': self._trail,
                           'points': points, 'wheels': wheels}, f)
        except OSError as e:
            return f'niet gelukt ({e})'
        return path

    def _on_scan(self, scan: LaserScan):
        with self._lock:
            if self._scans is not None:
                self._scans.append(scan)

    # ── Hulpjes ───────────────────────────────────────────────────────────────

    def _base(self):
        try:
            return _pose_2d(self._tf.lookup_transform('odom', 'base_link', Time()))
        except TransformException as e:
            raise _Abort(f'geen odom -> base_link ({e})')

    def _axle(self):
        return compose(self._base(), (self._axle_x, 0.0, 0.0))

    def _rel(self, pose):
        """pose (odom) uitgedrukt in het kar-frame."""
        return compose(inverse(self._cart), pose)

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self._cmd.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _check_cancel(self):
        if self._cancelled():
            raise _Abort('geannuleerd')

    def _wait(self, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout
        while not done.wait(LOOP_S):
            self._check_cancel()
            if time.monotonic() > deadline:
                raise _Abort('time-out')
        return future.result()
