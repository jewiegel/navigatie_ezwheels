#!/usr/bin/env python3

import json
import math
import os
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
    axis_cmd, compose, inverse, measure_cart, point_cmd, points_in_lane, rotate_cmd,
    stable_points, transform)
from .WaypointTaskServerBase import TaskResult, WaypointTaskServerBase

APPROACH_M = 1.4          # Nav2 brengt het midden van de robot zover voor het kar-midden
STAGE_M = 1.0             # daar precies op de as uitlijnen, dan recht naar binnen
# Rest stuurt het rechtdoor naar binnen rijden bij (1 m tot de kar, convergeert in ~0,3 m).
STAGE_TOL_Y, STAGE_TOL_TH = 0.02, math.radians(1.5)
FINAL_TOL_XY, FINAL_TOL_TH = 0.015, math.radians(1.5)
STAGE_ATTEMPTS = 3
CHECK_MARGIN = 0.06       # nameting: zoveel mag de kar van de vorige meting liggen
SCANS = 30                # stilstaand zoveel scans nemen ...
STABLE_FRACTION = 0.3     # ... en alleen punten houden die in 30% ervan terugkomen
SETTLE_S = 0.5            # na stoppen even wachten voor de eerste scan
LANE_MARGIN = 0.02        # m: punten zo dicht bij de robot geven een waarschuwing
LANE_BLOCK_POINTS = 5     # zoveel punten binnen de robotbreedte = obstakel: niet inrijden
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
    2. Stilstaan en de kar meten in een ROI rond de opgegeven plek (zie CartRoi): de vrije
       rijbaan, en links en rechts daarvan de voor- en achterwielen; het midden daartussen is
       de middellijn. Vanaf dan geldt de gemeten kar-pose (odom-frame), niet meer AMCL.
    3. Precies op de as van de kar zetten, STAGE_M ervoor, met de juiste richting; daar
       opnieuw meten met de punten van beide plekken samen, en zo nodig opnieuw uitlijnen.
    4. Recht naar binnen tot het midden op odometrie; richting bijdraaien. Onder de kar
       ziet de lidar de wielen maar half (rand van het blikveld): daar niet meten.

    Diepte: de lidar ziet alleen de voorkant van de wielen; depth_offset corrigeert dat
    (één keer inmeten).

    Elke meting wordt bewaard in ~/.ros/dock_cart_<tijd>.json (punten, verwachting, uitkomst).

    Vanaf stap 2 tot het einde staat docking/active op True (signal_manager: alarmlichten).
    """

    def __init__(self, node, callback_group=None):
        super().__init__('dock_cart')
        self._node = node
        self._log = node.get_logger()

        def param(name, default):
            return node.declare_parameter(f'dock_cart.{name}', default).value

        # Maten van de kar (standaard de echte kar, ook nagebouwd in de simulatie) en de robot
        # (standaard de gesimuleerde).
        self._cart_half = (param('cart_length', 0.70) / 2, param('cart_width', 0.845) / 2)
        # Midden van de wielpunten ligt zoveel voor het echte midden van de kar (in de
        # rijrichting): de lidar ziet alleen de voorkant van de wielen.
        self._depth_offset = param('depth_offset', 0.055)   # ingemeten in Gazebo (3 proeven)
        self._axle_x = param('axle_x', 0.13)   # wielas t.o.v. base_link: hier draait de robot om
        self._half_width = param('robot_half_width', 0.245)   # buitenkant van de robot
        # base_link eindigt zoveel voorbij het kar-midden (langere robot: verder doorrijden)
        self._extra = param('drive_in_extra', 0.0)
        # Zo ver mag de kar van de opgegeven plek staan (AMCL-fout + fout in de route).
        self._margin = param('search_margin', 0.25)
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
        self._evidence = []        # stabiele punten van alle meetplekken van deze poging (odom)
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
            self._evidence = []
            self._cart = self._measure(compose(odom_map, target), self._margin)
            for attempt in range(1, STAGE_ATTEMPTS + 1):
                feedback(f'uitlijnen voor de kar (poging {attempt})')
                stage = (self._axle_x - STAGE_M, 0.0)   # wielas STAGE_M voor de gedockte plek
                self._run(lambda: point_cmd(self._axle(), transform(self._cart, stage)))
                self._run(lambda: self._rotate_cmd())
                self._cart = self._measure(self._cart, CHECK_MARGIN)
                _, y, th = self._rel(self._axle())
                self._log.info(f'Uitgelijnd voor de kar: y={1000 * y:+.1f} mm, '
                               f'hoek={math.degrees(th):+.2f} graden')
                if abs(y) <= STAGE_TOL_Y and abs(th) <= STAGE_TOL_TH:
                    break
            else:
                # Echte robot: meting en regelaar halen de tolerantie niet altijd terwijl hij er
                # goed voor staat. Gewoon inrijden: axis_cmd stuurt onderweg naar de middellijn;
                # de rijbaancontrole hieronder blijft.
                self._log.warn(f'Uitlijnen niet binnen {1000 * STAGE_TOL_Y:.0f} mm / '
                               f'{math.degrees(STAGE_TOL_TH):.1f} graden na {STAGE_ATTEMPTS} '
                               'pogingen; toch inrijden')
            # Iets in de rijbaan onder de kar (naar binnen gedraaid wiel, beugel, iets op de
            # vloer)? Direct op de scanpunten, niet op de geschatte wielen.
            # Alleen onder de kar zelf: verder dan de achterkant van de kar komt de robot niet.
            # Tot de achterkant van de kar plus drive_in_extra (daar komt base_link).
            # Stoppen alleen bij een echt obstakel: minstens LANE_BLOCK_POINTS punten binnen de
            # robotbreedte (naar binnen gedraaid wiel). Losse randpunten van wielbeugels of
            # punten in de LANE_MARGIN: alleen waarschuwen (echte robot: wielen ~32 cm uit het
            # midden, robot 27,5 cm, dus weinig speling).
            blocking = points_in_lane(self._evidence, self._cart, self._cart_half[0] + self._extra,
                                      self._half_width)
            if len(blocking) >= LANE_BLOCK_POINTS:
                x, y, _ = self._rel((*blocking[0], 0.0))
                raise _Abort(f'iets in de rijbaan onder de kar ({len(blocking)} scanpunten, '
                             f'o.a. kar-frame x {x:+.2f}, y {y:+.2f}): naar binnen gedraaid '
                             'zwenkwiel? Rechtzetten en opnieuw proberen')
            near = points_in_lane(self._evidence, self._cart, self._cart_half[0] + self._extra,
                                  self._half_width + LANE_MARGIN)
            if near:
                ys = sorted(f'{100 * self._rel((*p, 0.0))[1]:+.1f}' for p in near)
                self._log.warn(f'{len(near)} scanpunt(en) dicht bij de rijbaan (kar-frame y in cm: '
                               f'{", ".join(ys[:8])}); toch inrijden')
            feedback('onder de kar rijden')
            self._run(lambda: axis_cmd(self._rel(self._axle()), self._axle_x + self._extra))
            self._run(lambda: self._rotate_cmd())
            x, y, th = self._rel(self._base())
            x -= self._extra
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

    def _measure(self, prior, margin):
        """Stilstaand SCANS scans nemen, de stabiele punten erbij nemen (samen met die van
        eerdere meetplekken) en de kar meten in de ROI: kar plus margin rond prior (odom).
        Geeft de pose van de kar in odom."""
        self._stop()
        time.sleep(SETTLE_S)
        with self._lock:
            self._scans = []
        deadline = time.monotonic() + 10.0
        while True:
            self._check_cancel()
            with self._lock:
                if len(self._scans) >= SCANS:
                    scans, self._scans = self._scans, None
                    break
            if time.monotonic() > deadline:
                raise _Abort('geen lidarscans')
            time.sleep(LOOP_S)

        per_scan = []
        for scan in scans:
            try:
                sensor = _pose_2d(self._tf.lookup_transform(
                    'odom', scan.header.frame_id, Time.from_msg(scan.header.stamp),
                    timeout=Duration(seconds=0.5)))
            except TransformException as e:
                raise _Abort(f'geen odom -> lidar ({e})')
            points = []
            for i, r in enumerate(scan.ranges):
                if scan.range_min < r < min(scan.range_max, MAX_RANGE_M):
                    a = scan.angle_min + i * scan.angle_increment
                    points.append(transform(sensor, (r * math.cos(a), r * math.sin(a))))
            per_scan.append(points)
        self._evidence += stable_points(per_scan, min_fraction=STABLE_FRACTION)

        result, why = measure_cart(self._evidence, prior, *self._cart_half, margin,
                                   2 * self._half_width, depth_offset=self._depth_offset)
        path = self._save(prior, margin, result)
        self._show(prior, margin, result)
        if result is None:
            raise _Abort(f'kar niet gevonden binnen {100 * margin:.0f} cm van de verwachte plek: '
                         f'{why}. Meting bewaard: {path}')
        pose = result['pose']
        e = compose(inverse(prior), pose)
        self._log.info(
            f"Kar gemeten: punten per wiel {result['counts']}, rijbaan "
            f"{100 * result['lane']:.0f} cm; {100 * e[0]:+.1f}/{100 * e[1]:+.1f} cm, "
            f"{math.degrees(e[2]):+.1f} graden van de verwachting. Meting bewaard: {path}")
        return pose

    def _show(self, prior, margin, result):
        """Markers op dock_cart/markers (odom): ROI (geel), midden van de punten per wiel
        (oranje), gemeten kar (groen)."""
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

        outline(prior, self._cart_half[0] + margin, self._cart_half[1] + margin,
                (1.0, 0.8, 0.0), 'roi')
        if result is not None:
            m = marker(Marker.SPHERE_LIST, (1.0, 0.4, 0.0), 0.05, 'wielen')
            m.points = [Point(x=x, y=y, z=0.10) for x, y in result['groups']]
            outline(result['pose'], *self._cart_half, (0.0, 0.9, 0.0), 'kar')
        self._markers.publish(MarkerArray(markers=markers))

    def _save(self, prior, margin, result):
        """Punten, verwachting en uitkomst als JSON in ~/.ros, om metingen na te kijken."""
        folder = os.environ.get('ROS_HOME', os.path.expanduser('~/.ros'))
        path = os.path.join(folder, time.strftime('dock_cart_%Y%m%d_%H%M%S.json'))
        try:
            os.makedirs(folder, exist_ok=True)
            with open(path, 'w') as f:
                json.dump({'prior': prior, 'margin': margin, 'points': self._evidence,
                           'cart_half': self._cart_half, 'depth_offset': self._depth_offset,
                           'result': result}, f)
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
