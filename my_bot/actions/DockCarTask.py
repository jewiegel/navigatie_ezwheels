#!/usr/bin/env python3

import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from docking import (
    axis_cmd, blend, cart_legs, compose, find_leg_candidates, inverse, match_cart, point_cmd,
    rotate_cmd, transform)
from .WaypointTaskServerBase import TaskResult, WaypointTaskServerBase

APPROACH_M = 1.4          # Nav2 brengt het midden van de robot zover voor het kar-midden
STAGE_M = 1.0             # daar precies op de as uitlijnen, dan recht naar binnen
STAGE_TOL_Y, STAGE_TOL_TH = 0.01, math.radians(1.0)
FINAL_TOL_XY, FINAL_TOL_TH = 0.015, math.radians(1.5)
STAGE_ATTEMPTS = 3
GATE_SEARCH, GATE_TRACK = 0.4, 0.12   # m rond de verwachte poot: AMCL-fout / na vinden
ALPHA = 0.2               # gewicht van een nieuwe scan in de geschatte kar-pose
CART_TIMEOUT_S = 2.0      # zo lang geen poten gezien: stoppen en wachten
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
    2. De vier poten zoeken met de lidar rond de verwachte plek; vanaf dan volgt de kar-pose
       uit de poten (odom-frame, gemiddeld over scans), niet meer uit AMCL.
    3. Precies op de as van de kar zetten, STAGE_M ervoor, met de juiste richting.
    4. Recht naar binnen tot het midden, onderweg bijsturen op de poten; richting bijdraaien.

    Vanaf stap 2 tot het einde staat docking/active op True (signal_manager: alarmlichten).
    """

    def __init__(self, node, callback_group=None):
        super().__init__('dock_cart')
        self._node = node
        self._log = node.get_logger()

        def param(name, default):
            return node.declare_parameter(f'dock_cart.{name}', default).value

        # Maten van de kar (poten midden op de wielen) en de robot; standaard die van de simulatie.
        self._legs = cart_legs(param('leg_x', 0.29), param('leg_y', 0.3625))
        self._leg_r = param('leg_r', 0.02)
        self._axle_x = param('axle_x', 0.13)   # wielas t.o.v. base_link: hier draait de robot om
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node)
        self._cmd = node.create_publisher(Twist, 'cmd_vel', 10)
        self._docking = node.create_publisher(
            Bool, 'docking/active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._docking.publish(Bool(data=False))
        self._nav = ActionClient(node, NavigateToPose, 'navigate_to_pose',
                                 callback_group=callback_group)
        node.create_subscription(LaserScan, '/laser_1/scan_filtered', self._on_scan, 10,
                                 callback_group=callback_group)
        self._lock = threading.Lock()
        self._cart = None          # geschatte pose van de kar (= doelpose) in odom
        self._locked = False       # kar al eens gevonden: kleine zoekstraal
        self._seen = 0.0           # wandkloktijd van de laatste scan met poten
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
            self._start_tracking(target)
            for attempt in range(1, STAGE_ATTEMPTS + 1):
                feedback(f'uitlijnen voor de kar (poging {attempt})')
                stage = (self._axle_x - STAGE_M, 0.0)   # wielas STAGE_M voor de gedockte plek
                self._run(lambda: point_cmd(self._axle(), self._in_cart(stage)))
                self._run(lambda: self._rotate_cmd())
                _, y, th = self._rel(self._axle())
                self._log.info(f'Uitgelijnd voor de kar: y={1000 * y:+.1f} mm, '
                               f'hoek={math.degrees(th):+.2f} graden')
                if abs(y) <= STAGE_TOL_Y and abs(th) <= STAGE_TOL_TH:
                    break
            else:
                raise _Abort('lukt niet om recht voor de kar te komen')
            feedback('onder de kar rijden')
            self._run(lambda: axis_cmd(self._rel(self._axle()), self._axle_x))
            self._run(lambda: self._rotate_cmd())
            time.sleep(1.0)    # stilstaan en nog een paar scans middelen voor de eindmeting
            x, y, th = self._rel(self._base())
            message = (f'gedockt: afwijking x={1000 * x:+.1f} mm, y={1000 * y:+.1f} mm, '
                       f'hoek={math.degrees(th):+.2f} graden')
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

    def _start_tracking(self, target_map):
        try:
            odom_map = _pose_2d(self._tf.lookup_transform('odom', 'map', Time()))
        except TransformException as e:
            raise _Abort(f'geen map -> odom ({e})')
        with self._lock:
            self._cart = compose(odom_map, target_map)   # verwachting uit AMCL
            self._locked = False
        deadline = time.monotonic() + 5.0
        while not self._locked:
            self._check_cancel()
            if time.monotonic() > deadline:
                raise _Abort('kar niet gevonden: geen drie poten op de verwachte plek')
            time.sleep(LOOP_S)
        time.sleep(1.0)   # eerst een paar scans middelen

    def _run(self, command):
        """command() geeft (v, w) of None (klaar); herhalen tot klaar. Stopt en wacht als de
        poten even niet gezien worden."""
        while True:
            self._check_cancel()
            if time.monotonic() - self._seen > CART_TIMEOUT_S:
                self._stop()
                deadline = time.monotonic() + 5.0
                while time.monotonic() - self._seen > CART_TIMEOUT_S:
                    self._check_cancel()
                    if time.monotonic() > deadline:
                        raise _Abort('kar uit het zicht geraakt')
                    time.sleep(LOOP_S)
            out = command()
            if out is None:
                self._stop()
                return
            self._publish(*out)
            time.sleep(LOOP_S)

    def _rotate_cmd(self):
        w = rotate_cmd(self._rel(self._axle())[2])
        return None if w is None else (0.0, w)

    # ── Kar volgen met de lidar ───────────────────────────────────────────────

    def _on_scan(self, scan: LaserScan):
        with self._lock:
            cart, locked = self._cart, self._locked
        if cart is None:
            return
        try:
            sensor = _pose_2d(self._tf.lookup_transform(
                'odom', scan.header.frame_id, Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.1)))
        except TransformException:
            return
        points = []
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < min(scan.range_max, MAX_RANGE_M):
                a = scan.angle_min + i * scan.angle_increment
                points.append(transform(sensor, (r * math.cos(a), r * math.sin(a))))
        # Op 1,5 m vallen er maar 2-4 stralen op een poot.
        centers = find_leg_candidates(points, sensor[:2], self._leg_r, min_points=2)
        match = match_cart(centers, cart, self._legs, GATE_TRACK if locked else GATE_SEARCH)
        if match is None or (not locked and match[1] < 3):
            return   # de eerste keer drie poten eisen: dan is het zeker de kar
        with self._lock:
            if self._cart is None:
                return
            self._cart = blend(self._cart, match[0], ALPHA) if self._locked else match[0]
            self._locked = True
            self._seen = time.monotonic()

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
        with self._lock:
            cart = self._cart
        return compose(inverse(cart), pose)

    def _in_cart(self, point):
        with self._lock:
            cart = self._cart
        return transform(cart, point)

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
