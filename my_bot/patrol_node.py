#!/usr/bin/env python3
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import math

from navigation_states import (
    IdleState, DrivingState, WaitingState, CompletedState, StoppedState, ErrorState)

WACHT_SECONDEN  = 120  # 2 minuten wachten bij blokkade
NAV_TIMEOUT_SEC = 60   # seconden per waypoint zonder voortgang → fout
PERSOON_WACHT_SECONDEN = 120
BACKUP_SPEED    = 0.1
BACKUP_DISTANCE = 0.5   # m — afstand achteruit na de 2-min wachttijd (was 0.3)
BACKUP_CMD_VEL_TOPIC = '/cmd_vel'
RENAV_PAUSE     = 2.0   # s — pauze tussen oud doel annuleren en nieuw doel sturen


class PatrolNode(Node):
    """NavigationContext: houdt de actieve NavigationState vast."""

    def __init__(self):
        super().__init__('patrol_node')

        self._states = {cls: cls(self) for cls in (
            IdleState, DrivingState, WaitingState,
            CompletedState, StoppedState, ErrorState)}
        self._state = self._states[IdleState]
        self._nav_timeout_sec = NAV_TIMEOUT_SEC   # gebruikt door DrivingState.on_tick

        # Hoofdnavigatie: NavigateThroughPoses (alle waypoints in één goal).
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        # Terugkeer: NavigateToPose (één waypoint)
        self._return_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._state_pub     = self.create_publisher(String, '/patrol_state', 10)
        self._buzzer_pub    = self.create_publisher(Bool,   '/buzzer',       10)
        self._indicator_pub = self.create_publisher(String, '/indicators',   10)

        planner_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._planner_pub = self.create_publisher(String, '/planner_selector', planner_qos)

        self.create_subscription(Bool, '/start_patrol',   self._on_start,   10)
        self.create_subscription(Bool, '/start_patrol2', self._on_start_2, 10)
        self.create_subscription(Bool, '/stop_patrol',    self._on_stop,    10)

        self._select_gridbased_planner()

        # Route 1 — knop 1 → /start_patrol
        self._route1 = [
            self._make_pose(7.66, -3.49, 0.0),
            self._make_pose(5.48, -0.6, 0.0),
            self._make_pose(7.0, 2.9, 0.0),
        ]
        # Route 2 — knop 2 → /start_patrol_2.  PAS DEZE WAYPOINTS AAN naar je tweede route.
        # (placeholder: nu route 1 in omgekeerde volgorde)
        self._route2 = [
            self._make_pose(7.0, 2.9, 0.0),
            self._make_pose(5.48, -0.6, 0.0),
            self._make_pose(7.66, -3.49, 0.0),
            self._make_pose(7.99, 3.32, 0.0),
            ]
        self._waypoints = self._route1   # actieve route; wordt bij elke start gezet

        # Cancel-services: ruim zombie-doelen op van beide action servers.
        self._cancel_ntps_client = self.create_client(
            CancelGoal, '/navigate_through_poses/_action/cancel_goal')
        self._cancel_ntp_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self._ctrl_lifecycle_client = self.create_client(
            ChangeState, '/controller_server/change_state')
        self._pending_start_timer = None

        self._cmdvel_pub        = self.create_publisher(Twist, BACKUP_CMD_VEL_TOPIC, 10)
        self._safety_wait_timer = None
        self._backup_timer      = None
        self._backup_start      = None
        self._backing_up        = False
        self._blockage_level    = 0
        self._beep_timer        = None

        self._current_index       = 0
        self._route_start_index   = 0  # index waarbij de actieve NavigateThroughPoses begon
        self._last_successful_idx = 0
        self._goal_handle         = None
        self._goal_seq            = 0  # volgnummer: resultaten van oude doelen negeren we
        self._stopped             = False
        self._wait_timer          = None
        self._trying_alternative  = False
        self._driving_since       = None

        self._safety_stopped    = False
        self._cmdvel_zero_since = None
        self._was_moving        = False

        # Veiligheidsfunctionaliteit tijdelijk uitgeschakeld voor debugging
        # self.create_subscription(Odometry, '/odom', self._on_odom_safety, 10)
        # self.create_timer(0.5, self._check_safety_stop)
        self.create_timer(5.0, self._on_tick)   # o.a. nav-timeout in DrivingState

        self._publish_state()
        self.get_logger().info('PatrolNode klaar — wacht op /start_patrol')

    def _select_gridbased_planner(self):
        msg = String()
        msg.data = 'GridBased'
        self._planner_pub.publish(msg)

    # ── State machine ─────────────────────────────────────────────────────────

    def _set_state(self, state_cls):
        old_state = self._state
        old_state.on_leave()
        self._state             = self._states[state_cls]
        self._safety_stopped    = False
        self._cmdvel_zero_since = None
        self._was_moving        = False
        self._publish_state()
        self.get_logger().info(f'[STATE] {old_state.name} → {self._state.name}')
        self._state.on_enter()

    def _publish_state(self):
        msg = String()
        msg.data = self._state.published_name()
        self._state_pub.publish(msg)

    # ── Start / stop ──────────────────────────────────────────────────────────

    def _on_start(self, msg: Bool):
        if msg.data:
            self._begin_route(1, self._route1)

    def _on_start_2(self, msg: Bool):
        if msg.data:
            self._begin_route(2, self._route2)

    def _begin_route(self, route_id: int, waypoints):
        if not self._state.can_start():
            self.get_logger().warn(f'Start genegeerd — robot is momenteel: {self._state.name}')
            return
        # Kies de route en start altijd vanaf het begin (geen voortgang onthouden).
        self._waypoints = waypoints
        self.get_logger().info(f'Startsignaal route {route_id} ontvangen — start bij waypoint 1')
        self._reset_timers_and_flags()   # schone lei: oude timers/achteruit-vlag opruimen
        self._select_gridbased_planner()
        self._current_index       = 0
        self._route_start_index   = 0
        self._last_successful_idx = 0
        self._stopped             = False
        self._trying_alternative  = False
        self._blockage_level      = 0
        self._cancel_all_nav2_goals()

    def _cancel_all_nav2_goals(self):
        """Cancel actieve doelen op beide action servers, daarna controller resetten."""
        pending = [0]

        def _on_done(future, name):
            try:
                n = len(future.result().goals_canceling)
                self.get_logger().info(f'{name}: {n} zombie-doel(en) geannuleerd')
            except Exception as exc:
                self.get_logger().warn(f'CancelGoal {name} fout: {exc}')
            pending[0] -= 1
            if pending[0] <= 0:
                self._reset_controller_server()

        req = CancelGoal.Request()

        if self._cancel_ntps_client.service_is_ready():
            pending[0] += 1
            f = self._cancel_ntps_client.call_async(req)
            f.add_done_callback(lambda fut: _on_done(fut, 'NavigateThroughPoses'))

        if self._cancel_ntp_client.service_is_ready():
            pending[0] += 1
            f = self._cancel_ntp_client.call_async(req)
            f.add_done_callback(lambda fut: _on_done(fut, 'NavigateToPose'))

        if pending[0] == 0:
            self.get_logger().warn('Geen cancel-services beschikbaar — sla cancel over')
            self._reset_controller_server()

    def _reset_controller_server(self):
        if not self._ctrl_lifecycle_client.service_is_ready():
            self.get_logger().warn(
                'controller_server lifecycle-service niet bereikbaar — sla reset over')
            self._schedule_start_navigation()
            return
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.get_logger().info('controller_server deactiveren om interne staat te wissen...')
        future = self._ctrl_lifecycle_client.call_async(req)
        future.add_done_callback(self._on_controller_reset_done)

    def _on_controller_reset_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    'controller_server gedeactiveerd — lifecycle_manager herstart hem')
            else:
                self.get_logger().warn('controller_server deactivate mislukt — toch doorgaan')
        except Exception as exc:
            self.get_logger().warn(f'controller_server lifecycle fout: {exc}')
        self._schedule_start_navigation(delay=8.0)

    def _schedule_start_navigation(self, delay: float = 1.0):
        if self._pending_start_timer is not None:
            self._pending_start_timer.cancel()
        self._pending_start_timer = self.create_timer(delay, self._do_start_navigation)

    def _do_start_navigation(self):
        if self._pending_start_timer is not None:
            self._pending_start_timer.cancel()
            self._pending_start_timer = None
        if self._stopped:
            return
        self.get_logger().info('Doelen verwerkt — route wordt gestart')
        self._navigate_through_remaining()

    def _schedule_return(self, delay: float = RENAV_PAUSE):
        """Plan een terugkeer-poging nadat het oude doel is geannuleerd."""
        if self._pending_start_timer is not None:
            self._pending_start_timer.cancel()
        self._pending_start_timer = self.create_timer(delay, self._do_return)

    def _do_return(self):
        if self._pending_start_timer is not None:
            self._pending_start_timer.cancel()
            self._pending_start_timer = None
        if self._stopped:
            return
        self._return_to_last()

    def _on_stop(self, msg: Bool):
        if msg.data and self._state.can_stop():
            self.get_logger().info('Stopsignaal ontvangen — route wordt onderbroken')
            self._stopped = True
            self._set_state(StoppedState)   # on_enter: timers opruimen + doel annuleren

    def _cancel_goal(self):
        """Annuleer het actieve doel en hoog het volgnummer op, zodat een laat
        binnenkomend (CANCELED/ABORTED) resultaat van dit doel ALTIJD door de
        seq-guard verworpen wordt — ongeacht de huidige state. Voorkomt dat een
        escalatietrap dubbel of overgeslagen wordt."""
        self._goal_seq += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def shutdown_cleanly(self):
        if self._goal_handle is None:
            return
        self.get_logger().info('Afsluiten — annuleer actief Nav2-doel...')
        try:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        except Exception as exc:
            self.get_logger().warn(f'Kon doel niet netjes annuleren: {exc}')
        finally:
            self._goal_handle = None

    def _cancel_wait_timer(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

    # ── Navigatie (alle resterende waypoints in één goal) ──────────────────────

    def _navigate_through_remaining(self):
        """Stuur waypoints vanaf _current_index als één NavigateThroughPoses goal.
        Elk nieuw doel krijgt een nieuw volgnummer; oude doelen negeren we."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Nav2 navigate_through_poses niet beschikbaar na 5 seconden')
            self._set_state(ErrorState)
            return

        remaining_poses = self._waypoints[self._current_index:]
        now = self.get_clock().now().to_msg()
        for p in remaining_poses:
            p.header.stamp = now

        goal = NavigateThroughPoses.Goal()
        goal.poses = remaining_poses
        self._route_start_index = self._current_index

        self._goal_seq += 1
        seq = self._goal_seq

        label = 'alternatief' if self._trying_alternative else 'route'
        self.get_logger().info(
            f'Stuur {label}: {len(remaining_poses)} waypoints '
            f'(waypoint {self._current_index + 1} t/m {len(self._waypoints)})')
        self._set_state(DrivingState)
        self._set_indicator('uit')   # eventuele gevaarslichten uit nu we weer rijden
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda f: self._on_goal_response(f, seq))

    def _on_feedback(self, feedback_msg):
        """Reset de per-waypoint timeout telkens als een waypoint gepasseerd wordt."""
        remaining = feedback_msg.feedback.number_of_poses_remaining
        total_sent = len(self._waypoints) - self._route_start_index
        passed = total_sent - remaining
        new_index = self._route_start_index + passed

        if new_index > self._current_index:
            old = self._current_index
            self._current_index = new_index
            self._last_successful_idx = old
            self._driving_since = self.get_clock().now()  # reset timeout per waypoint
            # Voortgang geboekt → blokkade voorbij: escalatieladder terug naar nul,
            # zodat een vólgend obstakel weer de volledige keten krijgt (wacht → achteruit → ...).
            self._blockage_level     = 0
            self._trying_alternative = False
            self.get_logger().info(
                f'Waypoint {old + 1} gepasseerd → navigeert naar {self._current_index + 1} '
                f'({remaining} resterend)')

    def _on_goal_response(self, future, seq):
        if seq != self._goal_seq:
            return  # verouderd doel — er is al een nieuwer doel gestuurd
        goal_handle = future.result()
        if not goal_handle.accepted:
            retries = getattr(self, '_goal_retries', 0)
            if retries < 1:
                self._goal_retries = retries + 1
                self.get_logger().warn(
                    f'Nav2 heeft het doel geweigerd — retry over 4s (poging {retries + 1}/1)')
                self._schedule_start_navigation(delay=4.0)
            else:
                self._goal_retries = 0
                self.get_logger().error('Nav2 heeft het doel geweigerd (ook na retry)')
                self._on_blocked()
            return
        self._goal_retries = 0
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_result(f, seq))

    def _on_result(self, future, seq):
        # Negeer resultaten van oude doelen of als we gestopt zijn. De rest beslist
        # de toestand zelf: alleen DrivingState handelt het resultaat af (bij het
        # ingaan van 'wachten' annuleren we het doel; dat CANCELED-resultaat mag de
        # blokkade-afhandeling niet opnieuw triggeren).
        if seq != self._goal_seq or self._stopped:
            return
        self._state.on_nav_result(future.result().status)

    # ── Blokkade afhandeling ──────────────────────────────────────────────────

    def _on_blocked(self):
        if self._stopped:
            return
        if self._blockage_level == 0:
            self.get_logger().warn(
                f'Blokkade bij waypoint {self._current_index + 1} — '
                f'wacht {WACHT_SECONDEN // 60} min op vrije doorgang')
            self._set_state(WaitingState)
            self._cancel_goal()   # doel annuleren → robot stopt en staat echt stil tijdens het wachten
            self._wait_timer = self.create_timer(WACHT_SECONDEN, self._on_wacht_voorbij)
        else:
            self._escalate_blockage()

    def _on_wacht_voorbij(self):
        self._cancel_wait_timer()
        if self._stopped:
            return
        self._escalate_blockage()

    def _escalate_blockage(self):
        if self._stopped:
            return

        if self._blockage_level == 0:
            self.get_logger().warn('Blokkade — piep, stukje achteruit en opnieuw proberen')
            self._blockage_level = 1
            self._cancel_goal()                 # zet het huidige pad uit
            self._beep()
            self._set_indicator('gevaar')        # gevaarslichten aan tijdens blokkade/achteruit
            self._start_backup()                 # na backup → vers pad (nieuw volgnummer)

        elif self._blockage_level == 1:
            self._blockage_level = 2
            next_index = self._current_index + 1
            if next_index < len(self._waypoints):
                self.get_logger().warn(
                    f'Nog steeds geblokkeerd — alternatieve route via waypoint {next_index + 1}')
                self._trying_alternative = True
                self._current_index      = next_index
                self._cancel_goal()       # oud pad uitzetten (hoogt _goal_seq op)
                self._set_state(WaitingState)   # robot stilzetten tijdens de pauze
                self._schedule_start_navigation(delay=RENAV_PAUSE)  # daarna vers pad
            else:
                self.get_logger().warn('Geen alternatief — terugkeren naar vorig waypoint')
                self._cancel_goal()
                self._set_state(WaitingState)
                self._schedule_return(delay=RENAV_PAUSE)
        else:
            self.get_logger().warn('Alternatieve route ook geblokkeerd — terugkeren')
            self._cancel_goal()
            self._set_state(WaitingState)
            self._schedule_return(delay=RENAV_PAUSE)

    def _beep(self):
        on = Bool()
        on.data = True
        self._buzzer_pub.publish(on)

        def _stop_beep():
            off = Bool()
            off.data = False
            self._buzzer_pub.publish(off)
            if self._beep_timer is not None:
                self._beep_timer.cancel()
                self._beep_timer = None

        self._beep_timer = self.create_timer(0.3, _stop_beep)

    def _set_indicator(self, state: str):
        """Stuur de knipperlichten direct aan, net als de buzzer via _beep().
        Het bericht gaat via /indicators → mqtt_hmi_bridge → MQTT → HMI (lamp_links/rechts).
        state: 'links', 'rechts', 'gevaar' (beide) of 'uit'."""
        msg = String()
        msg.data = state
        self._indicator_pub.publish(msg)

    def _reset_timers_and_flags(self):
        """Schone lei bij start/stop: alle lopende timers stoppen, vlaggen resetten
        en de wielen stilzetten. Voorkomt dat een oude run of een achteruit-actie
        blijft hangen (bv. _backing_up dat True blijft → permanent vastlopen)."""
        for t in (self._wait_timer, self._pending_start_timer, self._backup_timer,
                  self._beep_timer, self._safety_wait_timer):
            if t is not None:
                t.cancel()
        self._wait_timer         = None
        self._pending_start_timer = None
        self._backup_timer       = None
        self._beep_timer         = None
        self._safety_wait_timer  = None
        self._backing_up         = False
        self._goal_retries       = 0
        self._cmdvel_pub.publish(Twist())

    # ── Terugkeren ────────────────────────────────────────────────────────────

    def _return_to_last(self):
        if not self._return_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 navigate_to_pose niet beschikbaar — kan niet terugkeren')
            self._set_state(ErrorState)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._waypoints[self._last_successful_idx]
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self._goal_seq += 1
        seq = self._goal_seq

        self.get_logger().info(
            f'Terugkeren naar waypoint {self._last_successful_idx + 1}')
        self._set_state(DrivingState)
        future = self._return_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_return_response(f, seq))

    def _on_return_response(self, future, seq):
        if seq != self._goal_seq:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'FOUT: Kan niet terugkeren — operator ingrijpen vereist')
            self._set_state(ErrorState)
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_return_result(f, seq))

    def _on_return_result(self, future, seq):
        if seq != self._goal_seq:
            return
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                'FOUT: Route mislukt — robot teruggekeerd — operator ingrijpen vereist')
        else:
            self.get_logger().error(
                'FOUT: Route mislukt — kon ook niet terugkeren — operator ingrijpen vereist')
        self._set_state(ErrorState)

    # ── EZ-Wheel veiligheidsstop monitoring ───────────────────────────────────

    def _on_odom_safety(self, msg: Odometry):
        vx    = msg.twist.twist.linear.x
        vy    = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        speed = math.sqrt(vx**2 + vy**2)
        moving = speed > 0.01 or abs(omega) > 0.03

        if not moving:
            if self._was_moving and self._cmdvel_zero_since is None:
                self._cmdvel_zero_since = self.get_clock().now()
        else:
            self._was_moving = True
            self._cmdvel_zero_since = None
            if self._safety_stopped:
                self._safety_stopped = False
                self._cancel_safety_wait_timer()
                self._publish_state()
                self.get_logger().info('EZ-Wheel veiligheidsstop opgeheven — robot rijdt verder')

    def _check_safety_stop(self):
        if not isinstance(self._state, DrivingState) or self._safety_stopped:
            return
        if self._cmdvel_zero_since is None:
            return
        stil_sec = (self.get_clock().now() - self._cmdvel_zero_since).nanoseconds / 1e9
        if stil_sec > 15.0:
            self._safety_stopped = True
            self._publish_state()
            self.get_logger().info(
                'EZ-Wheel veiligheidsstop actief — robot gestopt door persoon/object')
            if self._safety_wait_timer is None and not self._backing_up:
                self._safety_wait_timer = self.create_timer(
                    PERSOON_WACHT_SECONDEN, self._on_person_wait_voorbij)

    def _on_tick(self):
        self._state.on_tick()

    # ── Achteruit rijden ──────────────────────────────────────────────────────

    def _cancel_safety_wait_timer(self):
        if self._safety_wait_timer is not None:
            self._safety_wait_timer.cancel()
            self._safety_wait_timer = None

    def _on_person_wait_voorbij(self):
        self._cancel_safety_wait_timer()
        if self._stopped or not self._safety_stopped:
            return
        self.get_logger().warn(
            f'Persoon blokkeert al {PERSOON_WACHT_SECONDEN // 60} min — escaleren')
        self._escalate_blockage()

    def _start_backup(self):
        self._backing_up   = True
        self._backup_start = self.get_clock().now()
        self._backup_timer = self.create_timer(0.1, self._backup_step)

    def _backup_step(self):
        duur       = BACKUP_DISTANCE / BACKUP_SPEED
        verstreken = (self.get_clock().now() - self._backup_start).nanoseconds / 1e9
        twist      = Twist()
        if verstreken < duur and not self._stopped:
            twist.linear.x = -BACKUP_SPEED
            self._cmdvel_pub.publish(twist)
            return
        twist.linear.x = 0.0
        self._cmdvel_pub.publish(twist)
        if self._backup_timer is not None:
            self._backup_timer.cancel()
            self._backup_timer = None
        self._backing_up = False
        if not self._stopped:
            self.get_logger().info('Achteruit klaar — vers pad sturen')
            self._navigate_through_remaining()

    # ── Hulpfuncties ──────────────────────────────────────────────────────────

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    node = PatrolNode()
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanly()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
