#!/usr/bin/env python3
"""MissionManagerNode: ROS-schil rond de MissionStateMachine.

Vertaalt services, topics en Nav2-resultaten naar MissionEvents en voert de
commands uit die de toestandsmachine teruggeeft via de MissionPorts.

Interface:
  service  start_route     (my_bot_interfaces/srv/StartRoute)  route starten op naam
  service  cancel_mission  (std_srvs/srv/Trigger)              missie afbreken / resetten
  topic    bt_state        (std_msgs/msg/String)               'BLOCKED' of 'DRIVING' vanuit de BT
  topic    mission_status  (std_msgs/msg/String)               toestand naar de UI
  client   get_route       (my_bot_interfaces/srv/GetRoute)    route_manager
  action   navigate_through_poses (nav2_msgs)
"""
import threading
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
from std_srvs.srv import Trigger

from my_bot_interfaces.srv import StartRoute
from mission_manager import MissionEvent, MissionStateMachine
from mission_manager.RouteServiceClient import RouteServiceClient
from mission_manager.enums import MissionKind
from mission_manager.ports import MissionPorts
from mission_manager.ports.Nav2ActionClient import Nav2ActionClient
from mission_manager.ports.Ros2Notifier import Ros2Notifier

BT_STATES = {'BLOCKED': MissionKind.BT_BLOCKED, 'DRIVING': MissionKind.BT_DRIVING}


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager')
        self.state_machine = MissionStateMachine()
        self.mission_ports = None
        self.route_client = None
        self._active = False
        self._lock = threading.RLock()   # callbacks komen uit meerdere threads
        self._queue = deque()
        self._executing = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Reentrant: start_route wacht op get_route terwijl die callback ook moet draaien.
        cb = ReentrantCallbackGroup()
        self.route_client = RouteServiceClient(self, cb)
        self.mission_ports = MissionPorts(
            nav=Nav2ActionClient(self, self.on_nav_result, cb),
            ui=Ros2Notifier(self))
        self.create_service(StartRoute, 'start_route', self.on_start_route, callback_group=cb)
        self.create_service(Trigger, 'cancel_mission', self.on_cancel, callback_group=cb)
        self.create_subscription(String, 'bt_state', self.on_bt_state, 10, callback_group=cb)
        self.get_logger().info('Geconfigureerd')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._active = True
        self.get_logger().info('Actief — wacht op start_route')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        self._dispatch(MissionEvent(MissionKind.RESET))   # rijdende robot stoppen
        return super().on_deactivate(state)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def on_start_route(self, req, res):
        name = req.route_name
        if not self._active:
            return self._reply(res, False, 'MissionManager is niet actief')
        if (self.state_machine.state, MissionKind.START_ROUTE) not in self.state_machine.transitions:
            return self._reply(res, False, f'Kan niet starten in {self.state_machine.state.name}')

        route = self.route_client.get(name)   # blokkeert, buiten de lock
        if route is None:
            return self._reply(res, False, f'Route "{name}" niet beschikbaar')
        if not route.waypoints:
            return self._reply(res, False, f'Route "{name}" heeft geen waypoints')

        if not self._dispatch(MissionEvent(MissionKind.START_ROUTE, name, route)):
            return self._reply(res, False, f'Kan niet starten in {self.state_machine.state.name}')
        return self._reply(res, True, f'Route "{name}" gestart ({len(route.waypoints)} waypoints)')

    def on_cancel(self, req, res):
        res.success = self._dispatch(MissionEvent(MissionKind.RESET))
        res.message = self.state_machine.state.name
        return res

    def on_bt_state(self, msg: String):
        kind = BT_STATES.get(msg.data.strip().upper())
        if kind is None:
            self.get_logger().warn(f'Onbekende bt_state "{msg.data}"', throttle_duration_sec=5.0)
            return
        self._dispatch(MissionEvent(kind))

    def on_nav_result(self, future):
        """future is None als het doel niet verstuurd of geweigerd is."""
        succeeded = future is not None and future.result().status == GoalStatus.STATUS_SUCCEEDED
        if not succeeded and future is not None:
            self.get_logger().warn(f'Nav2 doel mislukt (status {future.result().status})')
        kind = MissionKind.NAV_SUCCEEDED if succeeded else MissionKind.NAV_ABORTED
        self._dispatch(MissionEvent(kind))

    # ── Intern ────────────────────────────────────────────────────────────────

    def _dispatch(self, event: MissionEvent) -> bool:
        """Event naar de toestandsmachine en de commands uitvoeren. Een command
        kan zelf een event veroorzaken (bv. Nav2 niet bereikbaar); die commands
        komen achteraan de rij, zodat de UI de toestanden in de juiste volgorde krijgt."""
        with self._lock:
            old = self.state_machine.state
            commands = self.state_machine.handle(event)
            if commands is None:
                self.get_logger().debug(f'{event.kind.name} genegeerd in {old.name}')
                return False
            self.get_logger().info(
                f'[MISSIE] {old.name} --{event.kind.name}--> {self.state_machine.state.name}')
            self._queue.extend(commands)
            if not self._executing:
                self._executing = True
                try:
                    while self._queue:
                        self._queue.popleft().execute(self.mission_ports)
                finally:
                    self._executing = False
            return True

    @staticmethod
    def _reply(res, accepted: bool, message: str):
        res.accepted = accepted
        res.message = message
        return res


def main():
    rclpy.init()
    node = MissionManagerNode()
    # Zelf configureren en activeren; haal dit weg als een lifecycle_manager dat doet.
    node.trigger_configure()
    node.trigger_activate()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
