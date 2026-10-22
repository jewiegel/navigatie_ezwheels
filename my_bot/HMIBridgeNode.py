#!/usr/bin/env python3
"""HMIBridgeNode: koppeling tussen ROS en de HMI (FT2J) via een SignalBackend (MQTT).

Gebruikt dezelfde MQTT-topics als de oude mqtt_hmi_bridge (zie hmi_bridge/PayloadCodec.py):
lampen en buzzer als "1"/"0", de status als tekst, start/stop-knoppen als "1".
Robot -> HMI gaat bij elke verandering en elke heartbeat_s (hartslag).

Interface:
  sub  indicators      (std_msgs/String)  knipperlichten van de signal_manager; de bridge
       laat de lampen knipperen (blink_period_s)
  sub  buzzer_pattern  (std_msgs/String)  buzzerpatroon van de signal_manager; de bridge
       speelt het af op de aan/uit-uitgang (BuzzerPlayer)
  sub  mission_status  (std_msgs/String)  missietoestand van de MissionManager
  pub  hmi/mqtt_connected  (std_msgs/Bool, transient_local)  false als de MQTT-verbinding
       langer dan mqtt_timeout_s weg is (CommandWatchDog), bij elke verandering
  cli  start_route     (my_bot_interfaces/StartRoute)  startknop -> route start_route_name
  cli  cancel_mission  (std_srvs/Trigger)              stopknop

Parameters:
  backend           'mqtt', of 'memory' zonder broker (tests, simulatie)   (mqtt)
  broker_host       MQTT-broker                                           (localhost)
  broker_port                                                             (1883)
  heartbeat_s       uitgangen opnieuw sturen als er niets verandert        (1.0)
  blink_period_s    knipperen: helft aan, helft uit                       (1.0)
  mqtt_timeout_s    zo lang zonder MQTT-verbinding = weg                   (3.0)
  start_route_name  route voor de startknop; leeg = startknop doet niets  ('')
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from hmi_bridge.BuzzerPlayer import BuzzerPlayer
from hmi_bridge.CommandWatchDog import CommandWatchDog
from hmi_bridge.HMIMessages import SignalCommand
from hmi_bridge.InMemoryBackend import InMemoryBackend
from hmi_bridge.SignalBackend import SignalBackend
from my_bot_interfaces.srv import StartRoute


class HMIBridgeNode(Node):

    def __init__(self):
        super().__init__('hmi_bridge')
        backend = self.declare_parameter('backend', 'mqtt').value
        host = self.declare_parameter('broker_host', 'localhost').value
        port = self.declare_parameter('broker_port', 1883).value
        self.heartbeat_s = self.declare_parameter('heartbeat_s', 1.0).value
        self.blink_period_s = self.declare_parameter('blink_period_s', 1.0).value
        mqtt_timeout_s = self.declare_parameter('mqtt_timeout_s', 3.0).value
        self.start_route_name = self.declare_parameter('start_route_name', '').value

        if backend == 'memory':
            self.backend: SignalBackend = InMemoryBackend()
        else:
            from hmi_bridge.MqttBackend import MqttBackend   # paho alleen nodig voor MQTT
            self.backend = MqttBackend(host, port, self.get_logger())
        self.watchdog = CommandWatchDog(mqtt_timeout_s)
        self.buzzer = BuzzerPlayer()

        self._blinkers = 'uit'
        self._blink_since = 0.0         # begin van het huidige knipperlicht: start met aan
        self._mission = ''
        self._sent_command = None       # laatst verstuurde SignalCommand
        self._sent_time = None
        self._status = None             # laatst verwerkte knopdruk
        self._connected = None          # laatst gepubliceerde hmi/mqtt_connected

        self.create_subscription(String, 'indicators', self.on_indicators, 10)
        self.create_subscription(String, 'buzzer_pattern', self.on_buzzer, 10)
        self.create_subscription(String, 'mission_status', self.on_mission, 10)
        self.connected_pub = self.create_publisher(
            Bool, 'hmi/mqtt_connected',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.start_client = self.create_client(StartRoute, 'start_route')
        self.cancel_client = self.create_client(Trigger, 'cancel_mission')

        self.backend.connect()
        self.create_timer(0.05, self.update)    # 50 ms: fijn genoeg voor de buzzerpatronen
        self.get_logger().info(f'HMI-bridge gestart ({backend}: {host}:{port})')

    # ── ROS -> HMI ───────────────────────────────────────────────────────────

    def on_indicators(self, msg: String):
        if msg.data != self._blinkers:
            self._blinkers, self._blink_since = msg.data, self._now()

    def on_buzzer(self, msg: String):
        try:
            self.buzzer.play(msg.data, self._now())
        except ValueError as e:
            self.get_logger().warn(f'Buzzer: {e}')

    def on_mission(self, msg: String):
        self._mission = msg.data

    def update(self):
        now = self._now()
        blink_on = (now - self._blink_since) % self.blink_period_s < self.blink_period_s / 2
        command = SignalCommand(self._blinkers, blink_on, self.buzzer.is_on(now), self._mission)
        if self.backend.is_connected() and (
                command != self._sent_command or self._sent_time is None or
                now - self._sent_time >= self.heartbeat_s):
            self.backend.send(command)
            self._sent_command, self._sent_time = command, now

        status = self.backend.latest_status()
        if status is not None and status is not self._status:
            self._status = status
            self._handle_button(status)

        if self.backend.is_connected():
            self.watchdog.feed(now)
        connected = not self.watchdog.expired(now)
        if connected != self._connected:
            self._connected = connected
            self.connected_pub.publish(Bool(data=connected))
            # Twee aparte aanroepen: rclpy staat geen wisselend niveau op één logregel toe.
            if connected:
                self.get_logger().info('MQTT-verbinding met de HMI-broker in orde')
            else:
                self.get_logger().warn('Geen MQTT-verbinding met de HMI-broker')

    # ── HMI -> ROS ───────────────────────────────────────────────────────────

    def _handle_button(self, status):
        if status.stop:
            self.get_logger().info('HMI: stopknop')
            self._call(self.cancel_client, Trigger.Request())
        if status.start:
            if not self.start_route_name:
                self.get_logger().warn('HMI: startknop, maar start_route_name is leeg')
                return
            self.get_logger().info(f'HMI: startknop -> route "{self.start_route_name}"')
            self._call(self.start_client, StartRoute.Request(route_name=self.start_route_name))

    def _call(self, client, request):
        if not client.service_is_ready():
            self.get_logger().error(f'Service {client.srv_name} niet beschikbaar')
            return
        client.call_async(request).add_done_callback(
            lambda f: self.get_logger().info(f'{client.srv_name}: {f.result().message}'))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self):
        self.backend.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = HMIBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
