#!/usr/bin/env python3
"""HMIBridgeNode: koppeling tussen ROS en de HMI (FT2J) via MQTT.

Alles wat over MQTT gaat zit in een eigen klasse:
  hmi_bridge/inputs/   MqttInput:  één MQTT-topic -> iets in ROS (knoppen, EZ-Way-antwoord)
  hmi_bridge/outputs/  MqttOutput: iets uit ROS -> MQTT-topics (lampen, buzzer, status, batterij)
Elke klasse maakt in setup(ctx) zijn eigen ROS-subscriptions/clients en parameters (ctx = deze
node). Iets nieuws over MQTT = een nieuwe klasse en die hieronder in INPUTS of OUTPUTS zetten.

De node zelf: MQTT-verbinding, berichten naar de juiste input, uitgangen versturen bij elke
verandering en alles opnieuw elke heartbeat_s (hartslag), en de verbinding bewaken.

Topics (zelfde als de oude mqtt_hmi_bridge; uitgangen en knoppen zijn "1"/"0"):
  robot -> HMI  robot/outputs/lamp_links, lamp_rechts   LampsOutput    (indicators)
                robot/outputs/buzzer                    BuzzerOutput   (buzzer_pattern)
                robot/patrol_state, outputs/state_id    StateOutput    (mission_status,
                                                                        navigation/blocked)
                robot/outputs/batterij                  BatteryOutput  (battery_state)
  HMI -> robot  robot/start                             StartButtonInput  (start_route)
                robot/stop                              StopButtonInput   (cancel_mission)
                robot/kar                               KarButtonInput    (initialpose, start_route)
                robot/manual, robot/service             ModeInput -> EZ-Way amr_manager/request
  EZ-Way        amr_manager/result                      EzWayResultInput  (alleen loggen)

ROS:
  pub  hmi/mqtt_connected  (std_msgs/Bool, transient_local)  false als de MQTT-verbinding
       langer dan mqtt_timeout_s weg is (CommandWatchDog), bij elke verandering

Parameters (van de node; de inputs/outputs hebben er ook, zie hun klasse):
  broker_host       MQTT-broker                                           (localhost)
  broker_port                                                             (1883)
  heartbeat_s       uitgangen opnieuw sturen als er niets verandert        (1.0)
  mqtt_timeout_s    zo lang zonder MQTT-verbinding = weg                   (3.0)
  blink_period_s    LampsOutput: knipperen, helft aan, helft uit           (1.0)
  battery_empty_v   BatteryOutput: spanning bij 0 % (0 = percentage driver) (21.0)
  battery_full_v    BatteryOutput: spanning bij 100 %                     (28.0)
  start_route_name  StartButtonInput: leeg = startknop doet niets         ('')
  kar_route_name    KarButtonInput                                        ('kaas')
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from hmi_bridge.CommandWatchDog import CommandWatchDog
from hmi_bridge.inputs.EzWayResultInput import EzWayResultInput
from hmi_bridge.inputs.KarButtonInput import KarButtonInput
from hmi_bridge.inputs.ModeInput import ModeInput
from hmi_bridge.inputs.StartButtonInput import StartButtonInput
from hmi_bridge.inputs.StopButtonInput import StopButtonInput
from hmi_bridge.outputs.BatteryOutput import BatteryOutput
from hmi_bridge.outputs.BuzzerOutput import BuzzerOutput
from hmi_bridge.outputs.LampsOutput import LampsOutput
from hmi_bridge.outputs.StateOutput import StateOutput


def make_inputs():
    return [StartButtonInput(), StopButtonInput(), KarButtonInput(),
            ModeInput('robot/manual', 'MANUAL'), ModeInput('robot/service', 'SERVICE'),
            EzWayResultInput()]


def make_outputs():
    return [LampsOutput(), BuzzerOutput(), StateOutput(), BatteryOutput()]


class HMIBridgeNode(Node):

    def __init__(self):
        super().__init__('hmi_bridge')
        host = self.declare_parameter('broker_host', 'localhost').value
        port = self.declare_parameter('broker_port', 1883).value
        self.heartbeat_s = self.declare_parameter('heartbeat_s', 1.0).value
        self.watchdog = CommandWatchDog(self.declare_parameter('mqtt_timeout_s', 3.0).value)

        self.inputs = {i.topic: i for i in make_inputs()}
        self.outputs = make_outputs()
        for part in (*self.inputs.values(), *self.outputs):
            part.setup(self)

        self._sent = {}                 # topic -> laatst verstuurde payload
        self._sent_time = None          # laatste hartslag
        self._connected = None          # laatst gepubliceerde hmi/mqtt_connected
        self.connected_pub = self.create_publisher(
            Bool, 'hmi/mqtt_connected',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        from hmi_bridge.MqttBackend import MqttBackend   # paho alleen nodig voor MQTT
        self.mqtt = MqttBackend(host, port, self.inputs, self.get_logger())
        self.mqtt.connect()
        self.create_timer(0.05, self.update)    # 50 ms: fijn genoeg voor de buzzerpatronen
        self.get_logger().info(f'HMI-bridge gestart (MQTT {host}:{port})')

    # ── voor de inputs en outputs (ctx) ──────────────────────────────────────

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def mqtt_publish(self, topic: str, payload: bytes):
        self.mqtt.publish(topic, payload)

    def call(self, client, request):
        """Service aanroepen zonder te wachten; het antwoord komt in het log."""
        if not client.service_is_ready():
            self.get_logger().error(f'Service {client.srv_name} niet beschikbaar')
            return
        client.call_async(request).add_done_callback(
            lambda f: self.get_logger().info(f'{client.srv_name}: {f.result().message}'))

    # ── lus ──────────────────────────────────────────────────────────────────

    def update(self):
        now = self.now()
        for topic, payload in self.mqtt.receive():
            self.get_logger().debug(f'MQTT ontvangen: {topic} = {payload!r}')
            try:
                self.inputs[topic].on_message(payload)
            except Exception as e:      # een raar bericht van buiten mag de bridge niet stoppen
                self.get_logger().error(f'MQTT-bericht {topic} = {payload!r}: {e!r}')

        connected = self.mqtt.is_connected()
        if connected:
            heartbeat = self._sent_time is None or now - self._sent_time >= self.heartbeat_s
            for output in self.outputs:
                for topic, payload in output.payloads(now).items():
                    if heartbeat or self._sent.get(topic) != payload:
                        self.mqtt.publish(topic, payload)
                        self._sent[topic] = payload
            if heartbeat:
                self._sent_time = now
            self.watchdog.feed(now)

        alive = not self.watchdog.expired(now)
        if alive != self._connected:
            self._connected = alive
            self.connected_pub.publish(Bool(data=alive))
            # Twee aparte aanroepen: rclpy staat geen wisselend niveau op één logregel toe.
            if alive:
                self.get_logger().info('MQTT-verbinding met de HMI-broker in orde')
            else:
                self.get_logger().warn('Geen MQTT-verbinding met de HMI-broker')

    def destroy_node(self):
        self.mqtt.disconnect()
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
