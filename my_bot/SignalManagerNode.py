#!/usr/bin/env python3
"""SignalManagerNode: stuurt knipperlichten en claxon aan op basis van de navigatie.

sources (invoer) -> RobotFacts -> actuators (Blinkers, Horn) -> uitgangen.

Interface:
  sub  turn                (my_bot_interfaces/Turn)            TurnSource
  sub  mission_status      (std_msgs/String)                   RobotStateSource
  sub  navigation/blocked  (std_msgs/Bool, transient_local)    CollisionStateSource
  pub  indicators          (std_msgs/String)  'links', 'rechts', 'gevaar' of 'uit', bij elke
       verandering; mqtt_hmi_bridge zet dit om naar de lampen
  pub  buzzer              (std_msgs/Bool)    piepjes van Horn (True..False);
       mqtt_hmi_bridge zet dit om naar de buzzer

Parameters:
  hold_s          zo lang brandt het knipperlicht na als er geen bocht meer is  (1.0)
  turn_timeout_s  ouder turn-bericht is niet vers (turn_detector gestopt)       (1.0)
  rate_hz         hoe vaak RobotFacts en de actuators bijgewerkt worden         (10.0)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from my_bot_interfaces.msg import Turn
from signal_manager.RobotFacts import RobotFacts
from signal_manager.actuators.Blinkers import Blinkers
from signal_manager.actuators.Horn import Horn, HornPattern
from signal_manager.sources.CollisionStateSource import CollisionStateSource
from signal_manager.sources.RobotStateSource import RobotStateSource
from signal_manager.sources.TurnSource import TurnSource


class SignalManagerNode(Node):

    def __init__(self):
        super().__init__('signal_manager')
        hold_s = self.declare_parameter('hold_s', 1.0).value
        turn_timeout_s = self.declare_parameter('turn_timeout_s', 1.0).value
        rate_hz = self.declare_parameter('rate_hz', 10.0).value

        turn = TurnSource(turn_timeout_s)
        state = RobotStateSource()
        collision = CollisionStateSource()
        self.sources = [turn, state, collision]
        self.blinkers = Blinkers(hold_s)
        self.horn = Horn()

        self.create_subscription(Turn, 'turn', lambda m: turn.update(m, self._now()), 10)
        self.create_subscription(
            String, 'mission_status', lambda m: state.update(m, self._now()), 10)
        self.create_subscription(
            Bool, 'navigation/blocked', lambda m: collision.update(m, self._now()),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.indicator_pub = self.create_publisher(String, 'indicators', 10)
        self.buzzer_pub = self.create_publisher(Bool, 'buzzer', 10)
        self._published_state = None
        self._beep_steps = []        # nog af te spelen (aan, duur) van het huidige patroon
        self._beep_timer = None

        self.create_timer(1.0 / rate_hz, self.update)
        self.get_logger().info('SignalManager gestart')

    def update(self):
        now = self._now()
        facts = RobotFacts()
        for source in self.sources:
            source.apply(facts)
        facts.safety_inputs_ok = all(source.is_fresh(now) for source in self.sources)

        self.blinkers.update(facts, now)
        if self.blinkers.state != self._published_state:
            self._published_state = self.blinkers.state
            self.indicator_pub.publish(String(data=self.blinkers.state.value))
            self.get_logger().info(f'knipperlichten: {self.blinkers.state.value}')

        self.horn.update(facts, now)
        if self.horn.pending != HornPattern.NONE:
            self.get_logger().info(f'claxon: {self.horn.pending.name}')
            self._play(self.horn.pending)
            self.horn.pending = HornPattern.NONE

    # ── Claxon: patroon afspelen op /buzzer ──────────────────────────────────

    def _play(self, pattern: HornPattern):
        """Een nieuw patroon onderbreekt het vorige."""
        self._beep_steps = [(i % 2 == 0, d) for i, d in enumerate(pattern.value)]
        self._next_beep_step()

    def _next_beep_step(self):
        if self._beep_timer is not None:
            self.destroy_timer(self._beep_timer)
            self._beep_timer = None
        if not self._beep_steps:
            self.buzzer_pub.publish(Bool(data=False))
            return
        on, duration = self._beep_steps.pop(0)
        self.buzzer_pub.publish(Bool(data=on))
        self._beep_timer = self.create_timer(duration, self._next_beep_step)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main():
    rclpy.init()
    node = SignalManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
