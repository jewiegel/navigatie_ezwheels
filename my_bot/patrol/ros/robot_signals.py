"""Adapter voor alles wat de robot naar buiten doet buiten Nav2 om:
wielen (cmd_vel), buzzer, knipperlichten en de toestand naar de HMI."""
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

BEEP_SECONDS = 0.3


class RobotSignals:

    def __init__(self, node, scheduler, cmd_vel_topic: str = '/cmd_vel'):
        self._scheduler = scheduler
        self._cmdvel_pub    = node.create_publisher(Twist,  cmd_vel_topic,   10)
        self._buzzer_pub    = node.create_publisher(Bool,   '/buzzer',       10)
        self._indicator_pub = node.create_publisher(String, '/indicators',   10)
        self._state_pub     = node.create_publisher(String, '/patrol_state', 10)
        self._beep_timer = None

    def stop_wheels(self):
        self._cmdvel_pub.publish(Twist())

    def drive(self, linear_x: float):
        twist = Twist()
        twist.linear.x = linear_x
        self._cmdvel_pub.publish(twist)

    def beep(self):
        self._buzzer_pub.publish(Bool(data=True))
        if self._beep_timer is not None:
            self._beep_timer.cancel()
        self._beep_timer = self._scheduler.call_later(BEEP_SECONDS, self._beep_off)

    def _beep_off(self):
        self._beep_timer = None
        self._buzzer_pub.publish(Bool(data=False))

    def set_indicator(self, mode: str):
        """Via /indicators → mqtt_hmi_bridge → HMI (lamp_links/lamp_rechts).
        mode: 'links', 'rechts', 'gevaar' (beide) of 'uit'."""
        self._indicator_pub.publish(String(data=mode))

    def publish_state(self, name: str):
        self._state_pub.publish(String(data=name))
