from rclpy.node import Node

class SignalManagerNode(Node):

    def __init__(self):
        self.sources: SignalSource
        self.blinkers: Blinkers
        self.horn: Horn