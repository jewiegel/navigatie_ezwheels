class RobotFacts():

    def __init__(self):
        self.mode: RobotMode
        self.collision: CollisionAction
        self.turn: TurnDirection
        self.safety_inputs_ok: bool
