from abc import ABC, abstractmethod

class Actuator(ABC):

    @abstractmethod
    def update(facts: RobotFacts, now):
        pass