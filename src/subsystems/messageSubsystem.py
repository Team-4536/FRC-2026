from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


class MyMessage(Subsystem):
    def __init__(self) -> None:
        self.startMessage: str = "Hi Abdulfitah"
        self.currentMessage: str = self.startMessage
        self.messageAdd: str = "."

    def init(self) -> None:
        pass

    def periodic(self, rs: RobotState) -> None:
        self.currentMessage = self.currentMessage + self.messageAdd

    def disabled(self):
        pass

    def publish(self):
        print(self.currentMessage)
