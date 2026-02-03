from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


class MessageSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # self.startMessage: str = "Hi Abdulfitah"
        # self.currentMessage: str = self.startMessage
        # self.messageAdd: str = "."

    def phaseInit(self) -> None:
        pass

    def periodic(self, robotState: RobotState) -> RobotState:
        # self.currentMessage = self.currentMessage + self.messageAdd

        # print(self.currentMessage)

        print("hi")

        return robotState

    def disabled(self):
        pass
