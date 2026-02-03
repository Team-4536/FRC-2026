from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


class MessageSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.startMessage: str = "Hi Abdulfitah"
        self.currentMessage: str = self.startMessage
        self.messageAdd: str = "."
        self.isDisabled = True

    def phaseInit(self) -> None:
        pass

    def periodic(self, robotState: RobotState) -> RobotState:
        self.currentMessage = self.currentMessage + self.messageAdd

        print(self.currentMessage)

        self.isDisabled = False

        return robotState

    def disabled(self):
        if self.isDisabled == False:
            print("\n\n\n\n\n\n\n\n\n\n I'm done.")
            self.isDisabled = True
