from subsystems.robotState import DesiredState, CurrentState
from subsystems.subsystem import Subsystem


class SwerveDrive(Subsystem):
    def init(self) -> None:
        pass

    def periodic(self, desiredState: DesiredState, currentState: CurrentState) -> None:
        pass

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass
