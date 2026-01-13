from subsystems.desiredState import DesiredState
from subsystems.subsystem import Subsystem


class SwerveDrive(Subsystem):
    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        pass

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass
