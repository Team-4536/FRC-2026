from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from typing import Callable, Optional


class SubsystemMethodError(Exception):
    pass


class Subsystem(NetworkTablesMixin):
    def __init__(self, *, table: str = "telemetry", inst: bool = True):
        super().__init__(table=table, inst=inst)

    def phaseInit(self, robotState: RobotState) -> None:
        self._warn(self.phaseInit)

    def periodic(self, robotState: RobotState) -> None:
        self._warn(self.periodic)

    def robotPeriodic(self, robotState: RobotState) -> None:
        pass

    def disabled(self) -> None:
        self._warn(self.disabled)

    def publish(self) -> None:
        self._warn(self.publish)

    def _warn(self, method: Callable[..., Optional[RobotState]]) -> None:
        methodName = getattr(method, "__name__")
        raise SubsystemMethodError(
            f"{methodName} method required in {self.__class__.__name__}"
        )
