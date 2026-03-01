from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from typing import Callable, Optional


class SubsystemMethodError(Exception):
    pass


class Subsystem(NetworkTablesMixin):
    def __init__(self, *, table: str = "telemetry", instance: Optional[str] = None):
        super().__init__(table=table, instance=instance)

    def phaseInit(
        self, robotState: RobotState
    ) -> RobotState:  # TODO: stop returning RobotState as it is now unused
        self._warn(self.phaseInit)
        return robotState

    def periodic(
        self, robotState: RobotState
    ) -> RobotState:  # TODO: stop returning RobotState for the same reason ^^^
        self._warn(self.periodic)
        return robotState

    def disabled(self) -> None:
        self._warn(self.disabled)

    def publish(self) -> None:
        pass

    def _warn(self, method: Callable[..., Optional[RobotState]]) -> None:
        methodName = getattr(method, "__name__")
        raise SubsystemMethodError(
            f"{methodName} method required in {self.__class__.__name__}"
        )
