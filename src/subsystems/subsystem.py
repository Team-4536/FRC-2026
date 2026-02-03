from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from typing import Callable, Optional


class SubsystemMethodError(Exception):
    pass


class Subsystem(NetworkTablesMixin):
    def __init__(self, *, instance: Optional[str] = None):
        super().__init__(instance=instance)

    def phaseInit(self) -> None:
        self._warn(self.phaseInit)

    def periodic(self, robotState: RobotState) -> RobotState:  # type: ignore
        self._warn(self.periodic)

    def disabled(self) -> None:
        self._warn(self.disabled)

    def publish(self) -> None:
        pass

    def _warn(self, method: Callable[..., Optional[RobotState]]) -> None:
        methodName = getattr(method, "__name__", "<unknown>")
        raise SubsystemMethodError(
            f"{methodName} method required in {self.__class__.__name__}"
        )  # how to tell typing that i know method with have a .__name__
