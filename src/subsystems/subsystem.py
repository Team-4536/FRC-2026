from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from typing import Callable, Optional, Union


class SubsystemMethodError(Exception):
    pass


class Subsystem(NetworkTablesMixin):
    def __init__(self, *, instance: Optional[str] = None):
        super().__init__(instance=instance)

    def init(self) -> None:
        self._warn(self.init)

    def periodic(self, robotState: RobotState) -> RobotState:  # type: ignore
        self._warn(self.periodic)

    def disabled(self) -> None:
        self._warn(self.disabled)

    def _warn(self, method: Callable[..., Union[None, RobotState]]) -> None:
        raise SubsystemMethodError(
            f"{method.__name__} method required in {self.__class__.__name__}"
        )
