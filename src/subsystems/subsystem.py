from subsystems.desiredState import DesiredState
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Callable, Optional


class SubsystemMethodError(Exception):
    pass


class Subsystem(NetworkTablesMixin):
    def __init__(self, *, instance: Optional[str] = None):
        super().__init__(instance=instance)

    def init(self) -> None:
        self._warn(self.init)

    def periodic(self, ds: DesiredState) -> None:
        self._warn(self.periodic)

    def disabled(self) -> None:
        self._warn(self.disabled)

    def publish(self) -> None:
        self._warn(self.publish)

    def _warn(self, method: Callable[..., None]) -> None:
        raise SubsystemMethodError(
            f"{method.__name__} method required in {self.__class__.__name__}"
        )
