from subsystems.desiredState import DesiredState
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Callable, Optional


class Autos(NetworkTablesMixin):
    def __init__(self):
        pass

    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        pass

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass
