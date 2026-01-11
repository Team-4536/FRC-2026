from desiredState import DesiredState
from typing import Optional
from warnings import warn


class Subsystem:
    def init(self) -> None:
        warn("init method required")

    def periodic(self, ds: DesiredState) -> None:
        warn("periodic method required")

    def disable(self) -> None:
        warn("disable method required")

    # def update(self) -> None:
    #     warn("update method required")

    # def publish(self) -> None:
    #     warn("public method required")
