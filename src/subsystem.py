import warnings


class Subsystem:
    def init(self) -> None:
        warnings.warn("init method required")

    def periodic(self) -> None:
        warnings.warn("periodic method required")

    def disable(self) -> None:
        warnings.warn("disable method required")

    # def update(self) -> None:
    #     raise "update method required"

    # def publish(self) -> None:
    #     raise "public method required"
