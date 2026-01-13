from functools import partial
from ntcore import NetworkTableInstance
from typing import Any, Callable, Dict, Optional, Sequence


class NetworkTablesMixin:
    def __init__(self, *, instance: Optional[str] = None):
        if instance is None:
            instance = self.__class__.__name__
        else:
            instance = f"{self.__class__.__name__}/{instance}"

        self.table = NetworkTableInstance.getDefault().getTable(f"telemetry/{instance}")
        self._ntPersist: Dict[str, object] = {}

    def _publish(
        self, name: str, value: Any, topicFn: Callable[[str], Any], *instances: str
    ) -> None:
        if instances:
            name = f"{"/".join(instances)}/{name}"

        pub = self._ntPersist.get(name)
        if pub is None:
            topic = topicFn(name)
            pub = topic.publish()
            self._ntPersist[name] = pub

        pub.set(value)  # type: ignore[attr-defined]

    def publishString(self, name: str, value: str, *instances: str) -> None:
        self._publish(name, value, self.table.getStringTopic, *instances)

    def publishInt(self, name: str, value: int, *instances: str) -> None:
        self._publish(name, value, self.table.getIntegerTopic, *instances)

    def publishDouble(self, name: str, value: float, *instances: str) -> None:
        self._publish(name, value, self.table.getDoubleTopic, *instances)

    def publishBoolean(self, name: str, value: bool, *instances: str) -> None:
        self._publish(name, value, self.table.getBooleanTopic, *instances)

    def publishSwerve(self, name: str, value: Sequence[Any], *instances: str) -> None:
        topicFn = partial(self.table.getStructArrayTopic, type=value[0].__class__)
        self._publish(name, value, topicFn, *instances)
