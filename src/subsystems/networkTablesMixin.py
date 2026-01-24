from functools import partial
from ntcore import (
    BooleanTopic,
    DoubleTopic,
    IntegerTopic,
    NetworkTableInstance,
    StringTopic,
)
from typing import Any, Callable, Dict, Optional, Tuple, Union
from wpimath.kinematics import SwerveModuleState


class NetworkTablesMixin:
    def __init__(self, *, instance: Optional[str] = None):
        if instance is None:
            instance = self.__class__.__name__
        else:
            instance = f"{self.__class__.__name__}/{instance}"

        self.table = NetworkTableInstance.getDefault().getTable(f"telemetry/{instance}")
        self._ntPersist: Dict[str, object] = {}

    def __publish(
        self, name: str, value: Any, topicFn: Callable[[str], Any], *subtables: str
    ) -> None:
        if subtables:
            name = "/".join((*subtables, name))

        pub = self._ntPersist.get(name)
        if pub is None:
            topic = topicFn(name)
            pub = topic.publish()
            self._ntPersist[name] = pub

        pub.set(value)  # type: ignore[attr-defined]

    def publishString(self, name: str, value: str, *subtables: str) -> None:
        self.__publish(name, value, self.table.getStringTopic, *subtables)

    def publishInteger(self, name: str, value: int, *subtables: str) -> None:
        self.__publish(name, value, self.table.getIntegerTopic, *subtables)

    def publishDouble(self, name: str, value: float, *subtables: str) -> None:
        self.__publish(name, value, self.table.getDoubleTopic, *subtables)

    def publishBoolean(self, name: str, value: bool, *subtables: str) -> None:
        self.__publish(name, value, self.table.getBooleanTopic, *subtables)

    def publishSwerve(
        self,
        name: str,
        value: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
        *subtables: str,
    ) -> None:
        topicFn = partial(self.table.getStructArrayTopic, type=value[0].__class__)
        self.__publish(name, value, topicFn, *subtables)

    def __get(
        self,
        name: str,
        topicFn: Callable[[str], Any],
        *subtables: str,
        default: Optional[Union[str, int, float, bool]] = None,
    ) -> Any:
        if subtables:
            name = "/".join((*subtables, name))

        topic = topicFn(name)
        value = default
        if topic.exists():
            if isinstance(topic, StringTopic):
                default = default if isinstance(default, str) else ""
                value = topic.getEntry(default).get()
            elif isinstance(topic, IntegerTopic):
                default = default if isinstance(default, int) else 0
                value = topic.getEntry(default).get()
            elif isinstance(topic, DoubleTopic):
                default = default if isinstance(default, float) else 0
                value = topic.getEntry(default).get()
            elif isinstance(topic, BooleanTopic):
                default = default if isinstance(default, bool) else False
                value = topic.getEntry(default).get()
        return value

    def getString(
        self, name: str, *subtables: str, default: Optional[str] = None
    ) -> Optional[str]:
        val = self.__get(name, self.table.getStringTopic, *subtables, default=default)
        return str(val) if val is not None else None

    def getInteger(
        self, name: str, *subtables: str, default: Optional[int] = None
    ) -> Optional[int]:
        val = self.__get(name, self.table.getIntegerTopic, *subtables, default=default)
        return int(val) if val is not None else None

    def getDouble(
        self, name: str, *subtables: str, default: Optional[float] = None
    ) -> Optional[float]:
        val = self.__get(name, self.table.getDoubleTopic, *subtables, default=default)
        return float(val) if val is not None else None

    def getBoolean(
        self, name: str, *subtables: str, default: Optional[bool] = None
    ) -> Optional[bool]:
        val = self.__get(name, self.table.getBooleanTopic, *subtables, default=default)
        return bool(val) if val is not None else None
