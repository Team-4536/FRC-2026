from functools import partial
from ntcore import NetworkTableInstance, Value
from typing import Any, Callable, Dict, Optional, Sequence, Tuple, TypeAlias, Union
from wpimath.kinematics import SwerveModuleState

Struct: TypeAlias = object


class NetworkTablesMixin:
    def __init__(self, *, table: str = "telemetry", instance: Optional[str] = None):
        self.table = self._getTable(table, instance)
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

    def publishStringArray(
        self, name: str, value: Sequence[str], *subtables: str
    ) -> None:
        self.__publish(name, value, self.table.getStringArrayTopic, *subtables)

    def publishInteger(self, name: str, value: int, *subtables: str) -> None:
        self.__publish(name, value, self.table.getIntegerTopic, *subtables)

    def publishIntegerArray(
        self, name: str, value: Sequence[int], *subtables: str
    ) -> None:
        self.__publish(name, value, self.table.getIntegerArrayTopic, *subtables)

    def publishFloat(self, name: str, value: float, *subtables: str) -> None:
        self.__publish(name, value, self.table.getFloatTopic, *subtables)

    def publishFloatArray(
        self, name: str, value: Sequence[float], *subtables: str
    ) -> None:
        self.__publish(name, value, self.table.getFloatArrayTopic, *subtables)

    def publishBoolean(self, name: str, value: bool, *subtables: str) -> None:
        self.__publish(name, value, self.table.getBooleanTopic, *subtables)

    def publishBooleanArray(
        self, name: str, value: Sequence[bool], *subtables: str
    ) -> None:
        self.__publish(name, value, self.table.getBooleanArrayTopic, *subtables)

    def publishStruct(self, name: str, value: Struct, *subtables: str) -> None:
        topicFn = partial(self.table.getStructTopic, type=value.__class__)
        self.__publish(name, value, topicFn, *subtables)

    def publishStructArray(
        self, name: str, value: Sequence[Struct], *subtables: str
    ) -> None:
        topicFn = partial(self.table.getStructArrayTopic, type=value[0].__class__)
        self.__publish(name, value, topicFn, *subtables)

    def publishGeneric(
        self,
        name: str,
        value: Optional[
            Union[
                int,
                Sequence[int],
                str,
                Sequence[str],
                float,
                Sequence[float],
                Struct,
                Sequence[Struct],
            ]
        ],
        *subtables: str,
    ) -> None:
        if value is None:
            return

        if hasattr(value, "WPIStruct"):
            self.publishStruct(name, value)
            return
        elif isinstance(value, Sequence) and all(
            v is not None and hasattr(v, "WPIStruct") for v in value  # pyright: ignore
        ):
            self.publishStructArray(name, value)  # pyright: ignore
        elif subtables:
            name = "/".join((*subtables, name))

        if value is None:
            self.publishString(name, "Null", *subtables)

        typeStr = type(value).__name__  # pyright: ignore
        pub = self._ntPersist.get(name)
        if pub is None:
            topic = self.table.getTopic(name)
            pub = topic.genericPublish(typeStr)
            self._ntPersist[name] = pub

        try:
            pub.set(Value.makeValue(value))  # type: ignore
        except TypeError:
            self.publishGeneric("test", "Null", *subtables)

    def publishSwerve(
        self,
        name: str,
        value: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
        *subtables: str,
    ) -> None:
        self.publishStructArray(name, value, *subtables)

    def __get(
        self,
        name: str,
        topicFn: Callable[[str], Any],
        *subtables: str,
        default: Any,
    ) -> Any:
        if subtables:
            name = "/".join((*subtables, name))

        topic = topicFn(name)

        return topic.getEntry(default).get()

    def _getTable(self, table: str, instance: Optional[str] = None):
        if not instance:
            table = f"{table}/{self.__class__.__name__}"
        return NetworkTableInstance.getDefault().getTable(table)

    def getString(
        self, name: str, table: str = "telemetry", *subtables: str, default: str
    ) -> str:
        return self.__get(
            name, self._getTable(table).getStringTopic, *subtables, default=default
        )

    def getStringArray(
        self,
        name: str,
        table: str = "telemetry",
        *subtables: str,
        default: Sequence[str],
    ) -> Sequence[str]:
        return self.__get(
            name, self._getTable(table).getStringArrayTopic, *subtables, default=default
        )

    def getInteger(
        self, name: str, table: str = "telemetry", *subtables: str, default: int
    ) -> int:
        return self.__get(
            name, self._getTable(table).getIntegerTopic, *subtables, default=default
        )

    def getIntegerArray(
        self,
        name: str,
        table: str = "telemetry",
        *subtables: str,
        default: Sequence[int],
    ) -> Sequence[int]:
        return self.__get(
            name,
            self._getTable(table).getIntegerArrayTopic,
            *subtables,
            default=default,
        )

    def getFloat(
        self, name: str, table: str = "telemetry", *subtables: str, default: float
    ) -> float:
        return self.__get(
            name, self._getTable(table).getFloatTopic, *subtables, default=default
        )

    def getFloatArray(
        self,
        name: str,
        table: str = "telemetry",
        *subtables: str,
        default: Sequence[float],
    ) -> Sequence[float]:
        return self.__get(
            name, self._getTable(table).getFloatArrayTopic, *subtables, default=default
        )

    def getBoolean(
        self, name: str, table: str = "telemetry", *subtables: str, default: bool
    ) -> bool:
        return self.__get(
            name, self._getTable(table).getBooleanTopic, *subtables, default=default
        )

    def getBooleanArray(
        self,
        name: str,
        table: str = "telemetry",
        *subtables: str,
        default: Sequence[bool],
    ) -> Sequence[bool]:
        return self.__get(
            name,
            self._getTable(table).getBooleanArrayTopic,
            *subtables,
            default=default,
        )
