from functools import partial
from ntcore import NetworkTableInstance
from typing import (
    Any,
    Callable,
    cast,
    Dict,
    Optional,
    Sequence,
    Tuple,
    TypeAlias,
    Union,
)
from wpimath.kinematics import SwerveModuleState

Struct: TypeAlias = object


class NetworkTablesMixin:
    def __init__(self, *, table: str = "telemetry", instance: Optional[str] = None):
        if not instance:
            instance = self.__class__.__name__

        self.table = NetworkTableInstance.getDefault().getTable(f"{table}/{instance}")
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
        if isinstance(value, int):
            self.publishInteger(name, value, *subtables)
        elif isinstance(value, Sequence) and isinstance(value[0], int):
            self.publishIntegerArray(name, cast(Sequence[int], value), *subtables)
        elif isinstance(value, str):
            self.publishString(name, value, *subtables)
        elif isinstance(value, Sequence) and isinstance(value[0], str):
            self.publishStringArray(name, cast(Sequence[str], value), *subtables)
        elif isinstance(value, float):
            self.publishFloat(name, value, *subtables)
        elif isinstance(value, Sequence) and isinstance(value[0], float):
            self.publishFloatArray(name, cast(Sequence[float], value), *subtables)
        elif isinstance(value, bool):
            self.publishBoolean(name, value, *subtables)
        elif isinstance(value, Sequence) and isinstance(value[0], bool):
            self.publishBooleanArray(name, cast(Sequence[bool], value), *subtables)

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
        table: str = "teleop",
        *subtables: str,
        default: Any = None,
    ) -> Any:
        if subtables:
            name = "/".join((*subtables, name))

        topic = topicFn(name)

        return topic.getEntry(default).get()

    def getString(
        self,
        name: str,
        *subtables: str,
        default: Optional[str] = None,
        tableInstance: Optional[str] = None,
    ) -> Optional[str]:
        if not tableInstance:
            table = self.table
        else:
            table = self.table
        val = self.__get(name, table.getStringTopic, *subtables, default=default)
        return str(val) if val is not None else None

    def getStringArray(
        self, name: str, *subtables: str, default: Optional[Sequence[str]] = None
    ) -> Optional[Sequence[str]]:
        val = self.__get(
            name, self.table.getStringArrayTopic, *subtables, default=default
        )
        return val if val is not None else None

    def getInteger(
        self, name: str, *subtables: str, default: Optional[int] = None
    ) -> Optional[int]:
        val = self.__get(name, self.table.getIntegerTopic, *subtables, default=default)
        return int(val) if val is not None else None

    def getIntegerArray(
        self, name: str, *subtables: str, default: Optional[Sequence[int]] = None
    ) -> Optional[Sequence[int]]:
        val = self.__get(
            name, self.table.getIntegerArrayTopic, *subtables, default=default
        )
        return val if val is not None else None

    def getFloat(
        self, name: str, *subtables: str, default: Optional[float] = None
    ) -> Optional[float]:
        val = self.__get(name, self.table.getFloatTopic, *subtables, default=default)
        return float(val) if val is not None else None

    def getFloatArray(
        self, name: str, *subtables: str, default: Optional[Sequence[float]] = None
    ) -> Optional[Sequence[float]]:
        val = self.__get(
            name, self.table.getFloatArrayTopic, *subtables, default=default
        )
        return val if val is not None else None

    def getBoolean(
        self, name: str, *subtables: str, default: Optional[bool] = None
    ) -> Optional[bool]:
        val = self.__get(name, self.table.getBooleanTopic, *subtables, default=default)
        return bool(val) if val is not None else None

    def getBooleanArray(
        self, name: str, *subtables: str, default: Optional[Sequence[bool]] = None
    ) -> Optional[Sequence[bool]]:
        val = self.__get(
            name, self.table.getBooleanArrayTopic, *subtables, default=default
        )
        return val if val is not None else None
