from enum import Enum
from functools import partial
from ntcore import NetworkTable, NetworkTableInstance, Value
from typing import Any, Callable, Dict, Optional, Sequence, Tuple, TypeAlias, Union
from wpimath.kinematics import SwerveModuleState

Struct: TypeAlias = object
debugging: bool = True
from wpilib import getTime


class NetworkTablesMixin:
    _table: NetworkTable
    _ntPersist: Dict[str, object]
    _tablePersist: Dict[str, NetworkTable]

    def __init__(self, *, table: str = "telemetry", inst: bool = False):
        self._ntPersist = {}
        self._tablePersist = {}

        self._table = self._getTable(table, inst)

    def __publish(
        self,
        name: str,
        value: Any,
        topicFn: Callable[[str], Any],
        *subtables: str,
        debug: bool,
    ) -> None:
        if debug and not debugging:
            return

        if subtables:
            name = "/".join((*subtables, name))

        pub = self._ntPersist.get(name)
        if pub is None:
            topic = topicFn(name)
            pub = topic.publish()
            self._ntPersist[name] = pub

        try:
            pub.set(value)  # type: ignore[attr-defined]
        except TypeError:
            pass

    def publishString(
        self, name: str, value: str, *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(name, value, self._table.getStringTopic, *subtables, debug=debug)

    def publishStringArray(
        self, name: str, value: Sequence[str], *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(
            name, value, self._table.getStringArrayTopic, *subtables, debug=debug
        )

    def publishInteger(
        self, name: str, value: int, *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(
            name, value, self._table.getIntegerTopic, *subtables, debug=debug
        )

    def publishIntegerArray(
        self, name: str, value: Sequence[int], *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(
            name, value, self._table.getIntegerArrayTopic, *subtables, debug=debug
        )

    def publishFloat(
        self, name: str, value: float, *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(name, value, self._table.getFloatTopic, *subtables, debug=debug)

    def publishFloatArray(
        self, name: str, value: Sequence[float], *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(
            name, value, self._table.getFloatArrayTopic, *subtables, debug=debug
        )

    def publishBoolean(
        self, name: str, value: bool, *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(
            name, value, self._table.getBooleanTopic, *subtables, debug=debug
        )

    def publishBooleanArray(
        self, name: str, value: Sequence[bool], *subtables: str, debug: bool = False
    ) -> None:
        self.__publish(
            name, value, self._table.getBooleanArrayTopic, *subtables, debug=debug
        )

    def publishStruct(
        self, name: str, value: Struct, *subtables: str, debug: bool = False
    ) -> None:
        topicFn = partial(self._table.getStructTopic, type=value.__class__)
        self.__publish(name, value, topicFn, *subtables, debug=debug)

    def publishStructArray(
        self, name: str, value: Sequence[Struct], *subtables: str, debug: bool = False
    ) -> None:
        topicFn = partial(self._table.getStructArrayTopic, type=value[0].__class__)
        self.__publish(name, value, topicFn, *subtables, debug=debug)

    def publishSwerve(
        self,
        name: str,
        value: Tuple[SwerveModuleState, ...],
        *subtables: str,
        debug: bool = False,
    ) -> None:
        self.publishStructArray(name, value, *subtables, debug=debug)

    def publishGeneric(
        self,
        name: str,
        value: Optional[
            Union[
                int,
                Sequence[int],
                bool,
                Sequence[bool],
                str,
                Sequence[str],
                float,
                Sequence[float],
                Struct,
                Sequence[Struct],
            ]
        ],
        *subtables: str,
        debug: bool = False,
    ) -> None:
        if value is None:
            return

        if hasattr(value, "WPIStruct") and value is not None:
            self.publishStruct(name, value, *subtables, debug=debug)
            return
        elif isinstance(value, Sequence) and all(
            v is not None and hasattr(v, "WPIStruct") for v in value  # pyright: ignore
        ):
            self.publishStructArray(
                name, value, *subtables, debug=debug  # pyright: ignore
            )
            return
        elif isinstance(value, Enum):
            self.publishString(name, value.name, *subtables, debug=debug)
            return
        elif isinstance(value, bool):
            self.publishBoolean(name, value, *subtables, debug=debug)
            return
        elif subtables:
            name = "/".join((*subtables, name))

        typeStr = type(value).__name__  # pyright: ignore
        pub = self._ntPersist.get(name)
        if pub is None:
            topic = self._table.getTopic(name)
            pub = topic.genericPublish(typeStr)
            self._ntPersist[name] = pub

        try:
            pub.set(Value.makeValue(value))  # type: ignore
        except TypeError:
            return

    def __get(self, n: str, t: Callable[[str], Any], *s: str, d: Any) -> Any:
        if s:
            n = "/".join((*s, n))
        return t(n).getEntry(d).get()

    def _getTable(self, tableStr: Optional[str], inst: bool = False):
        if tableStr is None:
            tableStr = self._table.getPath()
        if inst:
            tableStr = f"{tableStr}/{self.__class__.__name__}"

        table = self._tablePersist.get(tableStr)
        if table is None:
            table = NetworkTableInstance.getDefault().getTable(tableStr)
            self._ntPersist[tableStr] = table

        return table

    def getString(
        self, name: str, table: Optional[str] = None, *subtables: str, default: str
    ) -> str:
        return self.__get(
            name, self._getTable(table).getStringTopic, *subtables, d=default
        )

    def getStringArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        default: Sequence[str],
    ) -> Sequence[str]:
        return self.__get(
            name, self._getTable(table).getStringArrayTopic, *subtables, d=default
        )

    def getInteger(
        self, name: str, table: Optional[str] = None, *subtables: str, default: int
    ) -> int:
        return self.__get(
            name, self._getTable(table).getIntegerTopic, *subtables, d=default
        )

    def getIntegerArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        default: Sequence[int],
    ) -> Sequence[int]:
        return self.__get(
            name, self._getTable(table).getIntegerArrayTopic, *subtables, d=default
        )

    def getFloat(
        self, name: str, table: Optional[str] = None, *subtables: str, default: float
    ) -> float:
        return self.__get(
            name, self._getTable(table).getFloatTopic, *subtables, d=default
        )

    def getFloatArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        default: Sequence[float],
    ) -> Sequence[float]:
        return self.__get(
            name, self._getTable(table).getFloatArrayTopic, *subtables, d=default
        )

    def getBoolean(
        self, name: str, table: Optional[str] = None, *subtables: str, default: bool
    ) -> bool:
        return self.__get(
            name, self._getTable(table).getBooleanTopic, *subtables, d=default
        )

    def getBooleanArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        default: Sequence[bool],
    ) -> Sequence[bool]:
        return self.__get(
            name, self._getTable(table).getBooleanArrayTopic, *subtables, d=default
        )
