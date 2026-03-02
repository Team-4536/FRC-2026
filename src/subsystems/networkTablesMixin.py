from functools import partial
from ntcore import NetworkTable, NetworkTableInstance, Value
from typing import Any, Callable, Dict, Optional, Sequence, Tuple, TypeAlias, Union
from wpimath.kinematics import SwerveModuleState

Struct: TypeAlias = object
debugging: bool = True


class NetworkTablesMixin:
    _table: NetworkTable
    _ntPersist: Dict[str, object]

    def __init__(self, *, table: str = "telemetry", inst: bool = True):
        self._table = self._getTable(table, inst)
        self._ntPersist = {}

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

        pub.set(value)  # type: ignore[attr-defined]

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
            topic = self._table.getTopic(name)
            pub = topic.genericPublish(typeStr)
            self._ntPersist[name] = pub

        try:
            pub.set(Value.makeValue(value))  # type: ignore
        except TypeError:
            self.publishGeneric("test", "Null", *subtables)

    def __get(self, n: str, t: Callable[[str], Any], *s: str, d: Any) -> Any:
        if s:
            n = "/".join((*s, n))
        return t(n).getEntry(d).get()

    def _getTable(self, table: Optional[str], inst: bool):
        if table is None:
            table = self._table.getPath()
        if inst:
            table = f"{table}/{self.__class__.__name__}"
        return NetworkTableInstance.getDefault().getTable(table)

    def getString(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: str,
    ) -> str:
        return self.__get(
            name, self._getTable(table, inst).getStringTopic, *subtables, d=default
        )

    def getStringArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: Sequence[str],
    ) -> Sequence[str]:
        return self.__get(
            name,
            self._getTable(table, inst).getStringArrayTopic,
            *subtables,
            d=default,
        )

    def getInteger(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: int,
    ) -> int:
        return self.__get(
            name, self._getTable(table, inst).getIntegerTopic, *subtables, d=default
        )

    def getIntegerArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: Sequence[int],
    ) -> Sequence[int]:
        return self.__get(
            name,
            self._getTable(table, inst).getIntegerArrayTopic,
            *subtables,
            d=default,
        )

    def getFloat(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: float,
    ) -> float:
        return self.__get(
            name, self._getTable(table, inst).getFloatTopic, *subtables, d=default
        )

    def getFloatArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: Sequence[float],
    ) -> Sequence[float]:
        return self.__get(
            name,
            self._getTable(table, inst).getFloatArrayTopic,
            *subtables,
            d=default,
        )

    def getBoolean(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: bool,
    ) -> bool:
        return self.__get(
            name, self._getTable(table, inst).getBooleanTopic, *subtables, d=default
        )

    def getBooleanArray(
        self,
        name: str,
        table: Optional[str] = None,
        *subtables: str,
        inst: bool = True,
        default: Sequence[bool],
    ) -> Sequence[bool]:
        return self.__get(
            name,
            self._getTable(table, inst).getBooleanArrayTopic,
            *subtables,
            d=default,
        )
