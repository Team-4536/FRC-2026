from abc import abstractmethod
from rev import SparkMax, SparkMaxConfig
from wpimath.units import radians
from wpimath.units import revolutions_per_minute as RPM


class Motor:
    _name: str
    _id: str

    def __init__(self, *, name: str) -> None:
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    @property
    @abstractmethod
    def id(self) -> str:
        pass


class RevMotor(Motor):
    def __init__(self, *, name: str, id: int) -> None:
        self._id = id
        self._name = name
        self._ctrlr = SparkMax(id, SparkMax.MotorType.kBrushless)

    @property
    def deviceId(self) -> int:
        return self._id

    def configure(self, *, config: SparkMaxConfig) -> None:
        self._ctrlr.configure(
            config=config,
            resetMode=SparkMax.ResetMode.kResetSafeParameters,
            persistMode=SparkMax.PersistMode.kPersistParameters,
        )

    def stop(self) -> None:
        self._ctrlr.set(0)

    @property
    def velocity(self) -> float:
        return 0  # temp

    def setVelocity(self, ref: RPM) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            value=ref,
            ctrl=SparkMax.ControlType.kMAXMotionVelocityControl,
        )

    def setPosition(self, ref: radians) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            value=ref,
            ctrl=SparkMax.ControlType.kMAXMotionPositionControl,
        )
