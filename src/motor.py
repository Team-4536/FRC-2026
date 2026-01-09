from rev import SparkAbsoluteEncoder, SparkMax, SparkMaxConfig, SparkRelativeEncoder
from wpimath.units import revolutions_per_minute as RPM


class RevMotor:
    def __init__(self, *, id: int) -> None:
        self._ctrlr = SparkMax(id, SparkMax.MotorType.kBrushless)

    def configure(self, *, config: SparkMaxConfig) -> None:
        self._ctrlr.configure(
            config=config,
            resetMode=SparkMax.ResetMode.kResetSafeParameters,
            persistMode=SparkMax.PersistMode.kPersistParameters,
        )

    def stop(self) -> None:
        self._ctrlr.set(0)

    def setVelocity(self, rpm: RPM) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            value=rpm,
            ctrl=SparkMax.ControlType.kMAXMotionVelocityControl,
        )

    def setPosition(self, rot) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            value=rot,
            ctrl=SparkMax.ControlType.kMAXMotionPositionControl,
        )

    def getEncoder(self) -> SparkRelativeEncoder:
        return self._ctrlr.getEncoder()

    def getAbsoluteEncoder(self) -> SparkAbsoluteEncoder:
        return self._ctrlr.getAbsoluteEncoder()
