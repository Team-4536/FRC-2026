from rev import (
    ClosedLoopConfig,
    ClosedLoopSlot,
    MAXMotionConfig,
    SparkAbsoluteEncoder,
    SparkBaseConfig,
    SparkMax,
    SparkMaxConfig,
    SparkRelativeEncoder,
)
from wpimath.units import revolutions_per_minute as RPM
from wpimath.units import radians


class RevMotor:
    def __init__(self, *, deviceID: int) -> None:
        self._ctrlr = SparkMax(deviceID, SparkMax.MotorType.kBrushless)

    def configure(self, *, config: SparkBaseConfig) -> None:
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

    def setPosition(self, rot: radians) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            value=rot,
            ctrl=SparkMax.ControlType.kMAXMotionPositionControl,
        )

    def getEncoder(self) -> SparkRelativeEncoder:
        return self._ctrlr.getEncoder()

    def getAbsoluteEncoder(self) -> SparkAbsoluteEncoder:
        return self._ctrlr.getAbsoluteEncoder()

    driveConfig: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .disableFollowerMode()
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            ClosedLoopConfig()
            .pidf(0.00019, 0, 0, 0.00002)
            .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0)
            .apply(
                MAXMotionConfig()
                .maxVelocity(2000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(50000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(1)
            )
        )
    )

    azimuthConfig: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .inverted(True)
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            ClosedLoopConfig()
            .pidf(0.15, 0, 0, 0)
            .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0)
            .positionWrappingEnabled(True)
            .apply(
                MAXMotionConfig()
                .maxVelocity(5000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(10000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(0.2)
            )
        )
    )
