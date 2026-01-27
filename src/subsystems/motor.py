from rev import (
    ClosedLoopConfig,
    ClosedLoopSlot,
    MAXMotionConfig,
    SparkBaseConfig,
    SparkMax,
    SparkMaxConfig,
    SparkRelativeEncoder,
    FeedbackSensor,
    ResetMode,
    PersistMode,
)
from math import pi as PI
from wpimath.units import radians
from wpimath.units import revolutions_per_minute as RPM


class RevMotor:
    def __init__(self, *, deviceID: int) -> None:
        self._ctrlr = SparkMax(deviceID, SparkMax.MotorType.kBrushless)

    def configure(self, *, config: SparkBaseConfig) -> None:
        self._ctrlr.configure(
            config=config,
            resetMode=ResetMode.kResetSafeParameters,
            persistMode=PersistMode.kPersistParameters,
        )

    def stopMotor(self) -> None:
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

    DRIVE_GEARiNG: float = 6.12

    AZIMUTH_GEARING: float = 21.4

    DRIVE_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .disableFollowerMode()
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            ClosedLoopConfig()
            .pidf(0.00019, 0, 0, 0.00002)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .apply(
                MAXMotionConfig()
                .maxVelocity(2000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(50000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(1)
            )
        )
    )

    AZIMUTH_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .inverted(True)
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            ClosedLoopConfig()
            .pidf(0.15, 0, 0, 0)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            # .positionWrappingEnabled(True)
            # .positionWrappingMinInput(-AZIMUTH_GEARING / 2)
            # .positionWrappingMaxInput(AZIMUTH_GEARING / 2)
            .apply(
                MAXMotionConfig()
                .maxVelocity(5000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(10000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(0.2)
            )
        )
    )

    DISABLED_DRIVE_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .disableFollowerMode()
        .setIdleMode(SparkMaxConfig.IdleMode.kCoast)
    )

    DISABLED_AZIMUTH_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .inverted(True)
        .setIdleMode(SparkMaxConfig.IdleMode.kCoast)
    )
