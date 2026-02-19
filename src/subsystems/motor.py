from rev import (
    ClosedLoopConfig,
    ClosedLoopSlot,
    FeedbackSensor,
    MAXMotionConfig,
    PersistMode,
    ResetMode,
    SparkBaseConfig,
    SparkMax,
    SparkMaxConfig,
    SparkRelativeEncoder,
    LimitSwitchConfig,
    SoftLimitConfig,
    FeedForwardConfig,
)
from wpimath.units import radians
from wpimath.units import revolutions_per_minute as RPM, radiansToRotations


class RevMotor:
    def __init__(self, *, deviceID: int) -> None:
        self._ctrlr = SparkMax(deviceID, SparkMax.MotorType.kBrushless)

    def configure(self, *, config: SparkBaseConfig) -> None:
        self._ctrlr.configure(
            config=config,
            resetMode=ResetMode.kResetSafeParameters,
            persistMode=PersistMode.kNoPersistParameters,
        )

    def stopMotor(self) -> None:
        self._ctrlr.set(0)

    def setThrottle(self, throttle: float) -> None:
        self._ctrlr.setVoltage(throttle * 12.0)

    def setVelocity(self, rpm: RPM) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            setpoint=rpm,
            ctrl=SparkMax.ControlType.kVelocity,
        )

    def setMaxMotionVelocity(self, rpm) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            setpoint=rpm, ctrl=SparkMax.ControlType.kMAXMotionVelocityControl
        )

    def setVoltage(self, volts: float) -> None:
        self._ctrlr.setVoltage(volts)

    def setPosition(self, rot: radians) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            setpoint=radiansToRotations(rot),
            ctrl=SparkMax.ControlType.kPosition,
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
            .pidf(0.00019, 0, 0, 0.00205)
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
            .positionWrappingEnabled(True)
            .positionWrappingMinInput(-AZIMUTH_GEARING / 2)
            .positionWrappingMaxInput(AZIMUTH_GEARING / 2)
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

    TURRET_YAW_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .inverted(True)
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            LimitSwitchConfig()
            .limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder)
            .reverseLimitSwitchEnabled(True)
            .forwardLimitSwitchEnabled(True)
        )
        .apply(
            SoftLimitConfig()
            .forwardSoftLimit(0)
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(True)
            .reverseSoftLimitEnabled(True)
        )
        .apply(
            ClosedLoopConfig()
            .pidf(0.039, 0, 0, 0.02)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .positionWrappingEnabled(True)
            .apply(
                MAXMotionConfig()
                .maxVelocity(5000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(10000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(0.2)
            )
            .apply(FeedForwardConfig().kA(0))
        )
    )
    TURRET_PITCH_CONFIG = AZIMUTH_CONFIG.apply(
        LimitSwitchConfig().reverseLimitSwitchEnabled(True)
    )

    FLYWHEEL_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .disableFollowerMode()
        .setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        .inverted(True)
        .apply(
            ClosedLoopConfig()
            .pidf(0.000025, 0, 0, 0.0018072289)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .apply(
                MAXMotionConfig()
                .maxVelocity(4000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(1000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(1)
            )
        )
    )

    KICK_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .disableFollowerMode()
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .inverted(False)
        .apply(
            ClosedLoopConfig()
            .pidf(0.00019, 0, 0, 0.00205)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .apply(
                MAXMotionConfig()
                .maxVelocity(4000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(1000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(1)
            )
        )
    )
