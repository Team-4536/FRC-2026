from math import tau
from phoenix6.units import rotation, volt as voltage
from rev import (
    ClosedLoopConfig,
    ClosedLoopSlot,
    FeedbackSensor,
    FeedForwardConfig,
    LimitSwitchConfig,
    MAXMotionConfig,
    PersistMode,
    ResetMode,
    SoftLimitConfig,
    SparkBaseConfig,
    SparkLimitSwitch,
    SparkMax,
    SparkMaxConfig,
    SparkRelativeEncoder,
)
from subsystems.utils import matchData
from wpimath.units import (
    degrees,
    degreesToRotations,
    inches,
    radians,
    radiansToRotations,
    revolutions_per_minute,
)

# could lowkey be 10 degrees idk
INIT_PITCH_ANGLE: degrees = 8.813
PITCH_RADIUS: inches = 9.342
LIL_PITCH_GEAR_RADIUS: inches = 0.552
ARC_RATIO = (
    PITCH_RADIUS / LIL_PITCH_GEAR_RADIUS
)  # how many rotations of the smol ladder gear is 1 rotation of the pitch
PITCH_GEARING: float = 16 * ARC_RATIO  # 4.86 / degreesToRotations(8)


class RevMotor:
    _ctrlr: SparkMax
    _encoder: SparkRelativeEncoder
    _simPosition: rotation

    def __init__(self, *, deviceID: int) -> None:
        self._ctrlr = SparkMax(deviceID, SparkMax.MotorType.kBrushless)
        self._encoder = self._ctrlr.getEncoder()
        self._simPosition = 0

    def configure(self, *, config: SparkBaseConfig) -> None:
        self._ctrlr.configure(
            config=config,
            resetMode=ResetMode.kResetSafeParameters,
            persistMode=PersistMode.kNoPersistParameters,
        )

    def stopMotor(self) -> None:
        self._ctrlr.set(0)

    def setThrottle(self, throttle: voltage) -> None:
        self._ctrlr.setVoltage(throttle * 12.0)

    def getForwardLimitSwitch(self) -> SparkLimitSwitch:
        return self._ctrlr.getForwardLimitSwitch()

    def getReverseLimitSwitch(self) -> SparkLimitSwitch:
        return self._ctrlr.getReverseLimitSwitch()

    def setVelocity(self, rpm: revolutions_per_minute) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            setpoint=rpm,
            ctrl=SparkMax.ControlType.kMAXMotionVelocityControl,
        )
        if matchData.isSimulation():
            self._simPosition += rpm * matchData.dt / 60
            self._encoder.setPosition(self._simPosition)

    def setVoltage(self, volts: float) -> None:
        self._ctrlr.setVoltage(volts)

    def setPosition(self, rot: radians) -> None:
        self._ctrlr.getClosedLoopController().setReference(
            setpoint=radiansToRotations(rot),
            ctrl=SparkMax.ControlType.kPosition,
        )
        if matchData.isSimulation():
            self._encoder.setPosition(rot / tau)

    def getEncoder(self) -> SparkRelativeEncoder:
        return self._encoder

    DRIVE_GEARiNG: float = 6.12

    AZIMUTH_GEARING: float = 21.4

    DRIVE_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40)
        .disableFollowerMode()
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .voltageCompensation(12)
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

    INTAKE_MOTOR_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(30, 30)
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
                .maxAcceleration(25000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(1)
            )
        )
    )

    INDEXER_MOTOR_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(15, 15)
        .disableFollowerMode()
        .inverted(False)
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            ClosedLoopConfig()
            .pidf(0.00019, 0, 0, 0.00205)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .apply(
                MAXMotionConfig()
                .maxVelocity(2000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(25000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(1)
            )
        )
    )

    INTAKE_RAISE_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(20, 20)
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
    ).apply(
        LimitSwitchConfig()
        .limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder)
        .forwardLimitSwitchEnabled(
            False
        )  # TODO when forward limit switch exists again change
        .reverseLimitSwitchEnabled(True)
        # .forwardLimitSwitchPosition(16.66)
        .reverseLimitSwitchPosition(0)
        .reverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
        )
        # .forwardLimitSwitchTriggerBehavior(
        #     LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
        # )
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
    )

    CLIMBER_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(30, 30)
        .disableFollowerMode()
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            LimitSwitchConfig()
            .limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder)
            .forwardLimitSwitchEnabled(False)
            .reverseLimitSwitchEnabled(True)
            # .forwardLimitSwitchPosition(16.66)
            .reverseLimitSwitchPosition(0)
            .reverseLimitSwitchTriggerBehavior(
                LimitSwitchConfig.Behavior.kStopMovingMotor
            )
            # .forwardLimitSwitchTriggerBehavior(
            #     LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
            # )
            # .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
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
        .smartCurrentLimit(20, 20)
        .inverted(True)
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            ClosedLoopConfig()
            .pidf(0.1, 0, 0, 0)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .positionWrappingEnabled(False)
            .allowedClosedLoopError(0.03)
            .apply(
                MAXMotionConfig()
                .maxVelocity(1000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(500, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(0.2)
            )
            .apply(FeedForwardConfig().kS(0.25, ClosedLoopSlot.kSlot0))
        )
        .apply(
            LimitSwitchConfig()
            .limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder)
            .forwardLimitSwitchEnabled(
                False
            )  # TODO when forward limit switch exists again change, it wont
            .reverseLimitSwitchEnabled(True)
            .reverseLimitSwitchPosition(0)
            .reverseLimitSwitchTriggerBehavior(
                LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
            )
            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        )
        .apply(SoftLimitConfig().forwardSoftLimit(16.66).forwardSoftLimitEnabled(True))
    )

    TURRET_PITCH_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(20, 20)
        .inverted(False)
        .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        .apply(
            ClosedLoopConfig()
            .pidf(0.035, 0, 0, 0.05)
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .positionWrappingEnabled(True)
            .apply(
                MAXMotionConfig()
                .maxVelocity(1000, ClosedLoopSlot.kSlot0)
                .maxAcceleration(500, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(0.2)
            )
        )
        .apply(
            LimitSwitchConfig().limitSwitchPositionSensor(
                FeedbackSensor.kPrimaryEncoder
            )
        )
        .apply(
            SoftLimitConfig()
            .forwardSoftLimit(19.5)
            .reverseSoftLimit(degreesToRotations(INIT_PITCH_ANGLE) * (16 * ARC_RATIO))
            .forwardSoftLimitEnabled(True)
            .reverseSoftLimitEnabled(True)
        )
    )

    FLYWHEEL_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40, 40)
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
                .maxAcceleration(4000, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(1)
            )
        )
    )

    KICK_CONFIG: SparkBaseConfig = (
        SparkMaxConfig()
        .smartCurrentLimit(40, 40)
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
