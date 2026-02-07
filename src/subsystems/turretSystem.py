from subsystems.motor import RevMotor
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState, ROBOT_RADIUS, getTangentalVelocity
from rev import (
    SparkRelativeEncoder,
    SparkBaseConfig,
    SparkMaxConfig,
    ClosedLoopConfig,
    MAXMotionConfig,
    ClosedLoopSlot,
    LimitSwitchConfig,
    FeedbackSensor,
)
from subsystems.inputs import Inputs
from subsystems.robotState import RobotState
from phoenix6.hardware import CANcoder
from wpimath.units import (
    rotationsToRadians,
    degreesToRadians,
    rotationsToDegrees,
    inchesToMeters,
    revolutions_per_minute as RPM,
    radians,
    meters,
    inches,
)
import math
from math import tau as TAU, pi as PI
import numpy as np
from wpimath.geometry import Pose2d, Translation3d, Translation2d, Rotation2d
from ntcore import NetworkTableInstance
from wpilib import DigitalInput
from enum import Enum


MAX_ROTATION = degreesToRadians(270)
TURRET_GAP = math.tau - MAX_ROTATION
YAW_GEARING = 12  # TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation
PITCH_GEARING = 2

# TODO find feild positions for each
RED_RIGHT_SHUTTLE_POS: Translation3d = Translation3d()
RED_LEFT_SHUTTLE_POS: Translation3d = Translation3d()
RED_SCORE_POS: Translation3d = Translation3d()
BLUE_RIGHT_SHUTTLE_POS: Translation3d = Translation3d()
BLUE_LEFT_SHUTTLE_POS: Translation3d = Translation3d()
BLUE_SCORE_POS: Translation3d = Translation3d()

BALL_RADIUS: inches = 5.91 / 2
Y_PASS_DIFF: meters = inchesToMeters(15 + BALL_RADIUS)
GOAL_HEIGHT: meters = 1  # not entirly correct, distance between goal and turret opening
Y_PASS: meters = GOAL_HEIGHT + Y_PASS_DIFF
HUB_RADIUS: inches = 24.5 / 2
X_PASS_DIFF: meters = inchesToMeters(HUB_RADIUS)

FLYWHEEL_RADIUS: inches = 3  # diameter of the wheels build decides to use
FLYWHEEL_CIRCUMFRENCE: meters = inchesToMeters(FLYWHEEL_RADIUS) * PI
GRAVITY = 9.80665

MANUAL_SPEED: RPM = 120

TURRET_DIST_FROM_CENTER: meters = inchesToMeters(10)  # TODO make correct
TURRET_PATH_CIRCUMFRENCE: meters = TURRET_DIST_FROM_CENTER * TAU


# TODO add an enum for which position we are targeting, shuttle or scoring
class TurretTarget(Enum):
    HUB = 1
    RIGHT_SHUTTLE = 2
    LEFT_SHUTTLE = 3


class Turret(Subsystem):
    # cancoder more like cantcoder
    # yaw is horizontal rotation
    # pitch is vertical

    def __init__(self, yawMotorID, pitchMotorID):

        super().__init__()

        self.yawMotor = RevMotor(
            deviceID=yawMotorID
        )  # get right ID, motor for turning horizontally

        self.pitchMotor = RevMotor(deviceID=pitchMotorID)

        self.yawMotor.AZIMUTH_CONFIG = (
            SparkMaxConfig()
            .smartCurrentLimit(40)
            .inverted(True)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            .apply(
                LimitSwitchConfig()
                .reverseLimitSwitchEnabled(True)
                .forwardLimitSwitchEnabled(True)
            )
            .apply(
                ClosedLoopConfig()
                .pidf(0.15, 0, 0, 0)
                .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                .positionWrappingEnabled(False)
                .apply(
                    MAXMotionConfig()
                    .maxVelocity(5000, ClosedLoopSlot.kSlot0)
                    .maxAcceleration(10000, ClosedLoopSlot.kSlot0)
                    .allowedClosedLoopError(0.2)
                )
            )
        )

        self.pitchMotor.AZIMUTH_CONFIG.apply(
            LimitSwitchConfig().reverseLimitSwitchEnabled(True)
        )

        self.yawMotor.configure(config=self.yawMotor.AZIMUTH_CONFIG)
        self.pitchMotor.configure(config=self.pitchMotor.AZIMUTH_CONFIG)

        self.yawEncoder = self.yawMotor.getEncoder()
        self.pitchEncoder = self.pitchMotor.getEncoder()

        self.yawPos = rotationsToRadians(self.yawEncoder.getPosition())
        self.pitchPos = rotationsToRadians(self.pitchEncoder.getPosition())

        self.odom = TurretOdometry()

        # self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.turretAngle: radians = rotationsToRadians(self.pitchEncoder.getPosition())

        self.homeSet: bool = True
        self.yawSetPoint = 0  # in relation to the field
        self.targetPos: Translation3d = Translation3d()

        self.yawLimitSwitch = DigitalInput(10)
        self.pitchLimitSwitch = DigitalInput(0)  # TODO chnage

        self.manualMode = False
        self.manualToggle = False
        # these velocity values are only used when in manual mode
        self.yawVelocity = 0
        self.pitchVelocity = 0

        self.turretAutoDepedencies: tuple = (None,)
        self.turretManDependencies: tuple = (None,)
        self.turretGenDepedencies: tuple = (None,)

    def init(self):
        self.homeSet: bool = True  # TODO change back to false
        self.yawSetPoint = 0  # in relation to the field

        self.turretAutoDepedencies: tuple = (None,)
        self.turretManDependencies: tuple = (None,)
        self.turretGenDepedencies: tuple = (None,)

    def periodic(self, robotState: RobotState) -> RobotState:

        self.turretAutoDepedencies: tuple = (
            robotState.optimalTurretAngle,
            robotState.hubDistance,
        )
        self.turretManDependencies: tuple = (robotState.turretManualSetpoint,)
        self.turretGenDepedencies: tuple = (robotState.pose,)

        self.yawPos = rotationsToRadians(self.yawEncoder.getPosition())
        self.pitchPos = rotationsToRadians(self.pitchEncoder.getPosition())

        if not self.homeSet:
            self.reset(self.yawLimitSwitch.get() and self.pitchLimitSwitch.get())
            return robotState

        self.manualToggle = robotState.turretManualToggle
        if robotState.turretManualToggle:
            self.manualMode = not self.manualMode
            robotState.turretManulMode = self.manualMode

        if not checkDependencies(self.turretGenDepedencies):
            return robotState

        robotYaw = robotState.pose.rotation().radians()
        self.odom.updateWithEncoder(robotYaw, self.yawEncoder)

        if self.manualMode:
            self.manualUpdate(robotState)

        else:
            self.automaticUpdate(robotState)

        self.maintainRotation(robotYaw)  # these go last

    def automaticUpdate(self, robotState: RobotState):

        if not checkDependencies(self.turretAutoDepedencies):
            return

        self.targetPoint(self.targetPos, robotState.pose, robotState)  # need odom
        robotState.optimalTurretAngle = calculateAngle(robotState.hubDistance)

        self.dontOverdoIt()

        self.yawMotor.setPosition(self.yawSetPoint * YAW_GEARING)
        self.pitchMotor.setPosition(robotState.optimalTurretAngle * PITCH_GEARING)

    def manualUpdate(self, robotState: RobotState):

        if not checkDependencies(self.turretManDependencies):
            return

        setPoint = robotState.turretManualSetpoint

        self.yawVelocity = 0
        self.pitchVelocity = 0

        if setPoint == -1:
            return

        if setPoint > 0 and setPoint < 180:
            self.yawVelocity = 1

        elif setPoint > 180 and setPoint < 360:
            self.yawVelocity = -1

        if setPoint > 270 or setPoint < 90:
            self.pitchVelocity = 1

        elif setPoint > 90 and setPoint < 270:
            self.pitchVelocity = -1

        self.yawVelocity *= MANUAL_SPEED
        self.pitchVelocity *= MANUAL_SPEED

        self.dontOverdoIt()

        self.yawMotor.setVelocity(self.yawVelocity)
        self.pitchMotor.setVelocity(self.pitchVelocity)

    def compensateSetpoint(self, time, roboLinV, roboOmegaV):  # TODO finish
        turretRotV = roboOmegaV * TURRET_PATH_CIRCUMFRENCE / ROBOT_RADIUS

    def disabled(self):
        self.yawMotor.stopMotor()
        self.pitchMotor.stopMotor()

        self.yawMotor.configure(config=self.yawMotor.DISABLED_AZIMUTH_CONFIG)
        self.pitchMotor.configure(config=self.pitchMotor.DISABLED_AZIMUTH_CONFIG)
        # do we need these .configure lines when revmotor allready does this?

        self.homeSet = True

    def publish(self):

        self.publishBoolean("Turret Home Set", self.homeSet)
        self.publishDouble("Turret Yaw Setpoint", self.yawSetPoint)
        self.publishDouble(
            "Turret Yaw Actual Motor Setpoint", self.yawSetPoint * YAW_GEARING
        )
        self.publishDouble(
            "Turret Yaw Feild Relative Rotation", self.odom.feildRelativeRot
        )
        self.publishDouble(
            "Turret Yaw Feild Relative Rotation Wrapped", self.odom.wrappedFRR
        )
        self.publishDouble("Turret Yaw Encoder Motor Pos", self.yawPos)
        self.publishDouble("Turret Yaw Actual Encoder Pos", self.yawPos / YAW_GEARING)
        self.publishDouble("Turret Yaw Encoder Motor Pos", self.yawPos)
        self.publishDouble(
            "Turret Yaw Actual Encoder Pos", self.pitchPos / PITCH_GEARING
        )
        self.publishDouble("Manual Yaw Velocity", self.yawVelocity)
        self.publishDouble("Manual Pitch Velocity", self.pitchVelocity)
        self.publishBoolean("Turret Manual Mode Active", self.manualMode)
        self.publishBoolean("Turret Manual Toggle Button", self.manualToggle)

    def maintainRotation(self, robotYaw):
        self.yawSetPoint -= self.odom.getGyroChange(robotYaw)

    def dontOverdoIt(self):
        if self.yawSetPoint > MAX_ROTATION + TURRET_GAP / 2:
            self.yawSetPoint = 0
        elif self.yawSetPoint > MAX_ROTATION:
            self.yawSetPoint = MAX_ROTATION

    def targetPoint(
        self, pointPose: Translation3d, robotPose: Pose2d, robotState: RobotState
    ) -> None:  # make velocity later

        xDiff = pointPose.X() - robotPose.X()
        yDiff = pointPose.Y() - robotPose.Y()
        self.yawSetPoint = math.atan(xDiff / yDiff)  # gets the yaw angle
        robotState.hubDistance = math.sqrt(xDiff**2 + yDiff**2)  # good ol' pathagoras

    def reset(self, limit):
        if not self.homeSet:

            self.yawMotor.setVelocity(-1)
            self.pitchMotor.setVelocity(-1)

            if limit:
                self.yawEncoder.setPosition(0)
                self.pitchEncoder.setPosition(0)
                self.homeSet = True


class TurretOdometry:
    def __init__(self):
        self.feildRelativeRot = 0
        self.wrappedFRR = 0  # wrapped field relative rotation
        self.pos: Pose2d = Pose2d()

    def updateWithEncoder(self, roboYaw, encoder: SparkRelativeEncoder):

        turretRaw = encoder.getPosition()
        turretRot = rotationsToRadians(turretRaw) / YAW_GEARING
        self.wrappedFRR = (turretRot % TAU) * np.sign(
            turretRot
        )  # modulo only returns positive so multiply by the sign
        self.feildRelativeRot = roboYaw - turretRot

    def getGyroChange(self, yaw):
        wrappedRoboYaw = (yaw % TAU) * np.sign(yaw)

        return self.wrappedFRR - wrappedRoboYaw


class Shooter(Subsystem):

    def __init__(self):
        # self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.hubDistance = 0  # hypotenuse of odometry and the Hub position

        self.kickMotor = RevMotor(deviceID=14)
        self.kickMotorEncoder = self.kickMotor.getEncoder()

        self.revingSpeed: RPM = 0
        self.shooterSpeed: RPM = self.kickMotorEncoder.getPosition()

        self.revingMotorTop: RevMotor = RevMotor(deviceID=11)
        self.revingMotorBottom: RevMotor = RevMotor(deviceID=12)

        self.revTopEncoder = self.revingMotorTop.getEncoder()
        self.revBottomEncoder = self.revingMotorBottom.getEncoder()

        self.manualMode: bool = False
        self.manualRevSpeed: RPM = 0
        self.manualKickSpeed: RPM = 0

        self.dependencies: tuple = (None,)

        super().__init__()

    def init(self) -> None:
        self.dependencies: tuple = (None,)

    def periodic(self, robotState: RobotState) -> RobotState:

        self.manualMode = self.getBoolean("shooter manual", default=False)
        if self.manualMode:
            self.manualUpdate()
            return robotState

        self.dependencies = (
            robotState.revShooter,
            robotState.shootShooter,
            robotState.optimalTurretAngle,
        )
        if not checkDependencies(self.dependencies):
            return robotState

        if robotState.revShooter > 0.1:
            self.revingSpeed = _calculateVelocity(
                robotState.optimalTurretAngle, robotState.hubDistance
            )
        elif robotState.shootShooter == 1:
            self.kickMotor.setVelocity(50)
        else:
            self.disabled()
        self.revingMotorTop.setVelocity(self.revingSpeed)
        self.revingMotorBottom.setVelocity(self.revingSpeed)

        return robotState

    def manualUpdate(self):

        self.manualRevSpeed = self.getDouble("manual shooter rpm", default=0)
        self.revingMotorTop.setVelocity(self.manualRevSpeed)
        self.revingMotorBottom.setVelocity(self.manualRevSpeed)
        self.manualKickSpeed = self.getDouble("manual kick speed", default=0)
        self.kickMotor.setVelocity(self.manualKickSpeed)

    def disabled(self) -> None:
        self.kickMotor.setVelocity(0)
        self.revingSpeed = 0

    def publish(self):
        self.publishBoolean("shooter manual", self.manualMode)
        self.publishDouble("revMotor set speed", self.revingSpeed)
        self.publishDouble("manual shooter rpm", self.manualRevSpeed)
        self.publishDouble("manual kick rpm", self.manualKickSpeed)
        self.publishDouble(
            "Kick motor encoder rpm", self.kickMotorEncoder.getVelocity()
        )
        self.publishDouble(
            "top reving motor encoder rpm", self.revTopEncoder.getVelocity()
        )
        self.publishDouble(
            "bottom reving motor encoder rpm", self.revBottomEncoder.getVelocity()
        )


def calculateAngle(d: meters) -> radians:
    xPass = d - X_PASS_DIFF
    h = GOAL_HEIGHT

    numerator = Y_PASS * d**2 - xPass**2 * h
    denom = d * (xPass * d - xPass**2)

    return math.atan(numerator / denom)


def _calculateVelocity(turretAngle, hubDistance) -> float:

    velocityMps = math.sqrt(
        (GRAVITY * turretAngle**2)
        / (
            2
            * math.cos(turretAngle)
            * (hubDistance * math.tan(turretAngle) - GOAL_HEIGHT)
        )
    )
    return velocityMps / FLYWHEEL_CIRCUMFRENCE * 60


def calculateTime(velocity, distance):
    return distance / velocity


def checkDependencies(depends: tuple) -> bool:

    for var in depends:
        if var is None:
            return False

    return True
