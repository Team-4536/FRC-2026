from subsystems.motor import RevMotor
from subsystems.subsystem import Subsystem
from subsystems.robotState import (
    RobotState,
    ROBOT_RADIUS,
    getTangentalVelocity,
    scaleTranslation2D,
)
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
    meters_per_second as MPS,
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
# TODO offset in radians from the zero of the gyro and zero of the turret
ZERO_OFFSET = 0
YAW_GEARING = 12  # TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation
PITCH_GEARING = 2

BATTERY_VOLTS: float = 12
MAX_RPM: RPM = 5676

# TODO find feild positions for each
RED_RIGHT_SHUTTLE_POS: Translation3d = Translation3d()
RED_LEFT_SHUTTLE_POS: Translation3d = Translation3d()
RED_SCORE_POS: Translation3d = Translation3d()
BLUE_RIGHT_SHUTTLE_POS: Translation3d = Translation3d()
BLUE_LEFT_SHUTTLE_POS: Translation3d = Translation3d()
BLUE_SCORE_POS: Translation3d = Translation3d()

BALL_RADIUS: inches = 5.91 / 2
Y_PASS_DIFF_HUB: meters = inchesToMeters(15 + BALL_RADIUS)
GOAL_HEIGHT: meters = 1  # not entirly correct, distance between goal and turret opening
Y_PASS_HUB: meters = GOAL_HEIGHT + Y_PASS_DIFF_HUB
HUB_RADIUS: inches = 24.5 / 2
X_PASS_DIFF_HUB: meters = inchesToMeters(HUB_RADIUS)

SHUTTLE_Y_PASS_DIFF: meters = 0.3
SHUTTLE_Y_PASS: meters = SHUTTLE_Y_PASS_DIFF
SHUTTLE_X_PASS_DIFF: meters

FLYWHEEL_RADIUS: inches = 3  # diameter of the wheels build decides to use
FLYWHEEL_CIRCUMFRENCE: meters = inchesToMeters(FLYWHEEL_RADIUS) * PI
GRAVITY = 9.80665

MANUAL_SPEED: RPM = 120
KICK_SPEED: RPM = 50

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

        self.yawMotor.configure(config=self.yawMotor.TURRET_YAW_CONFIG)
        self.pitchMotor.configure(config=self.pitchMotor.AZIMUTH_CONFIG)

        self.yawEncoder = self.yawMotor.getEncoder()
        self.pitchEncoder = self.pitchMotor.getEncoder()

        self.yawAngle = rotationsToRadians(self.yawEncoder.getPosition())
        self.pitchAngle = rotationsToRadians(self.pitchEncoder.getPosition())

        self.odom = TurretOdometry()
        self.targetType: TurretTarget = TurretTarget.HUB

        # self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.turretAngle: radians = rotationsToRadians(self.pitchEncoder.getPosition())

        self.homeSet: bool = True
        self.yawSetPoint = 0  # in relation to the field
        self.targetPos: Translation3d = Translation3d()

        self.yawLimitSwitch = DigitalInput(10)
        self.pitchLimitSwitch = DigitalInput(0)  # TODO chnage

        self.manualMode = True
        self.manualToggle = False
        # these velocity values are only used when in manual mode
        self.yawVelocity = 0
        self.pitchVelocity = 0

        self.turretAutoDepedencies: tuple = (None,)
        self.turretManDependencies: tuple = (None,)
        self.turretGenDepedencies: tuple = (None,)

        self.setPoint: Translation3d = Translation3d()

    def phaseInit(self):
        self.homeSet: bool = True  # TODO change back to false
        self.yawSetPoint = 0  # in relation to the field

        self.turretAutoDepedencies: tuple = (None,)
        self.turretManDependencies: tuple = (None,)
        self.turretGenDepedencies: tuple = (None,)

        self.setPoint: Translation3d = Translation3d()

    def periodic(self, robotState: RobotState) -> RobotState:

        self.turretAutoDepedencies: tuple = (
            robotState.optimalTurretAngle,
            robotState.targetDistance,
        )
        self.turretManDependencies: tuple = (robotState.turretManualSetpoint,)
        self.turretGenDepedencies: tuple = (robotState.pose,)

        # if not self.homeSet:
        #     self.reset(self.yawLimitSwitch.get() and self.pitchLimitSwitch.get())
        #     return robotState

        # self.manualToggle = robotState.turretManualToggle #TODO uncomment (was for testing)
        # if robotState.turretManualToggle:
        #     self.manualMode = not self.manualMode
        #     robotState.turretManulMode = self.manualMode

        if not checkDependencies(self.turretGenDepedencies):
            return robotState

        robotPose = robotState.pose
        self.odom.updateWithEncoder(robotPose, self.yawEncoder, self.pitchEncoder)
        self.yawAngle = self.odom.pose.rotation().radians()
        self.pitchAngle = self.odom.pitch

        if self.manualMode:
            self.manualUpdate(robotState)

        else:
            self.automaticUpdate(robotState)

        # self.maintainRotation(robotYaw)  # these go last

        return robotState

    def automaticUpdate(self, robotState: RobotState):

        if not checkDependencies(self.turretAutoDepedencies):
            return

        h = self.setPoint.z
        d = robotState.targetDistance
        robotState.optimalTurretAngle = calculateAngle(
            d, h, self.getXPass(d), self.getYPass()
        )

        self.targetPoint(self.targetPos, self.odom.pose, robotState)  # need odom

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

        self.yawMotor.setVelocity(RPMToVolts(self.yawVelocity))
        self.pitchMotor.setVelocity(RPMToVolts(self.pitchVelocity))

    def compensateSetpoint(
        self, time: float, roboLinV: Translation2d, roboOmegaSpeed: MPS
    ):  # TODO finish

        compensateVector: Translation2d = Translation2d()

        offset: Translation2d = self.odom.robotRelativePos
        tanVel: Translation2d = getTangentalVelocity(offset, roboOmegaSpeed)

        # add the mps values
        compensateVector.__add__(tanVel)
        compensateVector.__add__(roboLinV)

        # multiply by time to get the distance the ball would move
        compensateVector = scaleTranslation2D(compensateVector, time)

        # add in the opposite direction
        self.setPoint.__add__(Translation3d(compensateVector.rotateBy(Rotation2d(PI))))

    def disabled(self):
        self.yawMotor.stopMotor()
        self.pitchMotor.stopMotor()

        self.yawMotor.configure(config=self.yawMotor.DISABLED_AZIMUTH_CONFIG)
        self.pitchMotor.configure(config=self.pitchMotor.DISABLED_AZIMUTH_CONFIG)
        # do we need these .configure lines when revmotor allready does this?

        self.homeSet = True

    def publish(self):

        self.publishBoolean("Turret Home Set", self.homeSet)
        self.publishFloat("Turret Yaw Setpoint", self.yawSetPoint)
        self.publishFloat(
            "Turret Yaw Actual Motor Setpoint", self.yawSetPoint * YAW_GEARING
        )
        self.publishFloat(
            "Turret Yaw Feild Relative Rotation", self.odom.pose.rotation().radians()
        )
        self.publishFloat("Turret Yaw Encoder Motor Pos", self.yawAngle)
        self.publishFloat(
            "Turret Yaw Actual Encoder Pos", self.pitchAngle / PITCH_GEARING
        )
        self.publishFloat("Manual Yaw Velocity", self.yawVelocity)
        self.publishFloat("Manual Pitch Velocity", self.pitchVelocity)
        self.publishBoolean("Turret Manual Mode Active", self.manualMode)
        self.publishBoolean("Turret Manual Toggle Button", self.manualToggle)

    def dontOverdoIt(self):
        if self.yawSetPoint > MAX_ROTATION + TURRET_GAP / 2:
            self.yawSetPoint = 0
        elif self.yawSetPoint > MAX_ROTATION:
            self.yawSetPoint = MAX_ROTATION

    def targetPoint(
        self, pointPose: Translation3d, turretPose: Pose2d, robotState: RobotState
    ) -> None:  # make velocity later

        xDiff = pointPose.X() - turretPose.X()
        yDiff = pointPose.Y() - turretPose.Y()
        robotState.targetHeight = pointPose.z

        self.yawSetPoint = math.atan(xDiff / yDiff)  # gets the yaw angle
        relativeRot = (
            self.yawSetPoint
            - wrapAngle(robotState.pose.rotation().radians())
            + ZERO_OFFSET
        )
        self.yawMotor.setPosition(relativeRot * YAW_GEARING)

        robotState.targetDistance = math.sqrt(
            xDiff**2 + yDiff**2
        )  # good ol' pathagoras

        d = robotState.targetDistance
        h = robotState.targetHeight

        robotState.optimalTurretAngle = calculateAngle(
            d, h, self.getXPass(d), self.getYPass()
        )

        self.pitchMotor.setPosition(robotState.optimalTurretAngle * PITCH_GEARING)

    def getXPass(self, d) -> meters:
        xPass = d - X_PASS_DIFF_HUB  # for hub

        if (
            self.targetType == TurretTarget.LEFT_SHUTTLE
            or self.targetType == TurretTarget.RIGHT_SHUTTLE
        ):
            xPass = d - SHUTTLE_X_PASS_DIFF  # for shuttle positions

        return xPass

    def getYPass(self) -> meters:
        yPass = Y_PASS_HUB
        if (
            self.targetType == TurretTarget.LEFT_SHUTTLE
            or self.targetType == TurretTarget.RIGHT_SHUTTLE
        ):
            yPass = SHUTTLE_Y_PASS

        return yPass

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

        self.pitch: radians = 0
        self.robotRelativePos: Translation2d = Translation2d()
        self.pose: Pose2d = Pose2d()

    def updateWithEncoder(
        self,
        roboPose: Pose2d,
        yawEncoder: SparkRelativeEncoder,
        pitchEncoder: SparkRelativeEncoder,
    ):

        yawRaw = yawEncoder.getPosition()
        yawAngle = rotationsToRadians(yawRaw) / YAW_GEARING

        pitchRaw = pitchEncoder.getPosition()
        pitchAngle = rotationsToRadians(pitchRaw) / PITCH_GEARING

        wrappedTurretYaw = wrapAngle(yawAngle)
        wrappedPitch = wrapAngle(pitchAngle)
        wrappedRoboYaw = wrapAngle(roboPose.rotation().radians())

        robotRelativeX = math.cos(wrappedRoboYaw) * TURRET_DIST_FROM_CENTER
        robotRelativeY = math.sin(wrappedRoboYaw) * TURRET_DIST_FROM_CENTER

        self.robotRelativePos = Translation2d(robotRelativeX, robotRelativeY)
        feildPos: Translation2d = roboPose.translation().__add__(self.robotRelativePos)

        self.pose = Pose2d(feildPos, Rotation2d(wrappedRoboYaw - wrappedTurretYaw))
        self.pitch = wrappedPitch


class Shooter(Subsystem):

    def __init__(self):
        # self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.hubDistance = 0  # hypotenuse of odometry and the Hub position

        self.kickMotor = RevMotor(deviceID=18)
        self.kickMotorEncoder = self.kickMotor.getEncoder()

        self.revingSpeed: RPM = 0
        self.shooterSpeed: RPM = self.kickMotorEncoder.getPosition()

        self.revingMotorTop: RevMotor = RevMotor(deviceID=12)
        self.revingMotorBottom: RevMotor = RevMotor(deviceID=11)

        self.revTopEncoder = self.revingMotorTop.getEncoder()
        self.revBottomEncoder = self.revingMotorBottom.getEncoder()

        self.manualMode: bool = True
        self.manualRevSpeed: RPM = 2500
        self.manualKickSpeed: RPM = 50

        self.dependencies: tuple = (None,)

        self.publishFloat("Manual rev cap", self.manualRevSpeed)
        self.publishFloat("manual kick rpm", self.manualKickSpeed)
        self.publishBoolean("shooter manual", self.manualMode)

        super().__init__()

    def phaseInit(self) -> None:
        self.dependencies: tuple = (None,)

    def periodic(self, robotState: RobotState) -> RobotState:

        self.manualMode = self.getBoolean("shooter manual", default=False)
        self.manualRevSpeed = self.getFloat("Manual rev cap", default=0)
        self.revingSpeed = RPMToVolts(robotState.revSpeed * self.manualRevSpeed)

        self.dependencies = (
            robotState.revSpeed,
            robotState.kickShooter,
            robotState.optimalTurretAngle,
        )

        if not checkDependencies(self.dependencies):
            return robotState

        if self.manualMode:
            self.manualUpdate(robotState)
            return robotState

        return robotState

    def manualUpdate(self, robotState: RobotState):

        self.revingMotorTop.setVelocity(self.revingSpeed)
        self.revingMotorBottom.setVelocity(self.revingSpeed)

        self.manualKickSpeed = self.getFloat("manual kick rpm", default=0)
        self.kickMotor.setVelocity(self.manualKickSpeed * int(robotState.kickShooter))

    def dynamicUpdate(self, robotState: RobotState) -> None:

        if not robotState.targetLocked:
            return

        self.revingSpeed: MPS = _calculateVelocity(
            robotState.optimalTurretAngle, robotState.targetDistance
        )
        self.revShooters(self.revingSpeed)

    def revShooters(self, speed: RPM):
        self.revingMotorBottom.setVelocity(speed)
        self.revingMotorTop.setVelocity(speed)

    def disabled(self) -> None:
        self.kickMotor.setVelocity(0)
        self.revingSpeed = 0

    def publish(self):

        self.publishFloat("revMotor set speed", self.revingSpeed)
        self.publishFloat("Kick motor encoder rpm", self.kickMotorEncoder.getVelocity())
        self.publishFloat(
            "top reving motor encoder rpm", self.revTopEncoder.getVelocity()
        )
        self.publishFloat(
            "bottom reving motor encoder rpm", self.revBottomEncoder.getVelocity()
        )


def calculateAngle(d: meters, h: meters, xPass: meters, yPass: meters) -> radians:

    numerator = yPass * d**2 - xPass**2 * h
    denom = d * (xPass * d - xPass**2)

    return math.atan(numerator / denom)


def _calculateVelocity(turretAngle, hubDistance) -> MPS:

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


def RPMToVolts(rpm: RPM) -> float:

    return rpm / (MAX_RPM / BATTERY_VOLTS)


def wrapAngle(angle: radians) -> radians:

    # mod only returns positive like a bum
    wrappedAngle: radians = (angle % TAU) * np.sign(angle)
    return wrappedAngle
