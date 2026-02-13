from subsystems.motor import RevMotor
from subsystems.subsystem import Subsystem
from subsystems.robotState import (
    RobotState,
    TeamSide,
    TurretTarget,
    TurretMode,
    ROBOT_RADIUS,
    getTangentalVelocity,
    scaleTranslation2D,
    wrapAngle,
    MPSToRPM,
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


MAX_ROTATION: radians = degreesToRadians(270)
TURRET_GAP: radians = math.tau - MAX_ROTATION
# TODO offset in radians from the zero of the gyro and zero of the turret
ZERO_OFFSET: radians = 0
YAW_GEARING: float = (
    12  # TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation
)
PITCH_GEARING: float = 2

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
GRAVITY: MPS = 9.80665  # don't worry that it's positive

MANUAL_REV_SPEED: RPM = 3000  # TODO change to what emmet wants
MANUAL_SPEED: RPM = 120  # TODO tune, for the pitch and yaw speed
KICK_SPEED: RPM = 50

REV_ALLOWED_ERROR: RPM = 10  # TODO fine tune all these
YAW_ALLOWED_ERROR: radians = 0.05
PITCH_ALLOWED_ERROR: radians = 0.05


TURRET_DIST_FROM_CENTER: meters = inchesToMeters(10)  # TODO make correct
TURRET_PATH_CIRCUMFRENCE: meters = TURRET_DIST_FROM_CENTER * TAU


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

        self.turretAngle: radians = rotationsToRadians(self.pitchEncoder.getPosition())

        self.yawLimitSwitch = DigitalInput(10)
        self.pitchLimitSwitch = DigitalInput(0)  # TODO chage to be correct

        self.phaseInit()

    def phaseInit(self):
        self.homeSet: bool = False
        self.yawSetPoint: radians = 0  # in relation to the field
        self.relativeYawSetpoint: radians = 0  # in relation to the robot
        self.pitchSetpoint: radians = 0
        self.targetPos: Translation3d = Translation3d()

        self.targetLocked: bool = False

        self.target: TurretTarget = TurretTarget.NONE
        self.mode: TurretMode = TurretMode.MANUAL

        self.turretAutoDepedencies: tuple = (None,)
        self.turretManDependencies: tuple = (None,)
        self.turretGenDepedencies: tuple = (None,)

        # these velocity values are only used when in manual mode
        self.yawVelocity = 0
        self.pitchVelocity = 0

    def periodic(self, robotState: RobotState) -> RobotState:

        self.mode = self.getMode(robotState)

        self.targetLocked = self.getTargetLocked()

        self.turretAutoDepedencies: tuple = (
            robotState.optimalTurretAngle,
            robotState.targetDistance,
        )
        self.turretManDependencies: tuple = (robotState.turretManualSetpoint,)
        self.turretGenDepedencies: tuple = (robotState.pose,)

        if not self.homeSet:
            self.reset(self.yawLimitSwitch.get() and self.pitchLimitSwitch.get())
            return robotState

        # self.manualToggle = robotState.turretManualToggle #TODO uncomment (was for testing)

        if not checkDependencies(self.turretGenDepedencies):
            return robotState

        robotPose = robotState.pose
        self.odom.updateWithEncoder(robotPose, self.yawEncoder, self.pitchEncoder)
        self.yawAngle = self.odom.pose.rotation().radians()
        self.pitchAngle = self.odom.pitch

        if self.mode == TurretMode.MANUAL:
            self.manualUpdate(robotState)

        elif self.mode == TurretMode.DYNAMIC:
            self.target = self.getTarget(robotState)
            self.targetPos = self.getTargetPos(self.target, robotState.teamSide)
            self.automaticUpdate(robotState)

        elif self.mode == TurretMode.DISABLED:
            self.reset(self.yawLimitSwitch.get() and self.pitchLimitSwitch.get())

        return robotState

    def automaticUpdate(self, robotState: RobotState):

        if not checkDependencies(self.turretAutoDepedencies):
            return

        h = self.targetPos.z
        d = robotState.targetDistance
        angle = calculateAngle(d, h, self.getXPass(d), self.getYPass())
        velocity = _calculateVelocity(angle, d, h)
        time = calculateTime(velocity, d)
        self.compensateSetpoint(
            time, robotState.robotLinearVelocity, robotState.robotOmegaSpeed
        )

        self.targetPoint(self.targetPos, self.odom.pose, robotState)  # need robot odom

        self.dontOverdoIt()

        self.pitchSetpoint = robotState.optimalTurretAngle
        self.yawMotor.setPosition(self.relativeYawSetpoint * YAW_GEARING)
        self.pitchMotor.setPosition(self.pitchSetpoint * PITCH_GEARING)

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
        # self.maintainRotation(robotYaw) #TODO remake

        self.yawMotor.setVelocity(self.yawVelocity)
        self.pitchMotor.setVelocity(self.pitchVelocity)

    def getMode(self, rs: RobotState) -> TurretMode:

        mode = self.mode
        if rs.putAwayTurret:
            mode = (
                TurretMode.DISABLED
                if not (self.mode == TurretMode.DISABLED)
                else self.mode
            )
            return mode

        if rs.turretManualToggle:
            mode = (
                TurretMode.DYNAMIC
                if (self.mode == TurretMode.MANUAL)
                else TurretMode.MANUAL
            )

        return mode

    def getTarget(self, rs: RobotState) -> TurretTarget:  # TODO

        target: TurretTarget = self.target

        if rs.turretSwitchTarget:
            if not (self.target == TurretTarget.HUB):
                target = TurretTarget.HUB

        if not (target == TurretTarget.HUB):
            pass  # TODO determine side of field

        # TODO check if path will interfere with thing then

        return target

    def getTargetPos(self, target: TurretTarget, side: TeamSide) -> Translation3d:

        pos: Translation3d = self.targetPos

        if side == TeamSide.SIDE_BLUE:
            match target:
                case TurretTarget.HUB:
                    return BLUE_SCORE_POS

                case TurretTarget.SHUTTLE_RIGHT:
                    return BLUE_RIGHT_SHUTTLE_POS

                case TurretTarget.SHUTTLE_LEFT:
                    return BLUE_LEFT_SHUTTLE_POS

                case _:
                    return pos
        elif side == TeamSide.SIDE_RED:
            match target:
                case TurretTarget.HUB:
                    return RED_SCORE_POS

                case TurretTarget.SHUTTLE_RIGHT:
                    return RED_RIGHT_SHUTTLE_POS

                case TurretTarget.SHUTTLE_LEFT:
                    return RED_LEFT_SHUTTLE_POS

                case _:
                    return pos

    def getTargetLocked(self) -> bool:

        if abs(self.relativeYawSetpoint - self.yawAngle) > YAW_ALLOWED_ERROR:
            return False

        if abs(self.pitchSetpoint - self.pitchAngle) > PITCH_ALLOWED_ERROR:
            return False

        return True

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
        self.targetPos.__add__(Translation3d(compensateVector.rotateBy(Rotation2d(PI))))

    def dontOverdoIt(self):
        if self.relativeYawSetpoint > MAX_ROTATION + TURRET_GAP / 2:
            self.relativeYawSetpoint = 0
        elif self.relativeYawSetpoint > MAX_ROTATION:
            self.relativeYawSetpoint = MAX_ROTATION

    def targetPoint(
        self, pointPose: Translation3d, turretPose: Pose2d, robotState: RobotState
    ) -> None:  # make velocity later

        xDiff = pointPose.X() - turretPose.X()
        yDiff = pointPose.Y() - turretPose.Y()
        robotState.targetHeight = pointPose.z

        self.yawSetPoint = math.atan(xDiff / yDiff)  # gets the yaw angle
        self.relativeYawSetpoint = (
            self.yawSetPoint
            - wrapAngle(robotState.pose.rotation().radians())
            + ZERO_OFFSET
        )

        robotState.targetDistance = math.sqrt(
            xDiff**2 + yDiff**2
        )  # good ol' pathagoras

        d = robotState.targetDistance
        h = robotState.targetHeight

        robotState.optimalTurretAngle = calculateAngle(
            d, h, self.getXPass(d), self.getYPass()
        )

    def getXPass(self, d) -> meters:
        xPass = d - X_PASS_DIFF_HUB  # for hub

        if (
            self.target == TurretTarget.SHUTTLE_LEFT
            or self.target == TurretTarget.SHUTTLE_RIGHT
        ):
            xPass = d - SHUTTLE_X_PASS_DIFF  # for shuttle positions

        return xPass

    def getYPass(self) -> meters:
        yPass = Y_PASS_HUB
        if (
            self.target == TurretTarget.SHUTTLE_LEFT
            or self.target == TurretTarget.SHUTTLE_RIGHT
        ):
            yPass = SHUTTLE_Y_PASS

        return yPass

    def reset(self, limit: bool):
        if not self.homeSet:

            self.yawMotor.setVelocity(-1)
            self.pitchMotor.setVelocity(-1)

            if limit:
                self.yawEncoder.setPosition(0)
                self.pitchEncoder.setPosition(0)
                self.homeSet = True

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
        self.publishFloat("Turret Manual Yaw Velocity", self.yawVelocity)
        self.publishFloat("Turret Manual Pitch Velocity", self.pitchVelocity)
        self.publishGeneric("Turret Mode", self.mode)
        self.publishGeneric("Turret Target", self.target)
        self.publishGeneric("Turret target Position", self.targetPos)
        self.publishBoolean("Turret Target Locked", self.targetLocked)


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

        wrappedRoboYaw = wrapAngle(roboPose.rotation().radians())

        robotRelativeX = math.cos(wrappedRoboYaw) * TURRET_DIST_FROM_CENTER
        robotRelativeY = math.sin(wrappedRoboYaw) * TURRET_DIST_FROM_CENTER

        self.robotRelativePos = Translation2d(robotRelativeX, robotRelativeY)
        feildPos: Translation2d = roboPose.translation().__add__(self.robotRelativePos)

        self.pose = Pose2d(feildPos, Rotation2d(wrappedRoboYaw - yawAngle))
        self.pitch = pitchAngle


class Shooter(Subsystem):

    def __init__(self):
        super().__init__()

        self.kickMotor = RevMotor(deviceID=18)
        self.revingMotorTop: RevMotor = RevMotor(deviceID=12)
        self.revingMotorBottom: RevMotor = RevMotor(deviceID=11)

        self.kickMotorEncoder = self.kickMotor.getEncoder()
        self.revTopEncoder = self.revingMotorTop.getEncoder()
        self.revBottomEncoder = self.revingMotorBottom.getEncoder()

        self.revingSetpoint: RPM = 0
        self.revingSpeedTop: RPM = 0
        self.revingSpeedBottom: RPM = 0
        self.kickSpeed: RPM = self.kickMotorEncoder.getPosition()  # read value

        self.manualMode: bool = True
        self.manualRevSpeed: RPM = 2500
        self.manualKickSpeed: RPM = 50

        self.dependencies: tuple = (None,)

        self.fullyReved: bool = False

        self.publishFloat("Manual rev cap", self.manualRevSpeed)
        self.publishFloat("manual kick rpm", self.manualKickSpeed)
        self.publishBoolean("shooter manual", self.manualMode)

    def phaseInit(self) -> None:
        self.dependencies: tuple = (None,)

    def periodic(self, robotState: RobotState) -> RobotState:

        self.manualMode = self.getBoolean("shooter manual", default=False)
        self.manualRevSpeed = self.getFloat("Manual rev cap", default=0)
        self.manualKickSpeed = self.getFloat("manual kick rpm", default=0)

        self.fullyReved = self.getFullyReved()

        self.kickSpeed: RPM = self.kickMotorEncoder.getPosition()

        self.dependencies = (
            robotState.revSpeed,
            robotState.kickShooter,
            robotState.optimalTurretAngle,
        )

        if not checkDependencies(self.dependencies):
            return robotState

        if self.manualMode:
            self.manualUpdate(robotState)

        else:
            self.dynamicUpdate(robotState)

        self.revingSetpoint *= robotState.revSpeed  # multiplied by the trigger value

        self.revShooters(self.revingSetpoint)

        self.kickMotor.setVelocity(self.manualKickSpeed * int(robotState.kickShooter))

        return robotState

    def manualUpdate(self, robotState: RobotState):

        self.revingSetpoint = robotState.revSpeed * self.manualRevSpeed

    def dynamicUpdate(self, robotState: RobotState) -> None:

        mpsSetpoint: MPS = _calculateVelocity(
            robotState.optimalTurretAngle,
            robotState.targetDistance,
            robotState.targetHeight,
        )
        # TODO add a constant scale value to the speed
        self.revingSetpoint: RPM = MPSToRPM(mpsSetpoint, FLYWHEEL_CIRCUMFRENCE)

    def revShooters(self, speed: RPM):
        self.revingMotorBottom.setVelocity(speed)
        self.revingMotorTop.setVelocity(speed)

    def getFullyReved(self) -> bool:

        if (
            abs(self.revingSetpoint - self.revingSpeedTop) > REV_ALLOWED_ERROR
            and abs(self.revingSetpoint - self.revingSpeedBottom) > REV_ALLOWED_ERROR
        ):
            return False

        return True

    def disabled(self) -> None:
        self.kickMotor.setVelocity(0)
        self.revingSetpoint = 0

    def publish(self):

        self.publishFloat("revMotor set speed", self.revingSetpoint)
        self.publishFloat("Kick motor encoder rpm", self.kickMotorEncoder.getVelocity())
        self.publishFloat(
            "top reving motor encoder rpm", self.revTopEncoder.getVelocity()
        )
        self.publishFloat(
            "bottom reving motor encoder rpm", self.revBottomEncoder.getVelocity()
        )
        self.publishBoolean("Turret Fully Reved", self.fullyReved)


def calculateAngle(d: meters, h: meters, xPass: meters, yPass: meters) -> radians:

    numerator = yPass * d**2 - xPass**2 * h
    denom = d * (xPass * d - xPass**2)

    return math.atan(numerator / denom)


def _calculateVelocity(turretAngle, hubDistance, height) -> MPS:

    velocityMps = math.sqrt(
        (GRAVITY * turretAngle**2)
        / (2 * math.cos(turretAngle) * (hubDistance * math.tan(turretAngle) - height))
    )
    return velocityMps / FLYWHEEL_CIRCUMFRENCE * 60


def calculateTime(velocity, distance):
    return distance / velocity


def checkDependencies(depends: tuple) -> bool:

    for var in depends:
        if var is None:
            return False

    return True
