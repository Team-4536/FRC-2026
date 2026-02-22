from subsystems.motor import RevMotor
from subsystems.subsystem import Subsystem
from subsystems.robotState import (
    RobotState,
    TeamSide,
    TurretTarget,
    TurretMode,
    ROBOT_RADIUS,
)
from subsystems.utils import (
    RPMToVolts,
    getTangentalVelocity,
    scaleTranslation2D,
    wrapAngle,
    MPSToRPM,
    FIELD_LEN,
    FIELD_WIDTH,
)
from rev import (
    SparkRelativeEncoder,
    SoftLimitConfig,
)
from subsystems.inputs import Inputs
from subsystems.robotState import RobotState
from phoenix6.hardware import CANcoder
from wpimath.units import (
    rotationsToRadians,
    degreesToRadians,
    rotationsToDegrees,
    inchesToMeters,
    radiansToRotations,
    revolutions_per_minute as RPM,
    radians,
    meters,
    inches,
    meters_per_second as MPS,
    seconds,
)
import math
from math import tau as TAU, pi as PI
import numpy as np
from wpimath.geometry import Pose2d, Translation3d, Translation2d, Rotation2d
from wpilib import getTime
from ntcore import NetworkTableInstance
from wpilib import DigitalInput
from enum import Enum
from typing import Optional

MAX_PITCH: radians = degreesToRadians(90)  # in relation to up
MIN_PITCH: radians = degreesToRadians(40)
MAX_ROTATION: radians = PI
TURRET_GAP: radians = TAU - MAX_ROTATION
# TODO offset in radians from the zero of the gyro and zero of the turret
ZERO_OFFSET: radians = (MAX_ROTATION - PI) / 2 + (PI / 2)
YAW_GEARING: float = 100 / 3
PITCH_RADIUS: inches = 9
LIL_PITCH_GEAR_RADIUS: inches = 1
ARC_RATIO = (
    PITCH_RADIUS / LIL_PITCH_GEAR_RADIUS
)  # how many rotations of the smol ladder gear is 1 rotation of the pitch
PITCH_GEARING: float = 25 * ARC_RATIO  # two 5:1 gearboxes make 25
TURRET_HEIGHT: meters = inchesToMeters(15)

BALL_RADIUS: inches = 5.91 / 2
Y_PASS_DIFF_HUB: meters = inchesToMeters(15 + BALL_RADIUS)
GOAL_HEIGHT: meters = 1  # not entirly correct, distance between goal and turret opening
Y_PASS_HUB: meters = GOAL_HEIGHT + Y_PASS_DIFF_HUB
HUB_RADIUS: inches = 24.5 / 2
X_PASS_DIFF_HUB: meters = inchesToMeters(HUB_RADIUS)

SHUTTLE_Y_PASS_DIFF: meters = 0.3
SHUTTLE_Y_PASS: meters = SHUTTLE_Y_PASS_DIFF
SHUTTLE_X_PASS_DIFF: meters  # TODO the distance between where you want it to land and where you want it to clear

MAX_RPM: RPM = 5676

HUB_DIST_X: meters = inchesToMeters(158.6) + HUB_RADIUS
HUB_DIST_Y: meters = FIELD_WIDTH / 2
HUB_HEIGHT_Z: meters = inchesToMeters(73 - 15)

SHUTTLE_DIST_X: meters = 3
RIGHT_SHUTTLE_DIST_Y: meters = FIELD_WIDTH - 3
LEFT_SHUTTLE_DIST_Y: meters = 3

# TODO find feild positions for each
# x and y should be the same as what the robot thinks those are, z is height (in meters)
RED_RIGHT_SHUTTLE_POS: Translation3d = Translation3d(
    FIELD_LEN - SHUTTLE_DIST_X, FIELD_WIDTH - RIGHT_SHUTTLE_DIST_Y, 0
)
RED_LEFT_SHUTTLE_POS: Translation3d = Translation3d(
    FIELD_LEN - SHUTTLE_DIST_X, FIELD_WIDTH - LEFT_SHUTTLE_DIST_Y, 0
)
RED_SCORE_POS: Translation3d = Translation3d(
    FIELD_LEN - HUB_DIST_X,
    FIELD_WIDTH - HUB_DIST_Y,
    HUB_HEIGHT_Z - TURRET_HEIGHT,
)
BLUE_RIGHT_SHUTTLE_POS: Translation3d = Translation3d(
    SHUTTLE_DIST_X, RIGHT_SHUTTLE_DIST_Y, 0
)
BLUE_LEFT_SHUTTLE_POS: Translation3d = Translation3d(
    SHUTTLE_DIST_X, LEFT_SHUTTLE_DIST_Y, 0
)
BLUE_SCORE_POS: Translation3d = Translation3d(
    HUB_DIST_X,
    HUB_DIST_Y,
    HUB_HEIGHT_Z - TURRET_HEIGHT,
)

FLYWHEEL_RADIUS: inches = 3  # diameter of the wheels build decides to use
FLYWHEEL_CIRCUMFRENCE: meters = inchesToMeters(FLYWHEEL_RADIUS) * TAU
GRAVITY: MPS = 9.80665  # don't worry that it's positive
VELOCITY_SCALAR = 1  # to account for slight air drag

MANUAL_REV_SPEED: RPM = 3500  # TODO change to what emmet wants
MANUAL_SPEED: RPM = 50  # TODO tune, for the pitch and yaw speed
KICK_SPEED: RPM = 2000

# in percent
REV_ALLOWED_ERROR: float = 10  # TODO fine tune all these
# in radians
YAW_ALLOWED_ERROR: radians = 0.05
PITCH_ALLOWED_ERROR: radians = 0.05


TURRET_DIST_FROM_CENTER: meters = inchesToMeters(7.5)  # TODO make correct
TURRET_PATH_CIRCUMFRENCE: meters = TURRET_DIST_FROM_CENTER * TAU


class Turret(Subsystem):
    # cancoder more like cantcoder
    # yaw is horizontal rotation
    # pitch is vertical

    def __init__(self, yawMotorID: int, pitchMotorID: int):

        super().__init__()

        self.yawMotor = RevMotor(deviceID=yawMotorID)

        self.pitchMotor = RevMotor(deviceID=pitchMotorID)

        self.pitchMotor.configure(config=self.pitchMotor.TURRET_PITCH_CONFIG)
        self.yawMotor.configure(config=self.yawMotor.TURRET_YAW_CONFIG)

        self.yawEncoder = self.yawMotor.getEncoder()
        self.pitchEncoder = self.pitchMotor.getEncoder()

        self.yawEncoderPos = rotationsToRadians(self.yawEncoder.getPosition())
        self.yawAngle = 0  # yaw angle relative to the field
        self.pitchAngle = 0

        self.odom = TurretOdometry()

        self.turretAngle: radians = rotationsToRadians(self.pitchEncoder.getPosition())

        self.yawLimitSwitch = DigitalInput(10)
        self.pitchLimitSwitch = DigitalInput(0)  # TODO chage to be correct

        self.homeSet: bool = True
        self.yawSetPoint: radians = 0  # in relation to the field
        self.limitedYawSetpoint: radians = 0
        self.relativeYawSetpoint: radians = 0  # in relation to the robot
        self.pitchSetpoint: radians = PI / 2
        self.relativePitchSetpoint: radians = 0  # cuz zero points up
        self.targetPos: Translation3d = Translation3d()

        self.targetLocked: bool = False

        self.target: TurretTarget = TurretTarget.NONE
        self.mode: TurretMode = TurretMode.MANUAL

        self.turretAutoDepedencies: tuple = (None,)
        self.turretManDependencies: tuple = (None,)
        self.turretGenDepedencies: tuple = (None,)

        # these velocity values are only used when in manual mode
        self.yawVelocity: RPM = 0
        self.pitchVelocity: RPM = 0

        self.impossibleDynamic = False

        self.lastTime: seconds = getTime()

    def phaseInit(self, robotState: RobotState) -> RobotState:

        self.homeSet: bool = True
        self.yawSetPoint: radians = 0  # in relation to the field
        self.limitedYawSetpoint: radians = 0
        self.relativeYawSetpoint: radians = 0  # in relation to the robot
        self.pitchSetpoint: radians = PI / 2
        self.targetPos: Translation3d = Translation3d()

        self.targetLocked: bool = False

        self.target: TurretTarget = TurretTarget.HUB
        self.mode: TurretMode = TurretMode.DYNAMIC

        self.turretAutoDepedencies: tuple = (None,)
        self.turretManDependencies: tuple = (None,)
        self.turretGenDepedencies: tuple = (None,)

        # these velocity values are only used when in manual mode
        self.yawVelocity: RPM = 0
        self.pitchVelocity: RPM = 0

        self.impossibleDynamic = False

        self.lastTime: seconds = getTime()

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:

        if robotState.forceDynamicTurret:
            self.mode = TurretMode.DYNAMIC
        else:
            self.mode = self.getMode(robotState)

        if robotState.turretResetYawEncdoer:
            self.yawEncoder.setPosition(0)
            self.yawSetPoint = 0

        self.yawEncoderPos = rotationsToRadians(self.yawEncoder.getPosition())
        self.targetLocked = self.getTargetLocked()

        self.turretAutoDepedencies: tuple = (
            robotState.optimalTurretAngle,
            robotState.targetDistance,
        )
        self.turretManDependencies: tuple = (robotState.turretManualSetpoint,)
        self.turretGenDepedencies: tuple = (robotState.odometry.getEstimatedPosition(),)

        if not self.homeSet:
            self.reset(self.yawLimitSwitch.get() and self.pitchLimitSwitch.get())
            return robotState

        if not checkDependencies(self.turretGenDepedencies):
            return robotState

        robotPose = robotState.odometry.getEstimatedPosition()
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

        angle = 0
        velocity = 0
        time = 0

        try:
            angle = calculateAngle(d, h, self.getXPass(d), self.getYPass())
            velocity = _calculateVelocity(angle, d, h)
            time = calculateTime(velocity, d)

        except:
            self.impossibleDynamic = True
            robotState.dontShoot = True
            return

        self.compensateSetpoint(
            time, robotState.robotLinearVelocity, robotState.robotOmegaSpeed
        )

        self.targetPoint(self.targetPos, self.odom.pose, robotState)  # need robot odom

        robotState.optimalTurretAngle = self.dontOverDoItPitch(
            robotState.optimalTurretAngle
        )

        self.pitchSetpoint = self.getRelativePitchSetpoint(
            robotState.optimalTurretAngle
        )
        self.limitedYawSetpoint = self.dontOverdoItYaw(self.relativeYawSetpoint)

        self.yawMotor.setPosition(self.limitedYawSetpoint * YAW_GEARING)
        self.pitchMotor.setPosition(self.pitchSetpoint * PITCH_GEARING)

    def manualUpdate(self, robotState: RobotState):

        if not checkDependencies(self.turretManDependencies):
            return

        setPoint = robotState.turretManualSetpoint

        self.yawVelocity = 0
        self.pitchVelocity = 0

        if not setPoint == -1:
            if setPoint > 0 and setPoint < 180:
                self.yawVelocity = -1

            elif setPoint > 180 and setPoint < 360:
                self.yawVelocity = 1

            if setPoint > 270 or setPoint < 90:
                self.pitchVelocity = 1

            elif setPoint > 90 and setPoint < 270:
                self.pitchVelocity = -1

        if not (self.yawVelocity == 0) and not (
            self.relativeYawSetpoint == self.dontOverdoItYaw(self.relativeYawSetpoint)
        ):  # if we are manually moving interupt maintaining rotation in the turret gap

            self.yawSetPoint -= self.relativeYawSetpoint - self.dontOverdoItYaw(
                self.relativeYawSetpoint
            )

        self.pitchSetpoint = self.dontOverDoItPitch(self.pitchSetpoint)

        self.yawVelocity *= MANUAL_SPEED
        self.pitchVelocity *= MANUAL_SPEED

        time = getTime()
        timeDiff = time - self.lastTime  # time since last update
        # ensures it spins at a consistant speed
        self.pitchSetpoint += rotationsToRadians(self.pitchVelocity / 60 * timeDiff)
        self.yawSetPoint += rotationsToRadians(self.yawVelocity / 60 * timeDiff)

        self.lastTime = time

        self.yawSetPoint = wrapAngle(self.yawSetPoint)

        self.relativeYawSetpoint = (
            self.yawSetPoint
            - wrapAngle(robotState.odometry.getEstimatedPosition().rotation().radians())
            + ZERO_OFFSET
        )

        self.limitedYawSetpoint = self.dontOverdoItYaw(self.relativeYawSetpoint)
        self.relativePitchSetpoint = self.getRelativePitchSetpoint(self.pitchSetpoint)

        self.yawMotor.setPosition(self.limitedYawSetpoint * YAW_GEARING)
        self.pitchMotor.setPosition(self.relativePitchSetpoint * PITCH_GEARING)

    def getMode(self, rs: RobotState) -> TurretMode:

        mode = self.mode
        if rs.turretSwitchEnabled:
            mode = (
                TurretMode.DISABLED
                if not (self.mode == TurretMode.DISABLED)
                else self.mode
            )
            return mode

        if rs.turretSwitchMode:
            mode = (
                TurretMode.DYNAMIC
                if (self.mode == TurretMode.MANUAL)
                else TurretMode.MANUAL
            )

        return mode

    def getTarget(self, rs: RobotState) -> TurretTarget:

        target: TurretTarget = self.target

        if rs.turretSwitchTarget:
            if not (
                self.target == TurretTarget.HUB
            ):  # TODO it is a button rn but change to determine side of feild
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

        if abs(self.relativeYawSetpoint - self.yawEncoderPos) > YAW_ALLOWED_ERROR:
            return False

        if abs(self.pitchSetpoint - self.pitchAngle) > PITCH_ALLOWED_ERROR:
            return False

        return True

    def compensateSetpoint(
        self, time: float, roboLinV: Translation2d, roboOmegaSpeed: MPS
    ):

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

    def dontOverdoItYaw(self, angle) -> radians:

        if angle > MAX_ROTATION + (TURRET_GAP / 2) or (
            angle < 0 and angle > 0 - TURRET_GAP / 2
        ):
            return 0

        elif angle > MAX_ROTATION or angle < 0:
            return MAX_ROTATION

        return angle

    def dontOverDoItPitch(self, angle: radians) -> radians:

        if angle > MAX_PITCH:
            return MAX_PITCH

        elif angle < MIN_PITCH:
            return MIN_PITCH

        return angle

    def targetPoint(
        self, pointPose: Translation3d, turretPose: Pose2d, robotState: RobotState
    ) -> None:  # make velocity later

        xDiff = pointPose.X() - turretPose.X()
        yDiff = pointPose.Y() - turretPose.Y()
        robotState.targetHeight = pointPose.z

        self.yawSetPoint = math.atan(xDiff / yDiff)  # gets the yaw angle

        # self.yawSetPoint = wrapAngle(self.yawSetPoint)

        self.relativeYawSetpoint = (
            self.yawSetPoint
            - wrapAngle(robotState.odometry.getEstimatedPosition().rotation().radians())
            + ZERO_OFFSET
        )

        self.limitedYawSetpoint = self.dontOverdoItYaw(self.relativeYawSetpoint)

        robotState.targetDistance = math.sqrt(
            xDiff**2 + yDiff**2
        )  # good ol' pathagoras

        d = robotState.targetDistance
        h = robotState.targetHeight

        robotState.optimalTurretAngle = calculateAngle(
            d, h, self.getXPass(d), self.getYPass()
        )

    def getXPass(self, d: meters) -> meters:

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

    def getRelativePitchSetpoint(self, angle: radians) -> radians:
        return (PI / 2) - angle

    def reset(self, limit: bool):

        if limit:
            self.yawEncoder.setPosition(0)
            self.pitchEncoder.setPosition(0)
            self.homeSet = True

        else:
            self.yawMotor.setVelocity(-1)
            self.pitchMotor.setVelocity(-1)

    def disabled(self):
        self.yawMotor.stopMotor()
        self.pitchMotor.stopMotor()
        # do we need these .configure lines when revmotor allready does this?

        self.homeSet = True

    def publish(self):

        self.publishBoolean("Home Set", self.homeSet)
        self.publishFloat("Yaw Setpoint", self.yawSetPoint)

        self.publishFloat("Yaw Robot Relative Setpoint", self.relativeYawSetpoint)
        self.publishFloat("Yaw Feild Relative Rotation", self.yawAngle)
        self.publishFloat("Yaw Motor Pos", self.yawEncoderPos)
        self.publishFloat("Limited Yaw Setpoint", self.limitedYawSetpoint)
        self.publishFloat("Manual Yaw Velocity", self.yawVelocity)
        self.publishFloat("Manual Pitch Velocity", self.pitchVelocity)
        self.publishFloat("Mode", self.mode.value)
        self.publishFloat("Target", self.target.value)
        self.publishStruct("target Position", self.targetPos)
        self.publishBoolean("Target Locked", self.targetLocked)
        self.publishBoolean(
            "Yaw forward soft limit",
            self.yawMotor._ctrlr.getForwardSoftLimit().isReached(),
        )
        self.publishBoolean(
            "Yaw reverse soft limit",
            self.yawMotor._ctrlr.getReverseSoftLimit().isReached(),
        )
        self.publishFloat(
            "(Radians) Yaw Motor Actual Setpoint",
            self.relativeYawSetpoint * YAW_GEARING,
        )
        self.publishFloat(
            "(Rotations) Yaw Motor Actual Setpoint",
            self.relativeYawSetpoint * YAW_GEARING / TAU,
        )
        self.publishFloat("pitch setpoint", self.pitchSetpoint)
        self.publishFloat("Relative pitch setpoint", self.relativePitchSetpoint)
        self.publishFloat("pitch angle", self.pitchAngle)
        self.publishFloat(
            "Pitch motor pos", rotationsToRadians(self.pitchEncoder.getPosition())
        )


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
        wrappedYaw = wrapAngle(yawAngle)

        robotRelativeX = math.cos(wrappedRoboYaw + PI / 2) * TURRET_DIST_FROM_CENTER
        robotRelativeY = math.sin(wrappedRoboYaw + PI / 2) * TURRET_DIST_FROM_CENTER

        self.robotRelativePos = Translation2d(robotRelativeX, robotRelativeY)
        feildPos: Translation2d = roboPose.translation().__add__(self.robotRelativePos)

        self.pose = Pose2d(feildPos, Rotation2d(wrappedRoboYaw - wrappedYaw))
        self.pitch = pitchAngle


class Shooter(Subsystem):

    def __init__(self, kickerId, revTopId, revBottomId):
        super().__init__()

        self.kickMotor = RevMotor(deviceID=kickerId)
        self.revingMotorTop: RevMotor = RevMotor(deviceID=revTopId)
        self.revingMotorBottom: RevMotor = RevMotor(deviceID=revBottomId)

        self.kickMotor.configure(config=RevMotor.KICK_CONFIG)
        self.revingMotorBottom.configure(config=RevMotor.FLYWHEEL_CONFIG)
        self.revingMotorTop.configure(config=RevMotor.FLYWHEEL_CONFIG)

        self.kickMotorEncoder = self.kickMotor.getEncoder()
        self.revTopEncoder = self.revingMotorTop.getEncoder()
        self.revBottomEncoder = self.revingMotorBottom.getEncoder()

        self.revingSetpoint: RPM = 0
        self.revingSpeedTop: RPM = 0
        self.revingSpeedBottom: RPM = 0
        self.kickSpeed: RPM = self.kickMotorEncoder.getPosition()  # read value

        self.mode: TurretMode = TurretMode.MANUAL

        self.fullyReved: bool = False

        self.dependencies: tuple = (None,)

        self.kickSetPoint = 0
        self.kickShooter: bool = False

        # self.kickMotor.configure(config=RevMotor.KICK_CONFIG)
        # self.revingMotorBottom.configure(config=RevMotor.FLYWHEEL_CONFIG)
        # self.revingMotorTop.configure(config=RevMotor.FLYWHEEL_CONFIG)

        self.dontShoot = False

        self.publishFloat("shooter manual", self.mode.value)

    def phaseInit(self, robotState: RobotState) -> RobotState:

        self.dependencies: tuple = (None,)

        self.kickSetPoint = 0
        self.kickShooter: bool = False

        self.dontShoot = False

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:

        self.mode = robotState.turretMode

        self.fullyReved = self.getFullyReved()

        if not self.fullyReved:
            robotState.dontShoot = True

        self.kickSpeed: RPM = self.kickMotorEncoder.getPosition()

        self.dependencies = (
            robotState.revSpeed,
            robotState.kickShooter,
            # robotState.optimalTurretAngle,
        )

        self.kickShooter = robotState.kickShooter

        if not checkDependencies(self.dependencies):
            return robotState

        if self.mode == TurretMode.MANUAL:
            self.manualUpdate(robotState)

        elif self.mode == TurretMode.DYNAMIC:
            self.dynamicUpdate(robotState)

        else:
            return robotState

        self.revingSetpoint *= robotState.revSpeed  # multiplied by the trigger value

        self.revShooters(self.revingSetpoint)

        self.kickSetPoint = KICK_SPEED * int(robotState.kickShooter)

        self.dontShoot = robotState.dontShoot

        # if not self.dontShoot:
        self.kickMotor.setVoltage(RPMToVolts(self.kickSetPoint, MAX_RPM))

        return robotState

    def manualUpdate(self, robotState: RobotState):

        self.revingSetpoint = MANUAL_REV_SPEED

    def dynamicUpdate(self, robotState: RobotState) -> None:

        mpsSetpoint: MPS = _calculateVelocity(
            robotState.optimalTurretAngle,
            robotState.targetDistance,
            robotState.targetHeight,
        )

        mpsSetpoint *= VELOCITY_SCALAR
        # TODO add a constant scale value to the speed
        self.revingSetpoint: RPM = MPSToRPM(mpsSetpoint, FLYWHEEL_CIRCUMFRENCE)

    def revShooters(self, speed: RPM):
        self.revingMotorBottom.setMaxMotionVelocity(speed)
        self.revingMotorTop.setMaxMotionVelocity(speed)

    def getFullyReved(self) -> bool:

        if self.revingSetpoint == 0:
            return True

        if (
            100
            - (
                abs(self.revingSetpoint - self.revingSpeedTop)
                / self.revingSetpoint
                * 100
            )
            > REV_ALLOWED_ERROR
        ):
            return False

        return True

    def disabled(self) -> None:
        self.kickMotor.setVoltage(0)
        self.revShooters(0)
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
        self.publishBoolean("Fully Reved", self.fullyReved)
        self.publishBoolean("kick shooter spinning", self.kickShooter)
        self.publishFloat("kick setpoint", self.kickSetPoint)


def calculateAngle(d: meters, h: meters, xPass: meters, yPass: meters) -> radians:

    numerator = yPass * d**2 - xPass**2 * h
    denom = d * (xPass * d - xPass**2)

    return math.atan(numerator / denom)


def _calculateVelocity(
    turretAngle: radians, hubDistance: meters, height: meters
) -> MPS:

    velocityMps = math.sqrt(
        (GRAVITY * turretAngle**2)
        / (2 * math.cos(turretAngle) * (hubDistance * math.tan(turretAngle) - height))
    )
    return velocityMps / FLYWHEEL_CIRCUMFRENCE * 60


def calculateTime(velocity: MPS, distance: meters):
    return distance / velocity


def checkDependencies(depends: tuple) -> bool:

    for var in depends:
        if var is None:
            return False

    return True
