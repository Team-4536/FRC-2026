from math import atan, cos, pi as PI, sqrt, tan, tau as TAU
from rev import SparkRelativeEncoder
from subsystems.motor import RevMotor
import numpy as np
from subsystems.robotState import (
    RobotState,
    TurretMode,
    TurretTarget,
)
from subsystems.subsystem import Subsystem
from subsystems.utils import (
    FIELD_LEN,
    FIELD_WIDTH,
    getTangentAngle,
    MPSToRPM,
    RPMToMPS,
    RPMToVolts,
    scaleTranslation2D,
    wrapAngle,
)
from typing import Any, Tuple
from wpilib import DriverStation, getTime, FieldObject2d
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d
from wpimath.units import (
    degreesToRadians,
    inches,
    inchesToMeters,
    meters_per_second as MPS,
    meters,
    radians,
    revolutions_per_minute as RPM,
    rotationsToRadians,
    seconds,
)

ROBOT_RADIUS = inchesToMeters(Translation2d(11, 11).norm())

MAX_PITCH: radians = degreesToRadians(90)  # in relation to feild
MIN_PITCH: radians = degreesToRadians(40)
MAX_ROTATION: radians = PI
TURRET_GAP: radians = TAU - MAX_ROTATION
# TODO offset in radians from the zero of the gyro and zero of the turret
ZERO_OFFSET: radians = (MAX_ROTATION - PI) / 2 + (PI / 2)
YAW_GEARING: float = 100 / 3
PITCH_RADIUS: inches = 8
LIL_PITCH_GEAR_RADIUS: inches = 3 / 4
ARC_RATIO = (
    PITCH_RADIUS / LIL_PITCH_GEAR_RADIUS
)  # how many rotations of the smol ladder gear is 1 rotation of the pitch
PITCH_GEARING: float = 16 * ARC_RATIO  # two 4:1 gearboxes make 25
TURRET_HEIGHT: meters = inchesToMeters(15)

BALL_RADIUS: inches = 5.91 / 2

MAX_RPM: RPM = 5676
HUB_RADIUS: inches = 41.7 / 2
HUB_DIST_X: meters = inchesToMeters(158.6) + inchesToMeters(
    HUB_RADIUS
)  # + inchesToMeters(10)
HUB_DIST_Y: meters = FIELD_WIDTH / 2
HUB_HEIGHT_Z: meters = inchesToMeters(73 - 15)
Y_PASS_DIFF_HUB: meters = inchesToMeters(17 + BALL_RADIUS)
Y_PASS_HUB: meters = HUB_HEIGHT_Z + Y_PASS_DIFF_HUB
X_PASS_DIFF_HUB: meters = inchesToMeters(HUB_RADIUS)

SHUTTLE_DIST_X: meters = 3
RIGHT_SHUTTLE_DIST_Y: meters = 3
LEFT_SHUTTLE_DIST_Y: meters = FIELD_WIDTH - 3

# x and y should be the same as what the robot thinks those are, z is height (in meters)
RED_TOP_SHUTTLE_POS: Translation3d = Translation3d(
    FIELD_LEN - SHUTTLE_DIST_X, FIELD_WIDTH - RIGHT_SHUTTLE_DIST_Y, 0
)
RED_BOTTOM_SHUTTLE_POS: Translation3d = Translation3d(
    FIELD_LEN - SHUTTLE_DIST_X, FIELD_WIDTH - LEFT_SHUTTLE_DIST_Y, 0
)
RED_SCORE_POS: Translation3d = Translation3d(
    FIELD_LEN - HUB_DIST_X,
    HUB_DIST_Y,
    HUB_HEIGHT_Z - TURRET_HEIGHT,
)
BLUE_TOP_SHUTTLE_POS: Translation3d = Translation3d(
    SHUTTLE_DIST_X, RIGHT_SHUTTLE_DIST_Y, 0
)
BLUE_BOTTOM_SHUTTLE_POS: Translation3d = Translation3d(
    SHUTTLE_DIST_X, LEFT_SHUTTLE_DIST_Y, 0
)
BLUE_SCORE_POS: Translation3d = Translation3d(
    HUB_DIST_X,
    HUB_DIST_Y,
    HUB_HEIGHT_Z - TURRET_HEIGHT,
)

BOTTOM_FLYWHEEL_DIAMETER: inches = 3.5  # diameter of the wheels build decides to use
TOP_FLYWHEEL_DIAMETER: inches = 3
BOTTOM_FLYWHEEL_CIRCUMFRENCE: meters = inchesToMeters(BOTTOM_FLYWHEEL_DIAMETER) * PI
TOP_FLYWHEEL_CIRCUMFRENCE: meters = inchesToMeters(TOP_FLYWHEEL_DIAMETER) * PI
GRAVITY: MPS = 9.80665  # don't worry that it's positive
MANUAL_REV_SPEED: MPS = 13.96
MANUAL_SPEED: RPM = 50
KICK_SPEED: RPM = 3500

# in percent
REV_ALLOWED_ERROR: float = 3
# in radians
YAW_ALLOWED_ERROR: radians = 0.1
PITCH_ALLOWED_ERROR: radians = 0.1


TURRET_DIST_FROM_CENTER: meters = inchesToMeters(27 - (6 + 1 / 2))  # TODO make correct
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

        self.homeSet: bool = True
        self.yawSetPoint: radians = 0  # in relation to the field
        self.limitedYawSetpoint: radians = 0
        self.relativeYawSetpoint: radians = 0  # in relation to the robot
        self.pitchSetpoint: radians = PI / 2
        self.relativePitchSetpoint: radians = 0  # cuz zero points up
        self.targetPos: Translation3d = Translation3d()

        self.targetLocked: bool = False

        self.target: TurretTarget = TurretTarget.HUB
        self.mode: TurretMode = TurretMode.DYNAMIC

        self.turretManDependencies: Tuple[Any, ...] = (None,)
        self.turretGenDepedencies: Tuple[Any, ...] = (None,)
        self.turretGenDepedencies: Tuple[Any, ...] = (None,)
        # these velocity values are only used when in DYNAMIC mode
        self.yawVelocity: RPM = 0
        self.pitchVelocity: RPM = 0

        self.impossibleDynamic = False
        self.compensateFail = False
        self.dynamicFail = False

        self.lastTime: seconds = getTime()

        self.side: DriverStation.Alliance = (
            DriverStation.getAlliance() or DriverStation.Alliance.kRed
        )

        self.velocityVar = 0.0
        self.pitchVar = 0.0

        self.publishFloat("YawTargetOffset", 36)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self.fieldTargPos: FieldObject2d = robotState.odomField.getObject(
            "fieldTargPos"
        )
        self.passThrough: FieldObject2d = robotState.odomField.getObject("passThrough")

        self.homeSet: bool = True
        self.yawSetPoint: radians = 0  # in relation to the field
        self.limitedYawSetpoint: radians = 0
        self.relativeYawSetpoint: radians = 0  # in relation to the robot
        self.pitchSetpoint: radians = PI / 2
        self.targetPos: Translation3d = Translation3d()

        self.targetLocked: bool = False

        self.target: TurretTarget = TurretTarget.HUB
        self.mode: TurretMode = TurretMode.DYNAMIC

        self.turretAutoDepedencies = (None,)
        self.turretManDependencies = (None,)
        self.turretGenDepedencies = (None,)

        # these velocity values are only used when in manual mode
        self.yawVelocity: RPM = 0
        self.pitchVelocity: RPM = 0

        self.impossibleDynamic = False
        self.compensateFail = False
        self.dynamicFail = False

        self.lastTime: seconds = getTime()

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        self.compensateFail = False
        self.dynamicFail = False
        self.impossibleDynamic = False
        robotState.impossibleDynamic = False
        robotState.dontShoot = False
        self.mode = self.getMode(robotState)

        self.fieldTargPos.setPose(
            Pose2d(Translation2d(self.targetPos.x, self.targetPos.y), Rotation2d())
        )

        self.yawEncoderPos = rotationsToRadians(self.yawEncoder.getPosition())
        self.targetLocked = self.getTargetLocked()

        if not self.targetLocked:
            robotState.dontShoot = True

        self.turretManDependencies = (robotState.turretManualSetpoint,)
        self.turretGenDepedencies = (robotState.odometry,)

        if not self.homeSet:
            self.reset(self.yawMotor._ctrlr.getReverseLimitSwitch().get())
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
            self.targetPos = self.getTargetPos(
                self.target, DriverStation.getAlliance() or DriverStation.Alliance.kRed
            )
            self.automaticUpdate(robotState)

        elif self.mode == TurretMode.DISABLED:
            self.reset(self.yawMotor._ctrlr.getReverseLimitSwitch().get())

        robotState.turretMode = self.mode

        self.velocityVar = self.getFloat("velocity variable", default=0)
        self.ptchVar = self.getFloat("ptch variable", default=0)

        return robotState

    def automaticUpdate(self, robotState: RobotState):

        h = self.targetPos.z
        d = self.getTargetDist(self.targetPos, self.odom.pose)

        angle = 0
        velocity = 0
        time = 0

        self.publishFloat("d", d, debug=True)
        self.publishFloat("h", h, debug=True)
        self.publishFloat("xPass", self.getXPass(d), debug=True)
        self.publishFloat("ypass", self.getYPass(), debug=True)

        try:
            angle = calculateAngle(d, h, self.getXPass(d), self.getYPass())
            velocity = _calculateVelocity(angle, d, h)
            self.publishFloat("velocity", velocity)
            time = calculateTime(velocity, d)
            self.publishFloat("time", time, debug=True)

        except:
            self.dynamicFail = True
            self.impossibleDynamic = True
            robotState.impossibleDynamic = True
            robotState.dontShoot = True
            return
        self.publishFloatArray(
            "Robot Linear veloity",
            (
                robotState.robotLinearVelocity.norm(),
                (
                    0
                    if robotState.robotLinearVelocity.norm() == 0
                    else robotState.robotLinearVelocity.angle().radians()
                ),
            ),
            debug=True,
        )

        self.compensateSetpoint(
            time, robotState.robotLinearVelocity, robotState.robotOmegaSpeed
        )
        self.passThrough.setPose(self.odom.pose)
        try:
            self.targetPoint(
                self.targetPos, self.odom.pose, robotState
            )  # need robot odom
        except:
            self.compensateFail = True
            self.impossibleDynamic = True
            robotState.impossibleDynamic = True
            robotState.dontShoot = True
            return

        self.relativePitchSetpoint = self.getRelativePitchSetpoint(self.pitchSetpoint)

        self.yawMotor.setPosition(self.limitedYawSetpoint * YAW_GEARING)
        self.pitchMotor.setPosition(self.relativePitchSetpoint * PITCH_GEARING)

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
                self.pitchVelocity = -0.8

            elif setPoint > 90 and setPoint < 270:
                self.pitchVelocity = 0.8

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

    def getTargetDist(self, targetPos: Translation3d, pose: Pose2d) -> meters:
        dist: meters = Translation2d(targetPos.x, targetPos.y).distance(
            Translation2d(pose.x, pose.y)
        )
        return dist

    def getMode(self, rs: RobotState) -> TurretMode:
        mode = self.mode
        if rs.turretSwitchEnabled:
            mode = (
                TurretMode.DISABLED
                if not (self.mode == TurretMode.DISABLED)
                else self.mode
            )

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
            if target == TurretTarget.SHUTTLE_BOTTOM:
                target = TurretTarget.HUB
            else:
                target = TurretTarget(target.value + 1)

        return target

    def getTargetPos(
        self, target: TurretTarget, side: DriverStation.Alliance
    ) -> Translation3d:

        pos: Translation3d = self.targetPos
        self.publishString("TEAM side", side.name, debug=True)

        if side == DriverStation.Alliance.kBlue:
            match target:
                case TurretTarget.HUB:
                    return BLUE_SCORE_POS

                case TurretTarget.SHUTTLE_TOP:
                    return BLUE_TOP_SHUTTLE_POS

                case TurretTarget.SHUTTLE_BOTTOM:
                    return BLUE_BOTTOM_SHUTTLE_POS

                case _:
                    return pos
        elif side == DriverStation.Alliance.kRed:
            match target:
                case TurretTarget.HUB:
                    return RED_SCORE_POS

                case TurretTarget.SHUTTLE_TOP:
                    return RED_TOP_SHUTTLE_POS

                case TurretTarget.SHUTTLE_BOTTOM:
                    return RED_BOTTOM_SHUTTLE_POS

                case _:
                    return pos

        return Translation3d()

    def getTargetLocked(self) -> bool:

        if (
            abs(self.relativeYawSetpoint - self.yawEncoderPos / YAW_GEARING)
            > YAW_ALLOWED_ERROR
        ):
            return False

        if abs(self.pitchSetpoint - self.pitchAngle) > PITCH_ALLOWED_ERROR:
            return False

        return True

    def compensateSetpoint(
        self,
        time: float,
        roboLinV: Translation2d,
        roboOmegaSpeed: MPS,
    ):
        compensateVector: Translation2d = Translation2d()

        offset: Translation2d = self.odom.posFromRobot
        turretRotSpeed: MPS = roboOmegaSpeed * (TURRET_DIST_FROM_CENTER / ROBOT_RADIUS)
        tanVel: Translation2d = Translation2d(
            distance=turretRotSpeed, angle=Rotation2d(getTangentAngle(offset))
        )

        self.publishFloatArray(
            "Tangent velocity",
            (tanVel.norm(), tanVel.angle().radians()),
        )
        # add the mps values
        compensateVector += tanVel
        compensateVector += roboLinV

        # multiply by time to get the distance the ball would move

        compensateVector = scaleTranslation2D(compensateVector, time)
        compensateVector.rotateBy(Rotation2d(PI))

        # add in the opposite direction

        self.targetPos += Translation3d(compensateVector.x, compensateVector.y, 0)

    def dontOverdoItYaw(self, angle: radians) -> radians:
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
        self, pointPos: Translation3d, turretPose: Pose2d, robotState: RobotState
    ) -> None:

        xDiff = pointPos.X() - turretPose.X()
        # super high tech offset
        offset = self.getFloat("YawTargetOffset", default=36)
        xDiff += np.sign(xDiff) * inchesToMeters(offset)
        yDiff = pointPos.Y() - turretPose.Y()

        quadrant = 1
        # inverse tan only returns in quadrant 1 and 4
        if xDiff < 0 and yDiff > 0:
            quadrant = 2

        elif xDiff < 0 and yDiff < 0:
            quadrant = 3

        # we don't care if its 4

        # gets the yaw angle
        self.yawSetPoint = atan(yDiff / xDiff)

        # In case inverse tan won't output correct angle
        if quadrant == 2 or quadrant == 3:
            self.yawSetPoint += PI

        self.yawSetPoint = wrapAngle(self.yawSetPoint)

        self.relativeYawSetpoint = (
            self.yawSetPoint
            - wrapAngle(robotState.odometry.getEstimatedPosition().rotation().radians())
            + ZERO_OFFSET
        )

        self.relativeYawSetpoint = wrapAngle(self.relativeYawSetpoint)

        self.limitedYawSetpoint = self.dontOverdoItYaw(self.relativeYawSetpoint)

        d = self.getTargetDist(pointPos, turretPose)
        h = pointPos.z

        self.pitchSetpoint = calculateAngle(d, h, self.getXPass(d), self.getYPass())

        velocity: MPS = _calculateVelocity(self.pitchSetpoint, d, h)

        # scale x and y independently

        self.pitchSetpoint = self.dontOverDoItPitch(self.pitchSetpoint)

        robotState.turretVelocitySetpoint = Translation2d(
            distance=compensateSpeed(velocity),
            angle=Rotation2d(self.pitchSetpoint),
        )

    def getXPass(self, d: meters) -> meters:
        xPass = d - X_PASS_DIFF_HUB  # for hub

        return xPass

    def getYPass(self) -> meters:
        yPass = Y_PASS_HUB

        return yPass

    def getRelativePitchSetpoint(self, angle: radians) -> radians:
        return (PI / 2) - angle

    def reset(self, limit: bool):

        if limit:
            self.homeSet = True

        else:
            self.yawMotor.setVoltage(-2)  # changed to -2 from -1

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
        self.publishString("Mode", self.mode.name)
        self.publishString("Target", self.target.name)
        self.publishStruct("target Position", self.targetPos)
        self.publishBoolean("Target Locked", self.targetLocked)
        self.publishBoolean(
            "Yaw forward soft limit",
            self.yawMotor._ctrlr.getForwardSoftLimit().isReached(),  # pyright: ignore
        )
        self.publishBoolean(
            "Yaw reverse hard limit",
            self.yawMotor._ctrlr.getReverseLimitSwitch().get(),  # pyright: ignore
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
        self.publishFloat("Pitch motor pos", self.pitchEncoder.getPosition())
        self.publishStruct("Turret field pose", self.odom.pose)
        self.publishBoolean("Impossible dynamic", self.impossibleDynamic)
        self.publishBoolean("Initial dynamic calculation failed", self.dynamicFail)
        self.publishBoolean(
            "Failed to dynamic after setpoint was compensated", self.compensateFail
        )
        self.publishFloat("velocity variable", self.velocityVar)
        self.publishFloat("pitch variable", self.pitchVar)


class TurretOdometry:

    def __init__(self):
        self.pitch: radians = 0
        self.posFromRobot: Translation2d = Translation2d()
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
        pitchAngle = PI / 2 - (rotationsToRadians(pitchRaw) / PITCH_GEARING)

        wrappedRoboYaw = wrapAngle(roboPose.rotation().radians())
        wrappedYaw = wrapAngle(yawAngle)

        self.posFromRobot = Translation2d(
            distance=TURRET_DIST_FROM_CENTER, angle=Rotation2d(wrappedRoboYaw)
        )

        feildPos: Translation2d = roboPose.translation() + self.posFromRobot

        self.pose = Pose2d(feildPos, Rotation2d(wrappedRoboYaw + wrappedYaw))
        self.pitch = pitchAngle


class Shooter(Subsystem):
    def __init__(self, kickerID: int, revTopID: int, revBottomID: int):
        super().__init__()

        self.kickMotor = RevMotor(deviceID=kickerID)
        self.revingMotorTop: RevMotor = RevMotor(deviceID=revTopID)
        self.revingMotorBottom: RevMotor = RevMotor(deviceID=revBottomID)

        self.kickMotor.configure(config=RevMotor.KICK_CONFIG)
        self.revingMotorBottom.configure(config=RevMotor.FLYWHEEL_CONFIG)
        self.revingMotorTop.configure(config=RevMotor.FLYWHEEL_CONFIG)

        self.kickMotorEncoder = self.kickMotor.getEncoder()
        self.revTopEncoder = self.revingMotorTop.getEncoder()
        self.revBottomEncoder = self.revingMotorBottom.getEncoder()

        self.revingSetpoint: RPM = 0
        self.mpsSetpoint: MPS = 0
        self.revingSpeedTop: RPM = 0
        self.revingSpeedBottom: RPM = 0
        self.kickSpeed: RPM = self.kickMotorEncoder.getPosition()  # read value

        self.mode: TurretMode = TurretMode.MANUAL

        self.fullyReved: bool = False

        self.dependencies: Tuple[Any, ...] = (None,)

        self.kickSetPoint = 0
        self.kickShooter: int = False

        self.dontShoot = False

        self.publishFloat("shooter manual", self.mode.value)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self.dependencies = (None,)

        self.kickSetPoint = 0
        self.kickShooter: int = False

        self.dontShoot = False

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        self.mode = robotState.turretMode

        self.fullyReved = self.getFullyReved()

        if not self.fullyReved:
            robotState.dontShoot = True

        self.kickSpeed = self.kickMotorEncoder.getVelocity()
        self.revingSpeedTop = self.revTopEncoder.getVelocity()
        self.revingSpeedBottom = self.revBottomEncoder.getVelocity()

        self.dependencies = (
            robotState.revSpeed,
            robotState.kickShooter,
            robotState.indexerEject,
            robotState.ejectAll,
            robotState.dontShoot,
        )

        self.kickShooter = robotState.kickShooter

        if not checkDependencies(self.dependencies):
            return robotState

        if self.mode == TurretMode.MANUAL:
            self.manualUpdate()

        elif self.mode == TurretMode.DYNAMIC:
            self.dynamicUpdate(robotState)

        else:
            return robotState

        self.mpsSetpoint *= robotState.revSpeed  # multiplied by the trigger value

        if robotState.ejectAll > 0.3 or robotState.indexerEject:
            robotState.kickShooter = -1

        if robotState.intakeIndexer:
            robotState.kickShooter = 1

        self.revShooters(self.mpsSetpoint)

        self.kickSetPoint = KICK_SPEED * robotState.kickShooter

        self.dontShoot = robotState.dontShoot

        if not self.dontShoot:
            self.kickMotor.setVoltage(RPMToVolts(self.kickSetPoint, KICK_SPEED))

        return robotState

    def manualUpdate(self):
        self.mpsSetpoint = MANUAL_REV_SPEED

    def dynamicUpdate(self, robotState: RobotState) -> None:
        depend: Tuple[Any, ...] = (
            robotState.impossibleDynamic,
            robotState.turretVelocitySetpoint,
        )

        if not checkDependencies(depend):
            return

        if robotState.impossibleDynamic:
            return

        self.mpsSetpoint = robotState.turretVelocitySetpoint.norm()

    def revShooters(self, speed: MPS):
        top: RPM = MPSToRPM(speed, TOP_FLYWHEEL_CIRCUMFRENCE)
        bottom: RPM = MPSToRPM(speed, BOTTOM_FLYWHEEL_CIRCUMFRENCE)
        self.revingMotorBottom.setVelocity(bottom)
        self.revingMotorTop.setVelocity(top)

    def getFullyReved(self) -> bool:

        if self.revingSetpoint == 0:
            return True

        if 100 - (100 * self.getMPSSpeed() / self.mpsSetpoint) > REV_ALLOWED_ERROR:
            return False

        return True

    def getRevSpeed(self) -> RPM:
        avg = (self.revingSpeedTop + self.revingSpeedBottom) / 2
        return avg

    def getMPSSpeed(self) -> MPS:
        avg = (
            RPMToMPS(self.revingSpeedTop, TOP_FLYWHEEL_CIRCUMFRENCE)
            + RPMToMPS(self.revingSpeedBottom, BOTTOM_FLYWHEEL_CIRCUMFRENCE)
        ) / 2
        return avg

    def disabled(self) -> None:
        self.kickMotor.setVoltage(0)
        self.revShooters(0)
        self.revingSetpoint = 0
        self.mpsSetpoint = 0

    def publish(self):
        self.publishFloat("revMotor set speed", self.revingSetpoint)
        self.publishFloat("Kick motor encoder rpm", self.kickSpeed)
        self.publishFloat("top reving motor encoder rpm", self.revingSpeedTop)
        self.publishFloat("bottom reving motor encoder rpm", self.revingSpeedBottom)
        self.publishBoolean("Fully Reved", self.fullyReved)
        self.publishFloat("kick shooter spinning", self.kickShooter)
        self.publishFloat("kick setpoint", self.kickSetPoint)
        self.publishFloat("(MPS) reving read speed", self.getMPSSpeed())
        self.publishFloat("avg rev speed", self.getRevSpeed())
        self.publishFloat("MPS_setpoint", self.mpsSetpoint)


def calculateAngle(d: meters, h: meters, xPass: meters, yPass: meters) -> radians:
    numerator = yPass * d**2 - xPass**2 * h
    denom = d * (xPass * d - xPass**2)

    return atan(numerator / denom)


def _calculateVelocity(turretAngle: radians, distance: meters, height: meters) -> MPS:
    numer = GRAVITY * (distance**2)
    denom = 2 * (cos(turretAngle) ** 2) * (distance * tan(turretAngle) - height)
    velocityMps = sqrt(numer / denom)
    return velocityMps


def calculateTime(velocity: MPS, distance: meters):
    return distance / velocity


def checkDependencies(depends: Tuple[Any, ...]) -> bool:
    for var in depends:
        if var is None:
            return False

    return True


def compensateSpeed(speed: MPS) -> MPS:
    # desmos best fit line :D
    actual: MPS = 2.55728 * (speed) - 4.04025
    return actual
