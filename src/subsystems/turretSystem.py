from subsystems.motor import RevMotor
from subsystems.subsystem import Subsystem
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
from wpimath.geometry import Pose2d, Translation3d
from ntcore import NetworkTableInstance


MAX_ROTATION = degreesToRadians(270)
TURRET_GAP = math.tau - MAX_ROTATION
GEARING = 12  # TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation

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


class Turret(Subsystem):
    # cancoder more like cantcoder
    # yaw is horizontal rotation
    # pitch is vertical

    def __init__(self, yawMotorID, pitchMotorID):
        self.motorYaw = RevMotor(
            deviceID=yawMotorID
        )  # get right ID, motor for turning horizontally

        self.pitchMotor = RevMotor(deviceID=pitchMotorID)

        self.motorYaw.AZIMUTH_CONFIG = (
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

        self.motorYaw.configure(config=self.motorYaw.AZIMUTH_CONFIG)
        self.pitchMotor.configure(config=self.pitchMotor.AZIMUTH_CONFIG)

        self.yawEncoder = self.motorYaw.getEncoder()
        self.pitchEncoder = self.pitchMotor.getEncoder()

        self.yawPos = rotationsToRadians(self.yawEncoder.getPosition())

        self.odom = TurretOdometry()

        self.table = NetworkTableInstance.getDefault().getTable("telementry")

        self.turretAngle: radians = rotationsToRadians(self.pitchEncoder.getPosition())

        self.homeSet: bool = False
        self.yawSetPoint = 0  # in relation to the field
        self.target: Translation3d = Translation3d()

    def init(self):
        self.homeSet: bool = False
        self.yawSetPoint = 0  # in relation to the field

    def periodic(self, robotState: RobotState) -> RobotState:

        if not self.homeSet:
            self.reset(robotState.limitA)
            return robotState

        robotState.optimalTurretAngle = self.calculateAngle(robotState.hubDistance)
        robotYaw = robotState.pose.rotation().radians()
        self.yawPos = rotationsToRadians(self.yawEncoder.getPosition())
        self.odom.updateWithEncoder(robotYaw, self.yawEncoder)

        self.targetPoint(self.target, robotState.pose)  # need odom
        self.maintainRotation(robotYaw)  # these go last
        self.dontOverdoIt()

        self.motorYaw.setPosition(self.yawSetPoint * GEARING)

        return robotState

    def disabled(self):
        self.motorYaw.setVelocity(0)
        self.pitchMotor.setVelocity(0)

        self.motorYaw.configure(config=self.motorYaw.DISABLED_AZIMUTH_CONFIG)
        self.pitchMotor.configure(config=self.pitchMotor.DISABLED_AZIMUTH_CONFIG)
        # do we need these .configure lines when revmotor allready does this?

        self.homeSet = False

    def publish(self):

        self.table.putBoolean("Turret Home Set", self.homeSet)
        self.table.putNumber("Turret Yaw Setpoint", self.yawSetPoint)
        self.table.putNumber(
            "Turret Yaw Actual Motor Setpoint", self.yawSetPoint * GEARING
        )
        self.table.putNumber(
            "Turret Yaw Feild Relative Rotation", self.odom.feildRelativeRot
        )
        self.table.putNumber(
            "Turret Yaw Feild Relative Rotation Wrapped", self.odom.wrappedFRR
        )
        self.table.putNumber("Turret Yaw Encoder Motor Pos", self.yawPos)
        self.table.putNumber("Turret Yaw Actual Encoder Pos", self.yawPos / GEARING)

    def maintainRotation(self, robotYaw):
        self.yawSetPoint -= self.odom.getGyroChange(robotYaw)

    def dontOverdoIt(self):
        if self.yawSetPoint > MAX_ROTATION + TURRET_GAP / 2:
            self.yawSetPoint = 0
        elif self.yawSetPoint > MAX_ROTATION:
            self.yawSetPoint = MAX_ROTATION

    def targetPoint(
        self, pointPose: Translation3d, robotPose: Pose2d
    ) -> None:  # make velocity later

        xDiff = pointPose.X() - robotPose.X()
        yDiff = pointPose.Y() - robotPose.Y()
        self.yawSetPoint = math.atan(xDiff / yDiff)

    def reset(self, limit):
        if not self.homeSet:

            self.motorYaw.setVelocity(-1)

            if limit:
                self.yawEncoder.setPosition(0)
                self.homeSet = True

    def calculateAngle(self, d: meters) -> radians:
        xPass = d - X_PASS_DIFF
        h = GOAL_HEIGHT

        numerator = Y_PASS * d**2 - xPass**2 * h
        denom = d * (xPass * d - xPass**2)

        return math.atan(numerator / denom)


class TurretOdometry:
    def __init__(self):
        self.feildRelativeRot = 0
        self.wrappedFRR = 0  # wrapped field relative rotation
        self.pos: Pose2d = Pose2d()

    def updateWithEncoder(self, roboYaw, encoder: SparkRelativeEncoder):

        turretRaw = encoder.getPosition()
        turretRot = rotationsToRadians(turretRaw) / GEARING
        self.wrappedFRR = (turretRot % TAU) * np.sign(
            turretRot
        )  # modulo only returns positive so multiply by the sign
        self.feildRelativeRot = roboYaw - turretRot

    def getGyroChange(self, yaw):
        wrappedRoboYaw = (yaw % TAU) * np.sign(yaw)

        return self.wrappedFRR - wrappedRoboYaw


class Shooter(Subsystem):

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.grav = 9.80665

        self.hubDistance = 0  # hypotenuse of odometry and the Hub position
        self.wheelDiam: inches = 3  # radius of the wheels build decides to use
        self.wheelCirc: meters = inchesToMeters(self.wheelDiam) * PI

        self.kickMotor = RevMotor(deviceID=14)
        self.kickMotorEncoder = self.kickMotor.getEncoder()

        self.revingSpeed: RPM = 0
        self.shooterSpeed: RPM = self.kickMotorEncoder.getPosition()

        self.revingMotorTop = RevMotor(deviceID=11)
        self.revingMotorBottom = RevMotor(deviceID=12)

        super().__init__()

    def init(self) -> None:
        pass

    def periodic(self, robotState: RobotState) -> RobotState:

        self.revingMotorTop.setVelocity(self.revingSpeed)
        self.revingMotorBottom.setVelocity(self.revingSpeed)

        if robotState.revShooter > 0.1:
            self.revingSpeed = self._calculateVelocity(
                robotState.optimalTurretAngle, robotState.hubDistance
            )
        elif robotState.shootShooter == 1:
            pass  # feed stuff
        else:
            self.disabled()

        return robotState

    def _calculateVelocity(self, turretAngle, hubDistance) -> float:

        self.velocityMps = math.sqrt(
            (self.grav * turretAngle**2)
            / (
                2
                * math.cos(turretAngle)
                * (hubDistance * math.tan(turretAngle) - 0.9652)
            )
        )
        return self.velocityMps / self.wheelCirc * 60

    def disabled(self) -> None:
        self.kickMotor.setVelocity(0)
        self.revingSpeed = 0

    def publish(self):
        self.table.putNumber("revShooter", self.revingSpeed)
        self.table.putNumber("shootShooter", self.shooterSpeed)
