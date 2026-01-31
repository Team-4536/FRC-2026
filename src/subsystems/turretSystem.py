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
from wpimath.geometry import Pose3d
from ntcore import NetworkTableInstance


MAX_ROTATION = degreesToRadians(270)
TURRET_GAP = math.tau - MAX_ROTATION
GEARING = 12  # TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation


class Turret(Subsystem):
    # cancoder more like cantcoder
    # yaw is horizontal rotation
    # pitch is vertical

    def __init__(self):
        self.motorYaw = RevMotor(
            deviceID=13
        )  # get right ID, motor for turning horizontally

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

        self.pitchMotor = RevMotor(deviceID=17)

        self.motorYaw.configure(config=self.motorYaw.AZIMUTH_CONFIG)
        self.pitchMotor.configure(config=self.pitchMotor.AZIMUTH_CONFIG)

        self.yawEncoder = self.motorYaw.getEncoder()
        self.pitchEncoder = self.pitchMotor.getEncoder()

        self.yawPos = rotationsToRadians(self.yawEncoder.getPosition())

        self.homeSet: bool = False

        self.setPoint = 0  # in relation to the field

        self.odom = TurretOdometry()

        self.table = NetworkTableInstance.getDefault().getTable("telementry")

        self.turretAngle: radians = rotationsToRadians(self.pitchEncoder.getPosition())

        ballRadius: inches = 5.91 / 2
        yPassDiff: meters = inchesToMeters(15 + ballRadius)
        self.goalHeightFromTurret: meters = 1  # not entirly correct
        self.yPass: meters = self.goalHeightFromTurret + yPassDiff
        hubRadius: inches = 24.5 / 2
        self.xPassDiff: meters = inchesToMeters(hubRadius)

    def init(self):
        pass

    def periodic(self, rs: RobotState):

        if not self.homeSet:
            self.reset(rs.limitA)
            return

        rs.optimalTurretAngle = self.calculateAngle(rs.hubDistance)
        robotYaw = rs.pose.rotation().radians()
        self.yawPos = rotationsToRadians(self.yawEncoder.getPosition())
        self.odom.updateWithEncoder(robotYaw, self.yawEncoder)

        self.setPoint = rs.turretSetPoint

        self.targetPoint()  # need odom
        self.maintainSetpoint(robotYaw)  # these go last
        self.dontOverdoIt()

        self.motorYaw.setPosition(self.setPoint * GEARING)

    def disabled(self):
        self.motorYaw.setVelocity(0)

        self.motorYaw.configure(config=self.motorYaw.DISABLED_AZIMUTH_CONFIG)
        # do we need these .configure lines when revmotor allready does this?

        self.homeSet = False

    def publish(self):

        self.table.putBoolean("Turret Home Set", self.homeSet)
        self.table.putNumber("Turret Yaw Setpoint", self.setPoint)
        self.table.putNumber(
            "Turret Yaw Actual Motor Setpoint", self.setPoint * GEARING
        )
        self.table.putNumber(
            "Turret Yaw Feild Relative Rotation", self.odom.feildRelativeRot
        )
        self.table.putNumber(
            "Turret Yaw Feild Relative Rotation Wrapped", self.odom.wrappedFRR
        )
        self.table.putNumber("Turret Yaw Encoder Motor Pos", self.yawPos)
        self.table.putNumber("Turret Yaw Actual Encoder Pos", self.yawPos / GEARING)

    def maintainSetpoint(self, robotYaw):
        self.setPoint -= self.odom.getGyroChange(robotYaw)

    def dontOverdoIt(self):
        if self.setPoint > MAX_ROTATION + TURRET_GAP / 2:
            self.setPoint = 0
        elif self.setPoint > MAX_ROTATION:
            self.setPoint = MAX_ROTATION

    def targetPoint(
        self, pointPose: Pose3d, robotPose: Pose3d
    ) -> None:  # make velocity later

        xDiff = pointPose.X() - robotPose.X()
        yDiff = pointPose.Y() - robotPose.Y()
        self.setPoint = math.atan(xDiff / yDiff)

    def reset(self, limit):
        if not self.homeSet:

            self.motorYaw.setVelocity(-1)

            if limit:
                self.yawEncoder.setPosition(0)
                self.homeSet = True

    def calculateAngle(self, d: meters) -> radians:
        xPass = d - self.xPassDiff
        h = self.goalHeightFromTurret

        numerator = self.yPass * d**2 - xPass**2 * h
        denom = d * (xPass * d - xPass**2)

        return math.atan(numerator / denom)


class TurretOdometry:
    def __init__(self):
        self.feildRelativeRot = 0
        self.wrappedFRR = 0  # wrapped field relative rotation
        self.pos: Pose3d = Pose3d()

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
        self.pitchMotor = RevMotor(deviceID=17)
        self.kickMotorEncoder = self.kickMotor.getEncoder()

        self.revingSpeed: RPM = 0
        self.shooterSpeed: RPM = self.kickMotorEncoder.getPosition()

        self.revingMotorTop = RevMotor(deviceID=11)
        self.revingMotorBottom = RevMotor(deviceID=12)

        super().__init__()

    def init(self) -> None:
        pass

    def periodic(self, rs: RobotState) -> None:

        self.revingMotorTop.setVelocity(self.revingSpeed)
        self.revingMotorBottom.setVelocity(self.revingSpeed)

        if rs.revShooter > 0.1:
            self.revingSpeed = self._calculateVelocity(
                rs.optimalTurretAngle, rs.hubDistance
            )
        elif rs.shootShooter == 1:
            pass  # feed stuff
        else:
            self.disabled()

    def _calculateVelocity(self, turretAngle, hubDistance) -> float:

        self.velocityMps = math.sqrt(
            (self.grav * self.turretAngle**2)
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
        self.pitchMotor.setVelocity(0)
        self.pitchMotor.configure(config=self.pitchMotor.DISABLED_AZIMUTH_CONFIG)

    def publish(self):
        self.table.putNumber("revShooter", self.revingSpeed)
        self.table.putNumber("shootShooter", self.shooterSpeed)
