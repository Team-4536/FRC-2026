import math
from math import cos, pi as PI, sin, tau
from navx import AHRS
from phoenix6.hardware import CANcoder
from phoenix6.units import rotation
from rev import SparkBaseConfig
from subsystems.motor import RevMotor
from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import (
    RobotState,
    ROBOT_RADIUS,
)
from subsystems.utils import (
    getTangentalVelocity,
    getContributedRotation,
    RPMToMPS,
)
from subsystems.subsystem import Subsystem
from typing import NamedTuple, Tuple
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
)
from wpimath.units import (
    meters_per_second as MPS,
    rotationsPerMinuteToRadiansPerSecond,
    revolutions_per_minute as RPM,
    meters,
    radiansToRotations,
    feetToMeters,
    radians,
    rotationsToRadians,
    inchesToMeters,
)
from wpimath.estimator import SwerveDrive4PoseEstimator


class SwerveModule(NetworkTablesMixin):
    def __init__(
        self,
        *,
        driveMotorID: int,
        azimuthMotorID: int,
        azimuthEncoderID: int,
    ) -> None:
        super().__init__()

        self._driveMotor = RevMotor(deviceID=driveMotorID)
        self._azimuthMotor = RevMotor(deviceID=azimuthMotorID)

        self._driveEncoder = self._driveMotor.getEncoder()
        self._azimuthEncoder = self._azimuthMotor.getEncoder()
        self._absoluteEncoder = CANcoder(device_id=azimuthEncoderID)

        self._driveGearing = RevMotor.DRIVE_GEARiNG
        self._azimuthGearing = RevMotor.AZIMUTH_GEARING
        self._position = Translation2d(0, 0)

        wheelDiam: meters = 0.1016
        self._wheelCircumferance: meters = wheelDiam * PI

        self._driveEncoder.setPosition(0)
        self.resetAzimuthEncoder()

    @property
    def driveDistance(self) -> meters:
        return (
            self._driveEncoder.getPosition()
            * self._wheelCircumferance
            / self._driveGearing
        )

    @property
    def driveVelocity(self) -> MPS:
        motorRPS = self._driveEncoder.getVelocity() / 60
        wheelRPS = motorRPS / self._driveGearing
        wheelMPS = wheelRPS * self._wheelCircumferance

        return wheelMPS

    @property
    def azimuthRotation(self) -> rotation:
        return self._azimuthEncoder.getPosition() / self._azimuthGearing

    @property
    def absoluteAzimuthRotation(self) -> rotation:
        return (
            self._absoluteEncoder.get_absolute_position().value * self._azimuthGearing
        )

    @property
    def modulePosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            distance=self.driveDistance,
            angle=Rotation2d.fromRotations(self.azimuthRotation),
        )

    def configureDriveMotor(self, *, config: SparkBaseConfig) -> None:
        self._driveMotor.configure(config=config)

    def configureAzimuthMotor(self, *, config: SparkBaseConfig) -> None:
        self._azimuthMotor.configure(config=config)

    @property
    def position(self) -> Translation2d:
        return self._position

    def setPosition(self, x: float, y: float) -> None:
        self._position = Translation2d(x, y)

    def setDrive(self, velocity: MPS) -> None:
        wheelRPS = velocity / self._wheelCircumferance
        motorRPS = wheelRPS * self._driveGearing
        motorRPM = motorRPS * 60
        self._driveMotor.setVelocity(motorRPM)

    def setAzimuth(self, angle: Rotation2d) -> None:
        pos = self._azimuthGearing * angle.radians()
        self._azimuthMotor.setPosition(pos)

    def stopModule(self) -> None:
        self._driveMotor.stopMotor()
        self._azimuthMotor.stopMotor()

    def resetAzimuthEncoder(self) -> None:
        self._azimuthEncoder.setPosition(self.absoluteAzimuthRotation)


class SwerveModules(NamedTuple):
    frontLeft: SwerveModule
    frontRight: SwerveModule
    backLeft: SwerveModule
    backRight: SwerveModule

    def symmetricPosition(self, x: meters, y: meters) -> None:
        self.frontLeft.setPosition(x, y)
        self.frontRight.setPosition(x, -y)
        self.backLeft.setPosition(-x, y)
        self.backRight.setPosition(-x, -y)

    def stopModules(self) -> None:
        for m in self:
            m.stopModule()

    @property
    def positions(
        self,
    ) -> Tuple[Translation2d, Translation2d, Translation2d, Translation2d]:
        return tuple(m.position for m in self)  # type: ignore[return-value]

    @property
    def modulePositions(
        self,
    ) -> Tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return tuple(m.modulePosition for m in self)  # type: ignore[return-value]


class SwerveDrive(Subsystem):
    MAX_MODULE_SPEED: MPS = 5.15

    def __init__(self) -> None:
        super().__init__()

        WHEEL_DISTANCE: meters = inchesToMeters(10.875)

        self._modules = self._symmetricDrive(
            frontLeftDriveID=2,
            frontRightDriveID=4,
            backLeftDriveID=6,
            backRightDriveID=8,
            frontLeftAzimuthID=1,
            frontRightAzimuthID=3,
            backLeftAzimuthID=5,
            backRightAzimuthID=7,
            frontLeftEncoderID=21,
            frontRightEncoderID=22,
            backLeftEncoderID=23,
            backRightEncoderID=24,
            xPos=WHEEL_DISTANCE,
            yPos=WHEEL_DISTANCE,
        )

        self.initPos = Pose2d()
        self._kinematics = SwerveDrive4Kinematics(*self._modules.positions)
        self._gyro = AHRS(AHRS.NavXComType.kMXP_SPI)
        self._gyro.reset()

        self._swerveStates = self._kinematics.desaturateWheelSpeeds(
            self._kinematics.toSwerveModuleStates(ChassisSpeeds()), 0
        )

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self._configureDriveMotors(config=RevMotor.DRIVE_CONFIG)
        self._configureAzimuthMotors(config=RevMotor.AZIMUTH_CONFIG)

        for m in self._modules:
            m.resetAzimuthEncoder()

        return robotState

    def robotPeriodic(self, robotState: RobotState) -> RobotState:
        robotState.odometry.update(
            self._gyro.getRotation2d(),
            self._modules.modulePositions,
        )
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        if robotState.resetGyro:
            self._gyro.reset()
            robotState.odometry.resetPosition(
                self._gyro.getRotation2d(),
                self._modules.modulePositions,
                Pose2d(
                    robotState.odometry.getEstimatedPosition().translation(),
                    Rotation2d(),
                ),
            )

        self.drive(
            fieldSpeeds=robotState.fieldSpeeds,
            attainableMaxSpeed=self.MAX_MODULE_SPEED,
        )

        robotState.robotOmegaSpeed = self.getOmegaVelocity()
        robotState.robotLinearVelocity = self.getLinearVelocity()

        return robotState

    def disabled(self) -> None:
        self._modules.stopModules()
        self._configureDriveMotors(config=RevMotor.DISABLED_DRIVE_CONFIG)
        self._configureAzimuthMotors(config=RevMotor.DISABLED_AZIMUTH_CONFIG)

    def getLinearVelocity(self) -> Translation2d:
        vector = Translation2d()

        for module in self._modules:
            vector = vector.__add__(self.getDriveVelocity(module))

        return vector

    def getOmegaVelocity(self) -> MPS:

        sum = 0
        for module in self._modules:

            tanVel = getTangentalVelocity(
                module._position,
                self.getDriveVelocity(module).distance(Translation2d()),
            )

            sum += getContributedRotation(
                tanVel,
                rotationsToRadians(module._azimuthMotor.getEncoder().getPosition()),
            )

        return sum / 4 / ROBOT_RADIUS

    def getDriveVelocity(self, module: SwerveModule) -> Translation2d:
        rpm: RPM = module._driveMotor.getEncoder().getVelocity()
        speed = RPMToMPS(rpm, self._modules[0]._wheelCircumferance)
        angle: radians = rotationsToRadians(
            module._azimuthMotor.getEncoder().getVelocity()
        )
        vector = Translation2d(speed * math.cos(angle), speed * math.sin(angle))
        return vector

    def drive(self, fieldSpeeds: ChassisSpeeds, attainableMaxSpeed: MPS) -> None:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vx,
            fieldSpeeds.vy,
            fieldSpeeds.omega,
            self._gyro.getRotation2d(),
        )
        moduleStates = self._kinematics.toSwerveModuleStates(chassisSpeeds)

        self._swerveStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates=moduleStates,
            attainableMaxSpeed=attainableMaxSpeed,
        )

        for module, state in zip(self._modules, self._swerveStates):
            state.optimize(module.modulePosition.angle)
            module.setDrive(state.speed)
            module.setAzimuth(state.angle)

        # print(fieldSpeeds)

    def publish(self) -> None:
        self.publishSwerve("swerve_states", self._swerveStates)
        self.publishFloat("gyro_angle", self._gyro.getAngle() % 360)

        for i, state in enumerate(self._swerveStates):
            module = self._modules[i]
            name = f"{self._modules._fields[i]}_module"

            self.publishFloat(f"{name}_desired_speed", state.speed, "drive")
            self.publishFloat(f"{name}_actual_speed", module.driveVelocity, "drive")
            self.publishFloat(
                f"{name}_speed_delta", state.speed - module.driveVelocity, "drive"
            )
            self.publishFloat(f"{name}_position", module.driveDistance, "drive")

            self.publishFloat(
                f"{name}_desired_angle", state.angle.radians() / tau, "azimuth"
            )
            self.publishFloat(
                f"{name}_absolute_angle", module.absoluteAzimuthRotation, "azimuth"
            )
            self.publishFloat(
                f"{name}_relative_angle", module.azimuthRotation, "azimuth"
            )
            self.publishFloat(
                f"{name}_encoder_delta",
                module.absoluteAzimuthRotation - module.azimuthRotation,
                "azimuth",
            )

    def _configureDriveMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self._modules:
            module.configureDriveMotor(config=config)

    def _configureAzimuthMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self._modules:
            module.configureAzimuthMotor(config=config)

    def _symmetricDrive(
        self,
        *,
        frontLeftDriveID: int,
        frontLeftAzimuthID: int,
        frontLeftEncoderID: int,
        frontRightDriveID: int,
        frontRightAzimuthID: int,
        frontRightEncoderID: int,
        backLeftDriveID: int,
        backLeftAzimuthID: int,
        backLeftEncoderID: int,
        backRightDriveID: int,
        backRightAzimuthID: int,
        backRightEncoderID: int,
        xPos: meters,
        yPos: meters,
    ) -> SwerveModules:
        modules = SwerveModules(
            *(
                SwerveModule(
                    driveMotorID=drive,
                    azimuthMotorID=azimuth,
                    azimuthEncoderID=encoder,
                )
                for drive, azimuth, encoder in [
                    (frontLeftDriveID, frontLeftAzimuthID, frontLeftEncoderID),
                    (frontRightDriveID, frontRightAzimuthID, frontRightEncoderID),
                    (backLeftDriveID, backLeftAzimuthID, backLeftEncoderID),
                    (backRightDriveID, backRightAzimuthID, backRightEncoderID),
                ]
            )
        )
        modules.symmetricPosition(xPos, yPos)

        return modules
