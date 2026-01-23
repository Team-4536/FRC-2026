from math import tau as TAU
from navx import AHRS
from subsystems.networkTablesMixin import NetworkTablesMixin
from phoenix6.hardware import CANcoder
from phoenix6.units import rotation
from rev import SparkBaseConfig
from subsystems.robotState import RobotState
from subsystems.motor import RevMotor
from subsystems.subsystem import Subsystem
from typing import NamedTuple, Tuple
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)
from wpimath.units import meters_per_second as MPS
from wpimath.units import meters, radiansToRotations


class SwerveModule(NetworkTablesMixin):
    def __init__(
        self,
        *,
        driveMotor: RevMotor,
        azimuthMotor: RevMotor,
        azimuthEncoder: CANcoder,
    ) -> None:
        super().__init__()

        self.driveMotor = driveMotor
        self.azimuthMotor = azimuthMotor

        self.driveEncoder = self.driveMotor.getEncoder()
        self.azimuthEncoder = self.azimuthMotor.getEncoder()
        self.absoluteEncoder = azimuthEncoder

        self.driveGearing = RevMotor.DRIVE_GEARiNG
        self.azimuthGearing = RevMotor.AZIMUTH_GEARING
        self._position = Translation2d(0, 0)

        wheelDiam: meters = 0.1016
        self.wheelCircumferance: meters = wheelDiam * TAU

    @property
    def driveDistance(self) -> meters:
        return self.driveEncoder.getPosition() * self.wheelCircumferance / self.driveGearing

    @property
    def azimuthRotation(self) -> rotation:
        return self.azimuthEncoder.getPosition() / self.azimuthGearing

    @property
    def absoluteAzimuthRotation(self) -> rotation:
        return self.absoluteEncoder.get_absolute_position().value * self.azimuthGearing

    @property
    def modulePosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            distance=self.driveDistance,
            angle=Rotation2d.fromRotations(self.azimuthRotation),
        )

    def configureDriveMotor(self, *, config: SparkBaseConfig) -> None:
        self.driveMotor.configure(config=config)

    def configureAzimuthMotor(self, *, config: SparkBaseConfig) -> None:
        self.azimuthMotor.configure(config=config)

    @property
    def position(self) -> Translation2d:
        return self._position

    def setPosition(self, x: float, y: float) -> None:
        self._position = Translation2d(x, y)

    def setDrive(self, velocity: MPS) -> None:
        wheelRPS = velocity / self.wheelCircumferance
        motorRPS = wheelRPS * self.driveGearing
        motorRPM = motorRPS * 60

        self.driveMotor.setVelocity(motorRPM)

    def setAzimuth(self, angle: Rotation2d) -> None:
        pos = self.azimuthGearing * radiansToRotations(angle.radians())
        self.publishDouble("val 1", pos)
        self.azimuthMotor.setPosition(pos)

    def stopModule(self) -> None:
        self.driveMotor.stopMotor()
        self.azimuthMotor.stopMotor()


class SwerveModules(NamedTuple):
    frontLeft: SwerveModule
    frontRight: SwerveModule
    backLeft: SwerveModule
    backRight: SwerveModule

    def symmetricPosition(self, x: float, y: float) -> None:
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
    def __init__(self) -> None:
        super().__init__()

        WHEEL_DISTANCE: meters = 1
        self.modules = self.symmetricDrive(
            frontLeftDriveID=2,
            frontRightDriveID=8,
            backLeftDriveID=4,
            backRightDriveID=6,
            frontLeftAzimuthID=1,
            frontRightAzimuthID=7,
            backLeftAzimuthID=3,
            backRightAzimuthID=5,
            frontLeftEncoderID=21,
            frontRightEncoderID=24,
            backLeftEncoderID=22,
            backRightEncoderID=23,
            xPos=1,
            yPos=1,
        )
        self.modules.symmetricPosition(WHEEL_DISTANCE, WHEEL_DISTANCE)

        self.initPos = Pose2d()
        self.kinematics = SwerveDrive4Kinematics(*self.modules.positions)
        self.gyro = AHRS(AHRS.NavXComType.kMXP_SPI)

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            self.modules.modulePositions,
            self.initPos,
        )

    def init(self) -> None:
        for module in self.modules:
            module.driveEncoder.setPosition(0)
            module.azimuthEncoder.setPosition(module.absoluteAzimuthRotation)

        self.configureDriveMotors(config=RevMotor.DRIVE_CONFIG)
        self.configureAzimuthMotors(config=RevMotor.AZIMUTH_CONFIG)

    def periodic(self, robotState: RobotState) -> RobotState:
        self.pose = self.odometry.update(
            self.gyro.getRotation2d(),
            self.modules.modulePositions,
        )  # UNUSED

        self.drive(
            fieldSpeeds=robotState.fieldSpeeds, attainableMaxSpeed=robotState.abtainableMaxSpeed
        )

        return robotState

    def disabled(self) -> None:
        self.modules.stopModules()
        self.configureDriveMotors(config=RevMotor.DISABLED_DRIVE_CONFIG)
        self.configureAzimuthMotors(config=RevMotor.DISABLED_AZIMUTH_CONFIG)

    def drive(self, fieldSpeeds: ChassisSpeeds, attainableMaxSpeed: MPS) -> None:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vx,
            fieldSpeeds.vy,
            fieldSpeeds.omega,
            -self.gyro.getRotation2d(),
        )
        moduleStates = self.kinematics.toSwerveModuleStates(chassisSpeeds)

        moduleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates=moduleStates,
            attainableMaxSpeed=attainableMaxSpeed,
        )

        self.publishDouble("gyro_angle", self.gyro.getAngle(), "debug")
        self.publishDouble("unoptimized_angle", moduleStates[0].angle.degrees(), "debug")

        for module, state in zip(self.modules, moduleStates):
            state.optimize(module.modulePosition.angle)
            module.setDrive(state.speed)
            module.setAzimuth(state.angle)

        self.publishDouble("optimized_angle", moduleStates[0].angle.degrees(), "debug")

        self.publishSwerve("swerve_states", moduleStates)

    def configureDriveMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self.modules:
            module.configureDriveMotor(config=config)

    def configureAzimuthMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self.modules:
            module.configureAzimuthMotor(config=config)

    def symmetricDrive(
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
        xPos: int,
        yPos: int,
    ) -> SwerveModules:
        modules = SwerveModules(
            *(
                SwerveModule(
                    driveMotor=RevMotor(deviceID=drive),
                    azimuthMotor=RevMotor(deviceID=azimuth),
                    azimuthEncoder=CANcoder(device_id=encoder),
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
