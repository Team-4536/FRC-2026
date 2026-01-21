from __future__ import annotations
from math import tau as TAU
from navx import AHRS
from phoenix6.hardware import CANcoder
from phoenix6.units import rotation
from rev import SparkBaseConfig
from subsystems.desiredState import DesiredState
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


class SwerveModule:
    def __init__(
        self,
        *,
        driveMotor: RevMotor,
        azimuthMotor: RevMotor,
        azimuthEncoder: CANcoder,
        driveGearing: float,
        azimuthGearing: float,
    ) -> None:
        self._driveMotor = driveMotor
        self._azimuthMotor = azimuthMotor

        self._driveEncoder = self._driveMotor.getEncoder()
        self._azimuthEncoder = self._azimuthMotor.getEncoder()
        self._azimuthAbsoluteEncoder = azimuthEncoder

        self._driveGearing = driveGearing
        self._azimuthGearing = azimuthGearing
        self._position = Translation2d(0, 0)

        wheelDiam: meters = 0.1016
        self._wheelCircumferance: meters = wheelDiam * TAU

    @property
    def _driveDistance(self) -> meters:
        return (
            self._driveEncoder.getPosition()
            * self._wheelCircumferance
            / self._driveGearing
        )

    @property
    def _azimuthRotation(self) -> rotation:
        return self._azimuthEncoder.getPosition() / self._azimuthGearing

    @property
    def absoluteAzimuthRotation(self) -> rotation:
        return (
            self._azimuthAbsoluteEncoder.get_absolute_position().value
            * self._azimuthGearing
        )

    @property
    def modulePosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            distance=self._driveDistance,
            angle=Rotation2d.fromRotations(self._azimuthRotation),
        )

    @property
    def driveMotor(self) -> RevMotor:
        return self._driveMotor

    def configureDriveMotor(self, *, config: SparkBaseConfig) -> None:
        self._driveMotor.configure(config=config)

    @property
    def azimuthMotor(self) -> RevMotor:
        return self._azimuthMotor

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

        self.driveMotor.setVelocity(motorRPM)

    def setAzimuth(self, angle: Rotation2d) -> None:
        pos = self._azimuthGearing * radiansToRotations(angle.radians())
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
    def __init__(self, modules: SwerveModules) -> None:
        super().__init__()

        self._initPos = Pose2d()
        self._modules = modules
        self._kinematics = SwerveDrive4Kinematics(*modules.positions)
        self._gyro = AHRS(AHRS.NavXComType.kMXP_SPI)
        self._odometry = SwerveDrive4Odometry(
            self._kinematics,
            self._gyro.getRotation2d(),
            self._modules.modulePositions,
            self._initPos,
        )

    def init(self) -> None:
        for module in self._modules:
            module._driveEncoder.setPosition(0)
            module._azimuthEncoder.setPosition(module.absoluteAzimuthRotation)

    def periodic(self, ds: DesiredState) -> None:
        self.pose = self._odometry.update(
            self._gyro.getRotation2d(),
            self._modules.modulePositions,
        )  # UNUSED
        ds.yaw = self.pose.rotation().radians()
        self.drive(fieldSpeeds=ds.fieldSpeeds, attainableMaxSpeed=ds.abtainableMaxSpeed)

    def disabled(self) -> None:
        self._modules.stopModules()

    def publish(self) -> None:
        pass

    @property
    def modules(self) -> SwerveModules:
        return self._modules

    @property
    def kinematics(self) -> SwerveDrive4Kinematics:
        return self._kinematics

    def drive(self, fieldSpeeds: ChassisSpeeds, attainableMaxSpeed: MPS) -> None:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vx,
            fieldSpeeds.vy,
            fieldSpeeds.omega,
            -self._gyro.getRotation2d(),
        )
        moduleStates = self._kinematics.toSwerveModuleStates(chassisSpeeds)

        moduleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates=moduleStates,
            attainableMaxSpeed=attainableMaxSpeed,
        )

        self.publishDouble("gyro_angle", self._gyro.getAngle(), "debug")
        self.publishDouble(
            "unoptimized_angle", moduleStates[0].angle.degrees(), "debug"
        )

        for module, state in zip(self._modules, moduleStates):
            state.optimize(module.modulePosition.angle)
            module.setDrive(state.speed)
            module.setAzimuth(state.angle)

        self.publishDouble("optimized_angle", moduleStates[0].angle.degrees(), "debug")

        self.publishSwerve("swerve_states", moduleStates)

    def configureDriveMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self._modules:
            module.configureDriveMotor(config=config)

    def configureAzimuthMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self._modules:
            module.configureAzimuthMotor(config=config)

    @classmethod
    def symmetricDrive(
        cls,
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
        driveGearing: float = 1,
        azimuthGearing: float = 1,
        xPos: int,
        yPos: int,
    ) -> SwerveDrive:
        modules = SwerveModules(
            *(
                SwerveModule(
                    driveMotor=RevMotor(deviceID=drive),
                    azimuthMotor=RevMotor(deviceID=azimuth),
                    azimuthEncoder=CANcoder(device_id=encoder),
                    driveGearing=driveGearing,
                    azimuthGearing=azimuthGearing,
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

        return SwerveDrive(modules)
