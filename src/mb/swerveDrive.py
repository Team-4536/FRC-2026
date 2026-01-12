from __future__ import annotations
from math import tau as TAU
from mb.desiredState import DesiredState
from mb.motor import RevMotor
from mb.subsystem import Subsystem
from navx import AHRS
from rev import SparkBaseConfig
from typing import NamedTuple, Tuple
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)
from wpimath.units import meters
from wpimath.units import meters_per_second as MPS
from wpimath.units import radiansToRotations


class SwerveModule:
    def __init__(
        self,
        *,
        driveMotor: RevMotor,
        driveGearing: float,
        azimuthMotor: RevMotor,
        azimuthGearing: float,
    ) -> None:
        self._driveMotor = driveMotor
        self._driveGearing = driveGearing
        self._azimuthMotor = azimuthMotor
        self._azimuthGearing = azimuthGearing
        self._position = Translation2d(0, 0)

        self.absoluteAzimuth = self.azimuthMotor.getAbsoluteEncoder().getPosition()

        self.driveEncoder = self._driveMotor.getEncoder()
        self.azimuthEncoder = self._azimuthMotor.getEncoder()
        self.azimuthEncoder.setPosition(self.absoluteAzimuth)

        wheelDiam: meters = 0.1016
        self._wheelCircumferance: meters = wheelDiam * TAU

    @property
    def modulePosition(self) -> SwerveModulePosition:
        driveDistance = (
            self.driveEncoder.getPosition() * self._wheelCircumferance / self._driveGearing
        )
        azimuthRotation = Rotation2d.fromRotations(
            self.azimuthEncoder.getPosition() / self._azimuthGearing
        )

        return SwerveModulePosition(driveDistance, azimuthRotation)

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

    @property
    def positions(self) -> Tuple[Translation2d, Translation2d, Translation2d, Translation2d]:
        return tuple(m.position for m in self)  # type: ignore[return-value]

    @property
    def modulePositions(
        self,
    ) -> Tuple[
        SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition
    ]:
        return tuple(m.modulePosition for m in self)  # type: ignore[return-value]


class SwerveDrive(Subsystem):
    def __init__(self, modules: SwerveModules) -> None:
        super().__init__()

        self._initPos = Pose2d()
        self._modules = modules
        self._kinematics = SwerveDrive4Kinematics(*modules.positions)
        self._gyro = AHRS(AHRS.NavXComType.kUSB1)
        self._odometry = SwerveDrive4Odometry(
            self._kinematics,
            self._gyro.getRotation2d(),
            self._modules.modulePositions,
            self._initPos,
        )

    def init(self) -> None:
        for module in self._modules:
            module.driveEncoder.setPosition(0)
            module.azimuthEncoder.setPosition(module.absoluteAzimuth * module._azimuthGearing)

    def periodic(self, ds: DesiredState) -> None:
        self.pose = self._odometry.update(
            self._gyro.getRotation2d(),
            self._modules.modulePositions,
        )

        self.drive(fieldSpeeds=ds.fieldSpeeds)

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass

    @property
    def modules(self) -> SwerveModules:
        return self._modules

    @property
    def kinematics(self) -> SwerveDrive4Kinematics:
        return self._kinematics

    def drive(self, fieldSpeeds: ChassisSpeeds) -> None:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vx,
            fieldSpeeds.vy,
            fieldSpeeds.omega,
            Rotation2d.fromDegrees(-self._gyro.getAngle()),
        )
        moduleStates = self._kinematics.toSwerveModuleStates(chassisSpeeds)

        moduleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates=moduleStates,
            attainableMaxSpeed=5,
        )

        for module, state in zip(self._modules, moduleStates):
            state.optimize(module.modulePosition.angle)
            module.setDrive(state.speed)
            module.setAzimuth(state.angle)

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
        flDriveID: int,
        flAzimuthID: int,
        frDriveID: int,
        frAzimuthID: int,
        blDriveID: int,
        blAzimuthID: int,
        brDriveID: int,
        brAzimuthID: int,
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
                    driveGearing=driveGearing,
                    azimuthGearing=azimuthGearing,
                )
                for drive, azimuth in [
                    (flDriveID, flAzimuthID),
                    (frDriveID, frAzimuthID),
                    (blDriveID, blAzimuthID),
                    (brDriveID, brAzimuthID),
                ]
            )
        )
        modules.symmetricPosition(xPos, yPos)

        return SwerveDrive(modules)
