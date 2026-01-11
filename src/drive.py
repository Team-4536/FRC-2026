from __future__ import annotations
from desiredState import DesiredState
from math import tau as TAU
from motor import RevMotor
from navx import AHRS
from rev import SparkBaseConfig
from subsystem import Subsystem
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
        position: Translation2d = Translation2d(0, 0),
    ) -> None:
        self._driveMotor = driveMotor
        self._driveGearing = driveGearing
        self._azimuthMotor = azimuthMotor
        self._azimuthGearing = azimuthGearing
        self._position = position

        self.absoluteAzimuth = self.azimuthMotor.getAbsoluteEncoder().getPosition()

        self.driveEncoder = self._driveMotor.getEncoder()
        self.azimuthEncoder = self._azimuthMotor.getEncoder()
        self.azimuthEncoder.setPosition(self.absoluteAzimuth * self._azimuthGearing)

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
    # TO DO: Fix mypy being angry
    def positions(self) -> Tuple[Translation2d, Translation2d, Translation2d, Translation2d]:
        return tuple(m.position for m in self)  # type: ignore


class SwerveDrive(Subsystem):
    def __init__(self, modules: SwerveModules) -> None:
        self._initPos = Pose2d()
        self._modules = modules
        self._kinematics = SwerveDrive4Kinematics(*modules.positions)
        self._gyro = AHRS(AHRS.NavXComType.kUSB1)
        self._odometry = SwerveDrive4Odometry(
            self._kinematics,
            self._gyro.getRotation2d(),
            (
                self._modules.frontLeft.modulePosition,
                self._modules.frontRight.modulePosition,
                self._modules.backLeft.modulePosition,
                self._modules.backRight.modulePosition,
            ),
            self._initPos,
        )

    def init(self) -> None:
        for m in self._modules:
            m.driveEncoder.setPosition(0)
            m.azimuthEncoder.setPosition(m.absoluteAzimuth * m._azimuthGearing)

    # TO DO: Eventually replace fieldSpeeds argument with DesiredState object so that it will not be an override error (;
    def periodic(self, ds: DesiredState) -> None:
        self.pose = self._odometry.update(
            self._gyro.getRotation2d(),
            (
                self._modules.frontLeft.modulePosition,
                self._modules.frontRight.modulePosition,
                self._modules.backLeft.modulePosition,
                self._modules.backRight.modulePosition,
            ),
        )

        self.drive(fieldSpeeds=ds.fieldSpeeds)

    def disable(self) -> None:
        pass

    @property
    def modules(self) -> SwerveModules:
        return self._modules

    @property
    def kinematics(self) -> SwerveDrive4Kinematics:
        return self._kinematics

    def drive(self, fieldSpeeds: ChassisSpeeds) -> None:
        moduleStates = self._kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldSpeeds.vx,
                fieldSpeeds.vy,
                fieldSpeeds.omega,
                Rotation2d.fromDegrees(-self._gyro.getAngle()),
            )
        )

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
