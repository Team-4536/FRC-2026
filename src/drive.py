from math import tau as TAU
from motor import RevMotor
from navx import AHRS
from rev import SparkMax
from subsystem import Subsystem
from typing import NamedTuple
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)
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

        wheelDiam = 0.1016
        self._wheelCircumferance = wheelDiam * TAU  # meters

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
    def driveMotor(self):
        return self._driveMotor

    def configureDriveMotor(self, *, config) -> None:
        self._driveMotor.configure(config=config)

    @property
    def azimuthMotor(self):
        return self._azimuthMotor

    def configureAzimuthMotor(self, *, config) -> None:
        self._azimuthMotor.configure(config=config)

    @property
    def position(self):
        return self._position

    def setPosition(self, x, y) -> None:
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
    def positions(self) -> tuple:
        return tuple(m.position for m in self)


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

    def periodic(self, fieldSpeeds: ChassisSpeeds) -> None:
        self.pose = self._odometry.update(
            self._gyro.getRotation2d(),
            (
                self._modules.frontLeft.modulePosition,
                self._modules.frontRight.modulePosition,
                self._modules.backLeft.modulePosition,
                self._modules.backRight.modulePosition,
            ),
        )

        self.drive(fieldSpeeds)

    def disable(self) -> None:
        pass

    @property
    def modules(self) -> SwerveModules:
        return self._modules

    @property
    def kinematics(self) -> SwerveDrive4Kinematics:
        return self._kinematics

    def drive(self, fieldSpeeds: ChassisSpeeds) -> None:
        moduleStates = self._kinematics.toSwerveModuleStates(fieldSpeeds)

        moduleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates=moduleStates,
            attainableMaxSpeed=5,
        )

        for module, state in zip(self._modules, moduleStates):
            state.optimize(module.modulePosition.angle)
            module.setDrive(state.speed)
            module.setAzimuth(state.angle)

    def configureDriveMotors(self, *, config):
        for module in self._modules:
            module.configureDriveMotor(config=config)

    def configureAzimuthMotors(self, *, config):
        for module in self._modules:
            module.configureAzimuthMotor(config=config)

    @classmethod
    def symmetricDrive(
        cls,
        *,
        frontLeftDriveID: int,
        frontLeftAzimuthID: int,
        frontRightDriveID: int,
        frontRightAzimuthID: int,
        backLeftDriveID: int,
        backLeftAzimuthID: int,
        backRightDriveID: int,
        backRightAzimuthID: int,
        xPos: int,
        yPos: int,
        driveGearing: float = 1,
        azimuthGearing: float = 1,
    ):
        modules = SwerveModules(
            *(
                SwerveModule(
                    driveMotor=RevMotor(id=drive),
                    azimuthMotor=RevMotor(id=azimuth),
                    driveGearing=driveGearing,
                    azimuthGearing=azimuthGearing,
                )
                for drive, azimuth in [
                    (frontLeftDriveID, frontLeftAzimuthID),
                    (frontRightDriveID, frontRightAzimuthID),
                    (backLeftDriveID, backLeftAzimuthID),
                    (backRightDriveID, backRightAzimuthID),
                ]
            )
        )
        modules.symmetricPosition(xPos, yPos)

        return SwerveDrive(modules)
