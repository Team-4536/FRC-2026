from enum import Enum
from mb.subsystem import Subsystem
from mb.motors import Motor, RevMotor
from typing import NamedTuple
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from wpimath.units import meters_per_second as MPS
from wpimath.units import radiansToRotations


# class SwerveModulePosition(Enum):
#     FRONT_LEFT = "lf"
#     FRONT_RIGHT = "rf"
#     BACK_LEFT = "lr"
#     BACK_RIGHT = "rr"


class SwerveModule:
    def __init__(
        self,
        *,
        driveMotor: Motor,
        driveGearing: float,
        azimuthMotor: Motor,
        azimuthGearing: float,
        position: Translation2d = Translation2d(0, 0),
    ) -> None:
        self._driveMotor = driveMotor
        self._driveGearing = driveGearing
        self._azimuthMotor = azimuthMotor
        self._azimuthGearing = azimuthGearing
        self._position = position

        self._azimuth = Rotation2d(0)

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
        self.driveMotor.setVelocity(velocity)  # *10000

    def setAzimuth(self, angle: Rotation2d) -> None:
        self._azimuth = angle
        pos = self._azimuthGearing * radiansToRotations(angle.radians())
        self.azimuthMotor.setPosition(pos)


class SwerveModules(NamedTuple):
    frontLeft: SwerveModule
    frontRight: SwerveModule
    backLeft: SwerveModule
    backRight: SwerveModule

    def symmetricPosition(self, x: float, y: float) -> None:
        self.frontLeft.setPosition(x, y)
        self.frontRight.setPosition(x, y)
        self.backLeft.setPosition(x, y)
        self.backRight.setPosition(x, y)

    @property
    def positions(self):
        return [m.position for m in self]


class SwerveDrive(Subsystem):
    def __init__(self, modules: SwerveModules) -> None:
        self._modules = modules
        self._kinematics = SwerveDrive4Kinematics(*(modules.positions))

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
            state.optimize(module._azimuth)  # fix
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
        frontLeftDriveDev: int,
        frontLeftAzimuthDev: int,
        frontLeftEncoderDev: int,
        frontRightDriveDev: int,
        frontRightAzimuthDev: int,
        frontRightEncoderDev: int,
        backLeftDriveDev: int,
        backLeftAzimuthDev: int,
        backLeftEncoderDev: int,
        backRightDriveDev: int,
        backRightAzimuthDev: int,
        backRightEncoderDev: int,
        xPos: int,
        yPos: int,
        driveGearing: float = 1,
        azimuthGearing: float = 1,
    ):
        modules = SwerveModules(
            *(
                SwerveModule(
                    driveMotor=RevMotor(name=f"{name}_drive", id=drive),
                    azimuthMotor=RevMotor(name=f"{name}_azimuth", id=azimuth),
                    # encoder=CANcoder(encoder),
                    driveGearing=driveGearing,
                    azimuthGearing=azimuthGearing,
                )
                for name, drive, azimuth, encoder in [
                    ("lf", frontLeftDriveDev, frontLeftAzimuthDev, frontLeftEncoderDev),
                    ("rf", frontRightDriveDev, frontRightAzimuthDev, frontRightEncoderDev),
                    ("lr", backLeftDriveDev, backLeftAzimuthDev, backLeftEncoderDev),
                    ("rr", backRightDriveDev, backRightAzimuthDev, backRightEncoderDev),
                ]
            )
        )
        modules.symmetricPosition(xPos, yPos)

        return SwerveDrive(modules)
