from math import pi, tau
from navx import AHRS
from phoenix6.hardware import CANcoder
from rev import SparkBaseConfig
from subsystems.motor import RevMotor
from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import getTangentAngle, getContributedRotation, matchData
from typing import NamedTuple, Self, SupportsFloat, SupportsIndex, Tuple, Union
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.units import (
    degrees,
    inchesToMeters,
    meters_per_second,
    meters,
    radians_per_second,
)


class SwerveModule(NetworkTablesMixin):
    WHEEL_CIRCUMFERENCE: meters = 0.1016 * pi
    DRIVE_GEARING: float = RevMotor.DRIVE_GEARING
    AZIMUTH_GEARING: float = RevMotor.AZIMUTH_GEARING

    _driveMotor: RevMotor
    _azimuthMotor: RevMotor
    _absoluteEncoder: CANcoder

    _position: Translation2d

    def __init__(
        self,
        *,
        driveMotorID: int,
        azimuthMotorID: int,
        azimuthEncoderID: int,
        position: Translation2d = Translation2d(),
    ) -> None:
        super().__init__()

        self._driveMotor = RevMotor(deviceID=driveMotorID)
        self._azimuthMotor = RevMotor(deviceID=azimuthMotorID)
        self._absoluteEncoder = CANcoder(device_id=azimuthEncoderID)

        self._position = position

        self._driveMotor.setPosition(0)
        self.resetAzimuthEncoder()

    @property
    def driveDistance(self) -> meters:
        motorRot = self._driveMotor.getPosition()
        wheelRot = motorRot / self.DRIVE_GEARING
        wheelDist = wheelRot * self.WHEEL_CIRCUMFERENCE
        return wheelDist

    @property
    def driveVelocity(self) -> meters_per_second:
        motorRPS = self._driveMotor.getVelocity() / 60
        wheelRPS = motorRPS / self.DRIVE_GEARING
        wheelMPS = wheelRPS * self.WHEEL_CIRCUMFERENCE
        return wheelMPS

    @property
    def azimuthRotation(self) -> Rotation2d:
        moduleRot = self._azimuthMotor.getPosition() / self.AZIMUTH_GEARING
        return Rotation2d.fromRotations(moduleRot)

    @property
    def absoluteAzimuthRotation(self) -> Rotation2d:
        moduleRot = self._absoluteEncoder.get_absolute_position().value
        return Rotation2d.fromRotations(moduleRot)

    @property
    def modulePosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            distance=self.driveDistance,
            angle=self.azimuthRotation,
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

    def setDrive(self, velocity: meters_per_second) -> None:
        wheelRPS = velocity / self.WHEEL_CIRCUMFERENCE
        motorRPS = wheelRPS * self.DRIVE_GEARING
        motorRPM = motorRPS * 60
        self._driveMotor.setVelocity(motorRPM)

    def setAzimuth(self, angle: Rotation2d) -> None:
        motorRot = self.AZIMUTH_GEARING * angle.radians() / tau
        self._azimuthMotor.setPosition(motorRot)

    def stopModule(self) -> None:
        self._driveMotor.stopMotor()
        self._azimuthMotor.stopMotor()

    def resetAzimuthEncoder(self) -> None:
        absRot = self.absoluteAzimuthRotation.radians() / tau
        motorRot = absRot * self.AZIMUTH_GEARING
        self._azimuthMotor.setPosition(motorRot)


class SwerveModules(NamedTuple):
    frontLeft: SwerveModule
    frontRight: SwerveModule
    backLeft: SwerveModule
    backRight: SwerveModule

    def stopModules(self) -> None:
        for m in self:
            m.stopModule()

    def configureAzimuthMotors(self, *, config: SparkBaseConfig):
        for m in self:
            m.configureAzimuthMotor(config=config)

    def configureDriveMotors(self, *, config: SparkBaseConfig):
        for m in self:
            m.configureDriveMotor(config=config)

    @property
    def positions(
        self,
    ) -> Tuple[Translation2d, Translation2d, Translation2d, Translation2d]:
        return tuple(m.position for m in self)  # type: ignore

    @property
    def modulePositions(self) -> Tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return tuple(m.modulePosition for m in self)  # type: ignore


class Gyroscope(AHRS):
    _simYaw: Rotation2d = Rotation2d()
    _angleAdjustment: Rotation2d = Rotation2d()

    def __init__(self, comType: AHRS.NavXComType = AHRS.NavXComType.kMXP_SPI) -> None:
        super().__init__(comType)

    def reset(self) -> None:
        super().reset()
        self._simYaw = Rotation2d()

    def setAngleAdjustment(self, angle: Union[SupportsFloat, SupportsIndex]) -> None:
        super().setAngleAdjustment(angle)
        self._angleAdjustment = Rotation2d.fromDegrees(float(angle))

    def getYaw(self) -> degrees:
        if matchData.isSimulation():
            return self._simYaw.degrees()
        return super().getYaw()

    def getAngle(self) -> degrees:
        if matchData.isSimulation():
            return self._simYaw.degrees() - self._angleAdjustment.degrees()
        return super().getAngle()

    def getRotation2d(self) -> Rotation2d:
        if matchData.isSimulation():
            return self._simYaw - self._angleAdjustment
        return super().getRotation2d()

    def update(self, angularVelocity: radians_per_second) -> None:
        self._simYaw += Rotation2d(angularVelocity * matchData.dt)


class SwerveDrive(Subsystem):
    MAX_MODULE_SPEED: meters_per_second = 6
    ROBOT_RADIUS = inchesToMeters(Translation2d(11, 11).norm())

    _modules: SwerveModules
    _gyro: Gyroscope
    _kinematics: SwerveDrive4Kinematics
    _swerveStates: Tuple[SwerveModuleState, ...]

    _angleAdjustment: Rotation2d = Rotation2d()
    _disabledModules: bool = True

    def __init__(self, swerveModules: SwerveModules) -> None:
        super().__init__()

        self._modules = swerveModules

        self._kinematics = SwerveDrive4Kinematics(*self._modules.positions)
        self._gyro = Gyroscope(AHRS.NavXComType.kMXP_SPI)
        self._gyro.reset()

        self._swerveStates = self._kinematics.desaturateWheelSpeeds(
            self._kinematics.toSwerveModuleStates(ChassisSpeeds()), 0
        )

        self._disableModules()

    def phaseInit(self, robotState: RobotState) -> None:
        self._modules.configureDriveMotors(config=RevMotor.DRIVE_CONFIG)
        self._modules.configureAzimuthMotors(config=RevMotor.AZIMUTH_CONFIG)
        self._disabledModules = False

        for m in self._modules:
            m.resetAzimuthEncoder()

    def robotPeriodic(self, robotState: RobotState) -> None:
        robotState.odometry.update(
            self._gyro.getRotation2d(),
            self._modules.modulePositions,
        )
        robotState.gyro = self._gyro.getRotation2d()

    def periodic(self, robotState: RobotState) -> None:
        if robotState.resetGyro:
            self._angleAdjustment = self._gyro.getRotation2d()
            robotState.resetGyro = False

        if robotState.autosGyroResetToggle:
            self._gyro.setAngleAdjustment(robotState.autosGyroReset)
            robotState.autosGyroResetToggle = False

        self.drive(fieldSpeeds=robotState.fieldSpeeds)

        vx, vy = self.getLinearVelocity(
            robotState.odometry.getEstimatedPosition().rotation()
        )
        omega = self.getAngularVelocity()
        robotState.robotVelocity = ChassisSpeeds(vx, vy, omega)

        self._gyro.update(robotState.fieldSpeeds.omega)

    def disabled(self) -> None:
        self._modules.stopModules()
        if not self._disabledModules and matchData.timeSincePhaseInit > 1.5:
            self._disableModules()

    def getDriveVelocity(self, module: SwerveModule) -> Translation2d:
        speed = module.driveVelocity
        angle = module.azimuthRotation
        if abs(speed) < 1e-4:
            return Translation2d()
        vector = Translation2d(distance=speed, angle=angle)
        return vector

    def getLinearVelocity(self, roboRotation: Rotation2d) -> Translation2d:
        vector = Translation2d()

        for module in self._modules:
            vector += self.getDriveVelocity(module)

        if vector.norm() < 1e-4:
            return Translation2d()

        vector /= 4

        return vector.rotateBy(roboRotation)

    def getAngularVelocity(self) -> radians_per_second:
        sum = 0
        for module in self._modules:
            tanVel = getTangentAngle(module.position)
            sum += getContributedRotation(
                tanVel,
                self.getDriveVelocity(module),
            )

        return sum / (4 * self.ROBOT_RADIUS)

    def drive(self, fieldSpeeds: ChassisSpeeds) -> None:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vx,
            fieldSpeeds.vy,
            fieldSpeeds.omega,
            self._gyro.getRotation2d() - self._angleAdjustment,
        )
        moduleStates = self._kinematics.toSwerveModuleStates(chassisSpeeds)

        self._swerveStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates=moduleStates,
            attainableMaxSpeed=self.MAX_MODULE_SPEED,
        )

        for module, state in zip(self._modules, self._swerveStates):
            state.optimize(module.modulePosition.angle)
            module.setDrive(state.speed)
            module.setAzimuth(state.angle)

    def publish(self) -> None:
        self.publishStructArray("swerve_states", self._swerveStates)
        self.publishFloat("gyro_angle", self._gyro.getAngle() % 360)

        for i, state in enumerate(self._swerveStates):
            module, name = self._modules[i], self._modules._fields[i]

            self.publishFloat(f"{name}_desired_speed", state.speed, "drive", debug=True)
            self.publishFloat(
                f"{name}_actual_speed", module.driveVelocity, "drive", debug=True
            )
            self.publishFloat(
                f"{name}_speed_delta",
                state.speed - module.driveVelocity,
                "drive",
                debug=True,
            )

            absRot = module.absoluteAzimuthRotation.radians()
            relRot = module.azimuthRotation.radians()

            self.publishFloat(
                f"{name}_desired_angle", state.angle.radians(), "azimuth", debug=True
            )
            self.publishFloat(f"{name}_absolute_angle", absRot, "azimuth", debug=True)
            self.publishFloat(f"{name}_relative_angle", relRot, "azimuth", debug=True)

    def _disableModules(self) -> None:
        self._modules.configureDriveMotors(config=RevMotor.DISABLED_DRIVE_CONFIG)
        self._modules.configureAzimuthMotors(config=RevMotor.DISABLED_AZIMUTH_CONFIG)
        self._disabledModules = True

    @property
    def kinematics(self) -> SwerveDrive4Kinematics:
        return self._kinematics

    @property
    def modulePoses(self) -> Tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return self._modules.modulePositions

    @classmethod
    def symmetricDrive(  # TODO: remove defaults and set IDs in robot.py
        cls,
        *,
        FL_DriveID: int = 2,
        FR_DriveID: int = 4,
        BL_DriveID: int = 6,
        BR_DriveID: int = 8,
        FL_AzimuthID: int = 1,
        FR_AzimuthID: int = 3,
        BL_AzimuthID: int = 5,
        BR_AzimuthID: int = 7,
        FL_EncoderID: int = 21,
        FR_EncoderID: int = 22,
        BL_EncoderID: int = 23,
        BR_EncoderID: int = 24,
        xPos: meters,
        yPos: meters,
    ) -> Self:
        modules = SwerveModules(
            *(
                SwerveModule(
                    driveMotorID=drive,
                    azimuthMotorID=azimuth,
                    azimuthEncoderID=encoder,
                    position=Translation2d(x, y),
                )
                for drive, azimuth, encoder, x, y in [
                    (FL_DriveID, FL_AzimuthID, FL_EncoderID, xPos, yPos),
                    (FR_DriveID, FR_AzimuthID, FR_EncoderID, xPos, -yPos),
                    (BL_DriveID, BL_AzimuthID, BL_EncoderID, -xPos, yPos),
                    (BR_DriveID, BR_AzimuthID, BR_EncoderID, -xPos, -yPos),
                ]
            )
        )

        return cls(modules)
