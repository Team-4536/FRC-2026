from math import pi as PI
from navx import AHRS
from phoenix6.hardware import CANcoder
from phoenix6.units import rotation
from rev import SparkBaseConfig
from subsystems.motor import RevMotor
from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from typing import NamedTuple, Tuple
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.units import meters_per_second as MPS
from wpimath.units import meters, radiansToRotations, feetToMeters


class SwerveModule(NetworkTablesMixin):
    def __init__(
        self,
        *,
        driveMotorID: int,
        azimuthMotorID: int,
        azimuthEncoderID: int,
    ) -> None:
        super().__init__()

        self.driveMotor = RevMotor(deviceID=driveMotorID)
        self.azimuthMotor = RevMotor(deviceID=azimuthMotorID)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.azimuthEncoder = self.azimuthMotor.getEncoder()
        self.absoluteEncoder = CANcoder(device_id=azimuthEncoderID)

        self.driveGearing = RevMotor.DRIVE_GEARiNG
        self.azimuthGearing = RevMotor.AZIMUTH_GEARING
        self._position = Translation2d(0, 0)

        wheelDiam: meters = 0.1016
        self.wheelCircumferance: meters = wheelDiam * PI

    @property
    def driveDistance(self) -> meters:
        return (
            self.driveEncoder.getPosition()
            * self.wheelCircumferance
            / self.driveGearing
        )

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
        self.azimuthMotor.setPosition(pos)

    def stopModule(self) -> None:
        self.driveMotor.stopMotor()
        self.azimuthMotor.stopMotor()


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
    def __init__(self) -> None:
        super().__init__()

        WHEEL_DISTANCE: meters = feetToMeters(1)
        # self._modules = self.symmetricDrive(
        #     frontLeftDriveID=2,
        #     frontRightDriveID=8,
        #     backLeftDriveID=4,
        #     backRightDriveID=6,
        #     frontLeftAzimuthID=1,
        #     frontRightAzimuthID=7,
        #     backLeftAzimuthID=3,
        #     backRightAzimuthID=5,
        #     frontLeftEncoderID=21,
        #     frontRightEncoderID=24,
        #     backLeftEncoderID=22,
        #     backRightEncoderID=23,
        #     xPos=1,
        #     yPos=1,
        # )
        self._modules = self.symmetricDrive(
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
        self.gyro = AHRS(AHRS.NavXComType.kMXP_SPI)

        self.odometry = SwerveDrive4Odometry(
            self._kinematics,
            self.gyro.getRotation2d(),
            self._modules.modulePositions,
            self.initPos,
        )

    def init(self) -> None:
        # self.configureDriveMotors(config=RevMotor.DRIVE_CONFIG)
        self.configureAzimuthMotors(config=RevMotor.AZIMUTH_CONFIG)

        self._modules.frontRight.driveMotor.configure(
            config=RevMotor.DRIVE_CONFIG.apply(SparkBaseConfig().inverted(True))
        )
        self._modules.backRight.driveMotor.configure(
            config=RevMotor.DRIVE_CONFIG.apply(SparkBaseConfig().inverted(True))
        )
        self._modules.frontLeft.driveMotor.configure(
            config=RevMotor.DRIVE_CONFIG.apply(SparkBaseConfig().inverted(False))
        )
        self._modules.backLeft.driveMotor.configure(
            config=RevMotor.DRIVE_CONFIG.apply(SparkBaseConfig().inverted(False))
        )

        for module in self._modules:
            module.driveEncoder.setPosition(0)
            module.azimuthEncoder.setPosition(module.absoluteAzimuthRotation)

    def periodic(self, robotState: RobotState) -> RobotState:
        robotState.pose = self.odometry.update(
            self.gyro.getRotation2d(),
            self._modules.modulePositions,
        )

        self.drive(
            fieldSpeeds=robotState.fieldSpeeds,
            attainableMaxSpeed=robotState.abtainableMaxSpeed,
        )

        return robotState

    def disabled(self) -> None:
        self._modules.stopModules()
        self.configureDriveMotors(config=RevMotor.DISABLED_DRIVE_CONFIG)
        self.configureAzimuthMotors(config=RevMotor.DISABLED_AZIMUTH_CONFIG)

    def drive(self, fieldSpeeds: ChassisSpeeds, attainableMaxSpeed: MPS) -> None:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vx,
            fieldSpeeds.vy,
            fieldSpeeds.omega,
            self.gyro.getRotation2d(),
        )
        moduleStates = self._kinematics.toSwerveModuleStates(chassisSpeeds)

        moduleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates=moduleStates,
            attainableMaxSpeed=attainableMaxSpeed,
        )

        i = 1
        for module, state in zip(self._modules, moduleStates):
            state.optimize(module.modulePosition.angle)
            module.setDrive(state.speed)
            module.setAzimuth(state.angle)

            self.publishDouble(
                f"abs encoder {i}",
                module.absoluteEncoder.get_absolute_position().value_as_double,
                "debug",
            )
            self.publishDouble(
                f"encoder {i}", module.azimuthEncoder.getPosition(), "debug"
            )
            self.publishDouble(
                f"setpoint {i}",
                RevMotor.AZIMUTH_GEARING * radiansToRotations(state.angle.radians()),
                "debug",
            )
            i += 1

        self._publish(swerveStates=moduleStates)

    def configureDriveMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self._modules:
            module.configureDriveMotor(config=config)

    def configureAzimuthMotors(self, *, config: SparkBaseConfig) -> None:
        for module in self._modules:
            module.configureAzimuthMotor(config=config)

    def _publish(
        self,
        swerveStates: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ):
        self.publishSwerve("swerve_states", swerveStates)
        self.publishDouble("gyro_angle", self.gyro.getAngle() % 360)
        self.publishDouble("module_1_desired_speed", swerveStates[0].speed)
        self.publishDouble(
            "module_1_actual_speed",
            self._modules.frontLeft.driveEncoder.getVelocity()
            * self._modules.frontLeft.wheelCircumferance
            / RevMotor.DRIVE_GEARiNG
            / 60,
        )

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
