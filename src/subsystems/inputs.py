from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, lerp, Scalar
from wpilib import XboxController
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second


class Inputs(Subsystem):
    LOW_MAX_ABTAINABLE_SPEED: meters_per_second = 1.5
    MAX_ABTAINABLE_SPEED: meters_per_second = 5

    _driveCtrlr: XboxController
    _mechCtrlr: XboxController

    _linearDriveScalar: Scalar
    _circularDriveScalar: CircularScalar
    _linearScalar: Scalar

    def __init__(self, drivePort: int = 0, mechPort: int = 1) -> None:
        super().__init__()

        self._driveCtrlr = XboxController(drivePort)
        self._mechCtrlr = XboxController(mechPort)

        self._linearDriveScalar = Scalar(magnitude=tau)
        self._circularDriveScalar = CircularScalar(
            magnitude=self.LOW_MAX_ABTAINABLE_SPEED
        )
        self._linearScalar = Scalar()

        self._isTestMode: bool = False

        self.proxyControlMode: bool = False

        self.publishFloat("proxy drive x", 0)
        self.publishFloat("proxy drive y", 0)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        # Drive Controls
        self.proxyControlMode = self.getBoolean("proxy control mode", default=False)
        maxSpeed = lerp(
            self.LOW_MAX_ABTAINABLE_SPEED,
            self.MAX_ABTAINABLE_SPEED,
            min(1.0, self._driveCtrlr.getRightTriggerAxis() / 0.9),
        )
        robotState.fieldSpeeds = self._calculateDrive(maxSpeed)
        robotState.resetGyro = self._driveCtrlr.getStartButtonPressed()

        robotState.climbUp = self._driveCtrlr.getYButton()
        robotState.climbDown = self._driveCtrlr.getAButton()

        # Turret Controls
        robotState.turretSwitchMode = self._mechCtrlr.getYButtonPressed()
        robotState.turretManualSetpoint = self._mechCtrlr.getPOV()
        robotState.turretSwitchTarget = self._mechCtrlr.getXButtonPressed()
        robotState.revSpeed = self._mechCtrlr.getRightTriggerAxis()
        robotState.kickShooter = self._mechCtrlr.getRightBumper()

        # Intake Controls
        robotState.initialIntake = self._mechCtrlr.getAButton()
        robotState.intakeIndexer = self._mechCtrlr.getRightBumper()
        robotState.intakeEject = self._mechCtrlr.getBButton()
        # TODO chagne to not overlap with revspeed
        robotState.indexerEject = self._mechCtrlr.getBButton()
        robotState.intakePosYAxis = self._mechCtrlr.getRightY()
        robotState.intakeModeLeftBumperPressed = self._mechCtrlr.getLeftBumperPressed()
        robotState.ejectAll = self._mechCtrlr.getLeftTriggerAxis()
        robotState.intakePos = self._mechCtrlr.getBackButtonPressed()

        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        self.publishBoolean("proxy control mode", self.proxyControlMode)
        pass

    def _calculateDrive(self, maxSpeed: meters_per_second) -> ChassisSpeeds:
        self._circularDriveScalar.setMagnitude(maxSpeed)

        if not self.proxyControlMode:
            vx, vy = self._circularDriveScalar(
                x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
            )
        else:
            vx, vy = self._circularDriveScalar(
                x=self.getFloat("proxy drive x", default=0),
                y=self.getFloat("proxy drive y", default=0),
            )

        omega = self._linearDriveScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
