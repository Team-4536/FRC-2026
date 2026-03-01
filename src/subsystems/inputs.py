from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, lerp, Scalar
from wpilib import XboxController
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second


class Inputs(Subsystem):
    LOW_MAX_ABTAINABLE_SPEED: meters_per_second = 2
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

    def phaseInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        # Drive Controls
        maxSpeed = lerp(
            self.LOW_MAX_ABTAINABLE_SPEED,
            self.MAX_ABTAINABLE_SPEED,
            max(self._driveCtrlr.getRightTriggerAxis() / 0.9, 1.0),
        )
        robotState.fieldSpeeds = self._calculateDrive(maxSpeed)
        robotState.resetGyro = self._driveCtrlr.getStartButtonPressed()

        # Turret Controls
        robotState.turretSwitchMode = self._mechCtrlr.getYButtonPressed()
        robotState.turretManualSetpoint = self._mechCtrlr.getPOV()
        robotState.turretSwitchEnabled = self._mechCtrlr.getXButtonPressed()
        robotState.turretResetYawEncdoer = self._mechCtrlr.getStartButtonPressed()
        robotState.revSpeed = self._mechCtrlr.getRightTriggerAxis()
        robotState.kickShooter = self._mechCtrlr.getRightBumper()

        # Intake Controls
        robotState.initialIntake = self._mechCtrlr.getAButton()
        robotState.intakeIndexer = self._mechCtrlr.getRightBumper()
        robotState.intakeEject = self._mechCtrlr.getBButton()
        robotState.intakePosYAxis = self._mechCtrlr.getRightY()
        robotState.intakePos = self._mechCtrlr.getPOV()
        robotState.intakeMode = self._mechCtrlr.getLeftBumper()
        robotState.ejectAll = self._mechCtrlr.getLeftTriggerAxis()

        return robotState

    def disabled(self) -> None:
        pass

    def _calculateDrive(self, maxSpeed: meters_per_second) -> ChassisSpeeds:
        self._circularDriveScalar.setMagnitude(maxSpeed)
        vx, vy = self._circularDriveScalar(
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
        )

        omega = self._linearDriveScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
