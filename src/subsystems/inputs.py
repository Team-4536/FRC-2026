from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, lerp, Scalar
from wpilib import XboxController
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import radians_per_second as RPS, meters_per_second
from math import pi as PI


class Inputs(Subsystem):
    MAX_ABTAINABLE_SPEED: float = 5
    LOW_MAX_ABTAINABLE_SPEED: float = 2

    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
    ) -> None:
        super().__init__()
        # self.turretSpeed: float = 0.0

        self._driveCtrlr = XboxController(drivePort)
        self._mechCtrlr = XboxController(mechPort)

        self._linearScalar: Scalar = Scalar(magnitude=tau)
        self._circularScalar: CircularScalar = CircularScalar(
            magnitude=self.LOW_MAX_ABTAINABLE_SPEED
        )

        self._isTestMode: bool = False

    def phaseInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        # Drive Controls

        maxSpeed = lerp(
            self.LOW_MAX_ABTAINABLE_SPEED,
            self.MAX_ABTAINABLE_SPEED,
            max(self._driveCtrlr.getRightTriggerAxis() / 0.9, 1),
        )

        if self._driveCtrlr.getBackButtonPressed():
            self._isTestMode = not self._isTestMode

        if self._isTestMode:
            robotState.fieldSpeeds = ChassisSpeeds()
            if self._driveCtrlr.getBButton():
                robotState.fieldSpeeds = ChassisSpeeds(vx=5)
            elif self._driveCtrlr.getXButton():
                robotState.fieldSpeeds = ChassisSpeeds(omega=2)

        robotState.fieldSpeeds = self._calculateDrive(maxSpeed)
        robotState.resetGyro = self._driveCtrlr.getStartButtonPressed()

        robotState.motorDesiredState = self._linearScalar(self._mechCtrlr.getRightY())
        robotState.motorDesiredState = self._linearScalar(self._mechCtrlr.getRightY())

        robotState.revSpeed = self._mechCtrlr.getRightTriggerAxis()
        robotState.kickShooter = self._mechCtrlr.getRightBumper()

        robotState.turretSwitchMode = self._mechCtrlr.getYButtonPressed()
        robotState.turretManualSetpoint = self._mechCtrlr.getPOV()
        robotState.turretSwitchEnabled = self._mechCtrlr.getXButtonPressed()
        robotState.turretResetYawEncdoer = self._mechCtrlr.getStartButtonPressed()
        robotState.initialIntake = self._mechCtrlr.getAButton()
        robotState.intakeIndexer = self._mechCtrlr.getRightBumper()
        robotState.intakeEject = self._mechCtrlr.getBButton()
        robotState.intakePosYAxis = self._mechCtrlr.getLeftY()
        robotState.intakePos = self._mechCtrlr.getPOV()
        robotState.intakeMode = self._mechCtrlr.getLeftBumper()
        robotState.ejectAll = self._mechCtrlr.getLeftTriggerAxis()

        return robotState

    def disabled(self) -> None:
        pass

    def _calculateDrive(self, maxSpeed: meters_per_second) -> ChassisSpeeds:
        self._circularScalar.setMagnitude(maxSpeed)
        vx, vy = self._circularScalar(
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
        )

        omega = self._linearScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
