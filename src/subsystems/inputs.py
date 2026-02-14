from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, lerp, Scalar
from typing import Optional
from wpilib import XboxController as Ctrlr
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import radians_per_second as RPS
from math import pi as PI


class Inputs(Subsystem):
    MAX_ABTAINABLE_SPEED: float = 4
    LOW_MAX_ABTAINABLE_SPEED: float = 1.5

    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
    ) -> None:
        super().__init__()
        # self.turretSpeed: float = 0.0

        self.robotState = RobotState.empty(abtainableMaxSpeed=self.MAX_ABTAINABLE_SPEED)

        self._driveCtrlr = Ctrlr(drivePort)
        self._mechCtrlr = Ctrlr(mechPort)

        self._linearScalar: Scalar = Scalar(magnitude=tau)
        self._circularScalar: CircularScalar = CircularScalar(
            magnitude=self.LOW_MAX_ABTAINABLE_SPEED
        )

        self._isTestMode: bool = False

    def phaseInit(
        self, drivePort: Optional[int] = None, mechPort: Optional[int] = None
    ) -> None:
        self._driveCtrlr = Ctrlr(drivePort) if drivePort else self._driveCtrlr
        self._mechCtrlr = Ctrlr(mechPort) if mechPort else self._mechCtrlr

    def periodic(self, robotState: RobotState) -> RobotState:
        self.robotState = robotState

        if self._driveCtrlr.getBackButtonPressed():
            self._isTestMode = not self._isTestMode

        if self._isTestMode:
            robotState.fieldSpeeds = ChassisSpeeds()
            if self._driveCtrlr.getBButton():
                robotState.fieldSpeeds = ChassisSpeeds(vx=5)
            elif self._driveCtrlr.getXButton():
                robotState.fieldSpeeds = ChassisSpeeds(omega=2)

        self.robotState.abtainableMaxSpeed = lerp(
            self.LOW_MAX_ABTAINABLE_SPEED,
            self.MAX_ABTAINABLE_SPEED,
            self._driveCtrlr.getRightTriggerAxis(),
        )
        self.robotState.fieldSpeeds = self._calculateDrive()
        self.robotState.resetGyro = self._driveCtrlr.getStartButtonPressed()

        self.robotState.motorDesiredState = self._linearScalar(
            self._mechCtrlr.getRightY()
        )
        self.robotState.motorDesiredState = self._linearScalar(
            self._mechCtrlr.getRightY()
        )

        self.robotState.revSpeed = self._mechCtrlr.getRightTriggerAxis()
        self.robotState.kickShooter = self._mechCtrlr.getRightBumper()

        self.robotState.turretSwitchManual = self._mechCtrlr.getYButtonPressed()
        self.robotState.turretManualSetpoint = self._mechCtrlr.getPOV()
        self.robotState.turretSwitchEnabled = self._mechCtrlr.getXButtonPressed()
        self.robotState.turretSwitchTarget = self._mechCtrlr.getBButtonPressed()

        return robotState

    def disabled(self) -> None:
        self.robotState.fieldSpeeds = ChassisSpeeds()

    def _calculateDrive(self) -> ChassisSpeeds:
        self._circularScalar.setMagnitude(self.robotState.abtainableMaxSpeed)
        vx, vy = self._circularScalar(
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
        )

        omega: RPS = self._linearScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
