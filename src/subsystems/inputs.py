from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, Scalar
from typing import Optional
from wpilib import XboxController as Ctrlr
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import radians_per_second as RPS
from subsystems.robotState import RobotState


class Inputs(Subsystem):
    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
    ) -> None:
        super().__init__()

        self._driveCtrlr = Ctrlr(drivePort)
        self._mechCtrlr = Ctrlr(mechPort)

        self.robotState = RobotState.empty(abtainableMaxSpeed=5)

        self._linearScalar: Scalar = Scalar(magnitude=tau)
        self._circularScalar: CircularScalar = CircularScalar(
            magnitude=self.robotState.abtainableMaxSpeed
        )

        self._isTestMode: bool = False

    def init(
        self, drivePort: Optional[int] = None, mechPort: Optional[int] = None
    ) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl

    def phaseInit(
        self, drivePort: Optional[int] = None, mechPort: Optional[int] = None
    ) -> None:
        self._driveCtrlr = Ctrlr(drivePort) if drivePort else self._driveCtrlr
        self._mechCtrlr = Ctrlr(mechPort) if mechPort else self._mechCtrlr

    def periodic(self, robotState: RobotState) -> RobotState:
        if self._driveCtrlr.getBackButtonPressed():
            self._isTestMode = not self._isTestMode

        if self._isTestMode:
            robotState.fieldSpeeds = ChassisSpeeds()
            if self._driveCtrlr.getBButton():
                robotState.fieldSpeeds = ChassisSpeeds(vx=5)
            elif self._driveCtrlr.getXButton():
                robotState.fieldSpeeds = ChassisSpeeds(omega=2)
            return robotState

        self.robotState.fieldSpeeds = self._calculateDrive()
        self.robotState.resetGyro = self._driveCtrlr.getStartButtonPressed()

        # def periodic(self, rs: RobotState) -> None:
        self.robotState.fieldSpeeds = self._calculateDrive()
        self.robotState.intakeManualButton = self._mechCtrlr.getAButton()
        self.robotState.intakeSensorTest = self._mechCtrlr.getBButton()
        self.robotState.intakeEjectButton = self._mechCtrlr.getLeftBumper()
        self.robotState.intakePosButton = self._mechCtrlr.getYButton()
        self.robotState.intakePosAxis = self._mechCtrlr.getLeftY()
        return robotState

    def disabled(self) -> None:
        self.robotState.fieldSpeeds = ChassisSpeeds()

    def _calculateDrive(self) -> ChassisSpeeds:
        vx, vy = self._circularScalar(
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
        )
        vx, vy = self._circularScalar(
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
        )

        omega: RPS = self._linearScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
