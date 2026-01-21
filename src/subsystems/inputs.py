from math import tau as TAU
from subsystems.robotState import DesiredState, CurrentState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, Scalar
from typing import Optional
from wpilib import XboxController as Ctrlr
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import radians_per_second as RPS


class Inputs(Subsystem):
    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
    ) -> None:
        super().__init__()

        self._driveCtrl = Ctrlr(drivePort)
        self._mechCtrl = Ctrlr(mechPort)

        self._desiredState = DesiredState(fieldSpeeds=ChassisSpeeds())

        self._linearScalar: Scalar = Scalar(magnitude=TAU)
        self._circularScalar: CircularScalar = CircularScalar(
            magnitude=self._desiredState.abtainableMaxSpeed
        )

    def init(self, drivePort: Optional[int] = None, mechPort: Optional[int] = None) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl

    def periodic(self, desiredState: DesiredState, currentState: CurrentState) -> None:
        self._desiredState.fieldSpeeds = self._calculateDrive()

        self.publish()

    def disabled(self) -> None:
        self.desiredState.fieldSpeeds = ChassisSpeeds()

    def publish(self) -> None:
        self.desiredState.publish()

    def _calculateDrive(self) -> ChassisSpeeds:
        vx, vy = self._circularScalar(x=-self._driveCtrl.getLeftY(), y=-self._driveCtrl.getLeftX())

        omega: RPS = self._linearScalar(-self._driveCtrl.getRightX())

        return ChassisSpeeds(vx, vy, omega)

    @property
    def desiredState(self) -> DesiredState:
        return self._desiredState
