from math import pi as PI
from mb.desiredState import DesiredState
from mb.subsystem import Subsystem
from mb.utils import CircularScalar, Scalar
from typing import Optional
from wpilib import XboxController as Ctrlr
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS


class Inputs(Subsystem):
    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
        maxVelocity: MPS = 5,
        maxAngularVelocity: RPS = PI,
    ) -> None:
        super().__init__()

        self._driveCtrl = Ctrlr(drivePort)
        self._mechCtrl = Ctrlr(mechPort)

        self._linearScalar: Scalar = Scalar(magnitude=maxAngularVelocity)
        self._circularScalar: CircularScalar = CircularScalar(magnitude=maxVelocity)

        self.desiredState = DesiredState(fieldSpeeds=ChassisSpeeds())
        self.desiredState.init()

    def init(self, drivePort: Optional[int] = None, mechPort: Optional[int] = None) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl

    def periodic(self, ds: DesiredState) -> None:
        self.desiredState.fieldSpeeds = self._calculateDrive()

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        self.desiredState.publish()

    def _calculateDrive(self) -> ChassisSpeeds:
        vx, vy = self._circularScalar(x=-self._driveCtrl.getLeftY(), y=-self._driveCtrl.getLeftX())

        omega: RPS = self._linearScalar(-self._driveCtrl.getRightX())

        return ChassisSpeeds(vx, vy, omega)
