from math import pi as PI
from mb.desiredState import DesiredState
from mb.subsystem import Subsystem
from mb.utils import Scalar
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
        maxVelocity: MPS = 6,
        maxAngularVelocity: RPS = PI,
    ) -> None:
        super().__init__()

        self._driveCtrl = Ctrlr(drivePort)
        self._mechCtrl = Ctrlr(mechPort)
        self._maxVelocity = maxVelocity
        self._maxAngularVelocity = maxAngularVelocity

        self._scalar: Scalar = Scalar()

        self.desiredState = DesiredState(fieldSpeeds=ChassisSpeeds())

    def init(self, drivePort: Optional[int] = None, mechPort: Optional[int] = None) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl

    def periodic(self, ds: DesiredState) -> None:
        self.desiredState.fieldSpeeds = self._calculateDrive()

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass

    def _calculateDrive(self) -> ChassisSpeeds:
        x: float = self._scalar.scale(0 - self._driveCtrl.getLeftY())
        y: float = self._scalar.scale(0 - self._driveCtrl.getLeftX())

        vx: MPS = x * self._maxVelocity
        vy: MPS = y * self._maxVelocity

        rot: float = self._scalar.scale(0 - self._driveCtrl.getRightX())
        omega: RPS = rot * self._maxAngularVelocity

        return ChassisSpeeds(vx, vy, omega)
