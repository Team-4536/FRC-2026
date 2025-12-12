from math import pi as PI
from mb.controls import Ctrlr
from mb.desiredState import DesiredState
from mb.subsystem import Subsystem
from mb.utils import Scalar
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS


class Inputs(Subsystem, DesiredState):
    driveCtrl: Ctrlr
    mechCtrl: Ctrlr

    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
        maxVelocity: MPS = 3,
        maxAngularVelocity: RPS = PI,
    ) -> None:
        super().__init__()

        self.driveCtrl = Ctrlr(drivePort)
        self.mechCtrl = Ctrlr(mechPort)
        self.maxVelocity = maxVelocity
        self.maxAngularVelocity = maxAngularVelocity

        self.scalar: Scalar = Scalar()
        self.fieldSpeeds: ChassisSpeeds = ChassisSpeeds(vx=0, vy=0, omega=0)

    def init(self, drivePort: int = None, mechPort: int = None) -> None:
        self.driveCtrl = Ctrlr(drivePort) if drivePort else self.driveCtrl
        self.mechCtrl = Ctrlr(mechPort) if mechPort else self.mechCtrl

        self.periodic()

    def periodic(self) -> None:
        x: float = self.scalar(self.driveCtrl.getRightX())
        y: float = self.scalar(self.driveCtrl.getRightY())

        vx: MPS = x * self.maxVelocity
        vy: MPS = y * self.maxVelocity

        rot: float = self.scalar(self.driveCtrl.getLeftX())
        omega: RPS = rot * self.maxAngularVelocity

        self.fieldSpeeds = ChassisSpeeds(vx, vy, omega)
