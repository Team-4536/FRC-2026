from controls import Ctrlr
from math import pi as PI
from subsystem import Subsystem
from utils import Scalar
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS


class Inputs(Subsystem):
    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
        maxVelocity: MPS = 8,
        maxAngularVelocity: RPS = PI,
    ) -> None:
        super().__init__()

        self._driveCtrl = Ctrlr(drivePort)
        self._mechCtrl = Ctrlr(mechPort)
        self._maxVelocity = maxVelocity
        self._maxAngularVelocity = maxAngularVelocity

        self._scalar: Scalar = Scalar()
        self._fieldSpeeds: ChassisSpeeds = ChassisSpeeds(vx=0, vy=0, omega=0)

    def init(self, drivePort: int = None, mechPort: int = None) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl

        self.periodic()

    def periodic(self) -> None:
        x: float = self._scalar.scale(0 - self._driveCtrl.getRightX())
        y: float = self._scalar.scale(0 - self._driveCtrl.getRightY())

        vx: MPS = x * self._maxVelocity
        vy: MPS = y * self._maxVelocity

        rot: float = self._scalar.scale(self._driveCtrl.getLeftX())
        omega: RPS = rot * self._maxAngularVelocity

        self._fieldSpeeds = ChassisSpeeds(vx, vy, omega)

    @property
    def fieldSpeeds(self):
        return self._fieldSpeeds
