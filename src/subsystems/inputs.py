from math import tau as TAU
from subsystems.desiredState import DesiredState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, Scalar
from typing import Optional
from wpilib import XboxController as Ctrlr
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from wpilib import DigitalInput

class Inputs(Subsystem):
    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
        maxVelocity: MPS = 8,
        maxAngularVelocity: RPS = TAU,
    ) -> None:
        super().__init__()
        #self.turretSpeed: float = 0.0

        self._driveCtrl = Ctrlr(drivePort)
        self._mechCtrl = Ctrlr(mechPort)

        self._linearScalar: Scalar = Scalar(magnitude=maxAngularVelocity)
        self._circularScalar: CircularScalar = CircularScalar(magnitude=maxVelocity)

        self.desiredState = DesiredState(
            fieldSpeeds=ChassisSpeeds(), abtainableMaxSpeed=maxVelocity, turretSpeed=0, turretSetPoint=-1, motorDesiredState= 0, limitA = False, limitB = False
        
        )
        self.limitSwitch2 = DigitalInput(10) # we have to find the correct angle
        self.limitSwitch1 = DigitalInput(0)
    def init(self, drivePort: Optional[int] = None, mechPort: Optional[int] = None) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl


    def periodic(self, ds: DesiredState) -> None:
        self.desiredState.fieldSpeeds = self._calculateDrive()
        self.desiredState.turretSpeed = self._linearScalar(self._mechCtrl.getLeftX() * 30)
        self.desiredState.turretSetPoint = self._mechCtrl.getPOV()
        self.desiredState.motorDesiredState = self._linearScalar(self._mechCtrl.getRightY())
        self.desiredState.limitA = self.limitSwitch2.get()
        self.desiredState.limitB = self.limitSwitch1.get()

    def disabled(self) -> None:
        self.desiredState.fieldSpeeds = ChassisSpeeds()

    def publish(self) -> None:
        self.desiredState.publish()

    def _calculateDrive(self) -> ChassisSpeeds:
        vx, vy = self._circularScalar(x=-self._driveCtrl.getLeftY(), y=-self._driveCtrl.getLeftX())

        omega: RPS = self._linearScalar(-self._driveCtrl.getRightX())

        return ChassisSpeeds(vx, vy, omega)
