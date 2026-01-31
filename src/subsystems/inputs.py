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

        self._driveCtrl = Ctrlr(drivePort)
        self._mechCtrl = Ctrlr(mechPort)

        self._linearScalar: Scalar = Scalar(magnitude=maxAngularVelocity)
        self._circularScalar: CircularScalar = CircularScalar(magnitude=maxVelocity)

        #climbing 
        self.limitSwitch = DigitalInput(1) #we need to find the right number
        self.limitSwitchCon = DigitalInput(2) #^^

        #turret 
        self.limitLeftTur = DigitalInput(4) #^^
        self.limtiRightTur = DigitalInput(3) #^^
    

        self.desiredState = DesiredState(
            fieldSpeeds=ChassisSpeeds(),
            abtainableMaxSpeed=maxVelocity,
            revMotor=0,
            extended=False,
            contracted=True,
            buttonDown=False,
            buttonUp=False,
            buttonLeft=False,
            buttonRight=False,
            limitLeft=False,
            limitRight=False
        )

        self.revShooter: float = 0
        self.shootShooter: float = 0

        self.revShooter = False

    def init(
        self, drivePort: Optional[int] = None, mechPort: Optional[int] = None
    ) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl

    def periodic(self, ds: DesiredState) -> None:
        self.desiredState.fieldSpeeds = self._calculateDrive()

        self.desiredState.revMotor = self._mechCtrl.getRightTriggerAxis()

        #climbing 
        self.desiredState.extended = self.limitSwitch.get()
        self.desiredState.contracted = self.limitSwitchCon.get()
        self.desiredState.buttonUp = self._driveCtrl.getYButton
        self.desiredState.buttonDown = self._driveCtrl.getAButton()

        #turret
        self.desiredState.buttonLeft = self._driveCtrl.getLeftStickButton() 
        self.desiredState.buttonRight = self._driveCtrl.getLeftStickButton()
        self.desiredState.limitLeft = self.limitLeftTur.get()
        self.desiredState.limitRight = self.limtiRightTur.get()

    def disabled(self) -> None:
        self.desiredState.fieldSpeeds = ChassisSpeeds()

    def publish(self) -> None:
        self.desiredState.publish()

    def _calculateDrive(self) -> ChassisSpeeds:
        vx, vy = self._circularScalar(
            x=-self._driveCtrl.getLeftY(), y=-self._driveCtrl.getLeftX()
        )

        omega: RPS = self._linearScalar(-self._driveCtrl.getRightX())

        return ChassisSpeeds(vx, vy, omega)
