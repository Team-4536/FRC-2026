from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, Scalar
from typing import Optional
from wpilib import XboxController as Ctrlr
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import radians_per_second as RPS
from wpilib import DigitalInput


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
        )

        self.revShooter: float = 0
        self.shootShooter: float = 0

        self.revShooter = False

    def init(
        self, drivePort: Optional[int] = None, mechPort: Optional[int] = None
    ) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl
        self._linearScalar: Scalar = Scalar(magnitude=tau)
        self._circularScalar: CircularScalar = CircularScalar(
            magnitude=self.robotState.abtainableMaxSpeed
        )

        self._isTestMode: bool = False

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

        return robotState

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
        self.robotState.fieldSpeeds = ChassisSpeeds()

    def _calculateDrive(self) -> ChassisSpeeds:
        vx, vy = self._circularScalar(
<<<<<<< HEAD
            x=-self._driveCtrl.getLeftY(), y=-self._driveCtrl.getLeftX()
=======
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
>>>>>>> 51f5352d65d38f56a1a90d17f94e92f53a6c1ba3
        )

        omega: RPS = self._linearScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
