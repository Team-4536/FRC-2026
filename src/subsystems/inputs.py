from math import tau as TAU
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, Scalar
from typing import Optional
from wpilib import XboxController as Ctrlr
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from math import pi as PI


class Inputs(Subsystem):
    def __init__(
        self,
        drivePort: int = 0,
        mechPort: int = 1,
        maxAngularVelocity: RPS = TAU,
    ) -> None:
        super().__init__()
        # self.turretSpeed: float = 0.0

        self._driveCtrl = Ctrlr(drivePort)
        self._mechCtrl = Ctrlr(mechPort)

        self.robotState = RobotState.empty(abtainableMaxSpeed=5)

        self._linearScalar: Scalar = Scalar(magnitude=maxAngularVelocity)
        self._circularScalar: CircularScalar = CircularScalar(
            magnitude=self.robotState.abtainableMaxSpeed
        )

        self.revShooter: float = 0
        self.shootShooter: float = 0

        self.revShooter = False

    def init(
        self, drivePort: Optional[int] = None, mechPort: Optional[int] = None
    ) -> None:
        self._driveCtrl = Ctrlr(drivePort) if drivePort else self._driveCtrl
        self._mechCtrl = Ctrlr(mechPort) if mechPort else self._mechCtrl

    def periodic(self) -> None:
        self.robotState.fieldSpeeds = self._calculateDrive()
        self.robotState.turretSpeed = self._linearScalar(self._mechCtrl.getLeftX() * 30)
        self.robotState.motorDesiredState = self._linearScalar(
            self._mechCtrl.getRightY()
        )
        self.robotState.turretSetPoint = (3 * PI) / 2 / 2
        self.robotState.motorDesiredState = self._linearScalar(
            self._mechCtrl.getRightY()
        )
        self.robotState.turretManualToggle = self._mechCtrl.getYButtonPressed() # TODO assign button for manual toggle
        self.robotState.turretManualSetpoint = self._mechCtrl.getPOV()

    def periodic(self, robotState: RobotState) -> RobotState:
        self.robotState.fieldSpeeds = self._calculateDrive()
        self.robotState.revShooter = self._mechCtrl.getRightTriggerAxis()
        self.robotState.kickShooter = self._mechCtrl.getRightBumper()
        return robotState

    def disabled(self) -> None:
        self.robotState.fieldSpeeds = ChassisSpeeds()

    def publish(self) -> None:
        self.robotState.publish()

    def _calculateDrive(self) -> ChassisSpeeds:
        vx, vy = self._circularScalar(
            x=-self._driveCtrl.getLeftY(), y=-self._driveCtrl.getLeftX()
        )

        omega: RPS = self._linearScalar(-self._driveCtrl.getRightX())

        return ChassisSpeeds(vx, vy, omega)
