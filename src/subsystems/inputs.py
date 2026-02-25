from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, lerp, Scalar
from wpilib import XboxController
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second


class Inputs(Subsystem):
    LOW_MAX_ABTAINABLE_SPEED: meters_per_second = 2
    MAX_ABTAINABLE_SPEED: meters_per_second = 5

    def __init__(self, drivePort: int = 0, mechPort: int = 1) -> None:
        super().__init__()

        self._driveCtrlr = XboxController(drivePort)
        self._mechCtrlr = XboxController(mechPort)

        self._linearScalar: Scalar = Scalar(magnitude=tau)
        self._circularScalar: CircularScalar = CircularScalar(
            magnitude=self.LOW_MAX_ABTAINABLE_SPEED
        )

    def phaseInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        # Drive Controls
        maxSpeed = lerp(
            self.LOW_MAX_ABTAINABLE_SPEED,
            self.MAX_ABTAINABLE_SPEED,
            max(self._driveCtrlr.getRightTriggerAxis() / 0.9, 1.0),
        )
        robotState.fieldSpeeds = self._calculateDrive(maxSpeed)
        robotState.resetGyro = self._driveCtrlr.getStartButtonPressed()

        return robotState

    def disabled(self) -> None:
        pass

    def _calculateDrive(self, maxSpeed: meters_per_second) -> ChassisSpeeds:
        self._circularScalar.setMagnitude(maxSpeed)
        vx, vy = self._circularScalar(
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
        )

        omega = self._linearScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
