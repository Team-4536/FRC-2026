from math import tau
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, lerp, Scalar

from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second
from wpilib import Joystick


class Inputs(Subsystem):
    LOW_MAX_ABTAINABLE_SPEED: meters_per_second = 1.5
    MAX_ABTAINABLE_SPEED: meters_per_second = 5

    _leftCtrlr: Joystick
    _rightCtrlr: Joystick

    _linearDriveScalar: Scalar
    _circularDriveScalar: CircularScalar
    _linearScalar: Scalar

    def __init__(self, drivePort: int = 0, mechPort: int = 1) -> None:
        super().__init__()

        self._leftCtrlr = Joystick(drivePort)
        self._rightCtrlr = Joystick(mechPort)

        self.polarity = True

        self._linearDriveScalar = Scalar(magnitude=tau)
        self._circularDriveScalar = CircularScalar(
            magnitude=self.LOW_MAX_ABTAINABLE_SPEED
        )
        self._linearScalar = Scalar()

        self.polarity: bool = True

        self.proxyControlMode: bool = False

        self.publishFloat("proxy drive x", 0)
        self.publishFloat("proxy drive y", 0)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        robotState.resetGyro = True

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        # Drive Controls
        # self.proxyControlMode = self.getBoolean("proxy control mode", default=False)

        maxSpeed = lerp(
            self.LOW_MAX_ABTAINABLE_SPEED,
            self.MAX_ABTAINABLE_SPEED,
            min(1.0, self._leftCtrlr.getRawAxis(4) / 0.9),
        )
        robotState.fieldSpeeds = self._calculateDrive(maxSpeed)
        robotState.resetGyro = self._rightCtrlr.getRawButtonPressed(5)
        self.polarity = (
            not self.polarity
            if self._rightCtrlr.getRawButtonPressed(6)
            else self.polarity
        )

        # Climb Controls
        robotState.climbUp = self._leftCtrlr.getRawButton(10)
        robotState.climbDown = self._leftCtrlr.getRawButtonPressed(12)

        # Turret Controls
        robotState.turretSwitchMode = self._leftCtrlr.getRawButtonPressed(5)
        robotState.turretManualSetpoint = self._leftCtrlr.getPOV()
        robotState.turretSwitchTarget = self._leftCtrlr.getRawButtonPressed(3)
        robotState.revSpeed = self._rightCtrlr.getRawButton(1)
        robotState.kickShooter = self._leftCtrlr.getRawButton(1)

        # Intake Controls
        robotState.initialIntake = self._rightCtrlr.getRawButton(3)
        robotState.intakeIndexer = self._rightCtrlr.getRawButton(7)
        robotState.intakeEject = self._leftCtrlr.getRawButton(3)
        robotState.indexerEject = self._leftCtrlr.getRawButton(4)
        robotState.intakePosYAxis = self._rightCtrlr.getPOV()
        robotState.intakeModeLeftBumperPressed = self._rightCtrlr.getRawButtonPressed(7)
        robotState.ejectAll = self._rightCtrlr.getRawButton(4)
        robotState.intakePos = self._rightCtrlr.getRawButtonPressed(11)

        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        self.publishBoolean("proxy control mode", self.proxyControlMode)
        pass

    def _calculateDrive(self, maxSpeed: meters_per_second) -> ChassisSpeeds:
        self._circularDriveScalar.setMagnitude(maxSpeed)

        if not self.proxyControlMode:
            vx, vy = self._circularDriveScalar(
                x=-self._rightCtrlr.getRawAxis(1), y=-self._leftCtrlr.getRawAxis(2)
            )
            if not self.polarity:
                vx, vy = -vx, -vy
        else:
            # vx, vy = self._circularDriveScalar(
            #     x=self.getFloat("proxy drive x", default=0),
            #     y=self.getFloat("proxy drive y", default=0),
            # )
            vx, vy = 0, 0

        omega = self._linearDriveScalar(-self._rightCtrlr.getRawAxis(3))

        return ChassisSpeeds(vx, vy, omega)
