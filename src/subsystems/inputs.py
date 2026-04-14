from math import tau
from subsystems.robotState import ClimberState, RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import CircularScalar, lerp, Scalar
from wpilib import XboxController
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second


class Inputs(Subsystem):
    LOW_MAX_ABTAINABLE_SPEED: meters_per_second = 1.5
    MAX_ABTAINABLE_SPEED: meters_per_second = 5

    _driveCtrlr: XboxController
    _mechCtrlr: XboxController

    _linearDriveScalar: Scalar
    _circularDriveScalar: CircularScalar
    _linearScalar: Scalar

    def __init__(self, drivePort: int = 0, mechPort: int = 1) -> None:
        super().__init__()

        self._driveCtrlr = XboxController(drivePort)
        self._mechCtrlr = XboxController(mechPort)

        self.polarity = True

        self._linearDriveScalar = Scalar(magnitude=tau)
        self._circularDriveScalar = CircularScalar(
            magnitude=self.LOW_MAX_ABTAINABLE_SPEED
        )
        self._linearScalar = Scalar()

        self.polarity: bool = True

        self.publishFloat("proxy_drive_x", 0)
        self.publishFloat("proxy_drive_y", 0)
        self.publishBoolean("proxy_control_mode", False)

    def phaseInit(self, robotState: RobotState) -> None:
        robotState.resetGyro = True

    def periodic(self, robotState: RobotState) -> None:
        # Drive Controls
        maxSpeed = lerp(
            self.LOW_MAX_ABTAINABLE_SPEED,
            self.MAX_ABTAINABLE_SPEED,
            min(1.0, self._driveCtrlr.getRightTriggerAxis() / 0.9),
        )
        self.polarity = (
            not self.polarity if self._driveCtrlr.getXButtonPressed() else self.polarity
        )
        robotState.fieldSpeeds = self._calculateDrive(maxSpeed)
        robotState.resetGyro = self._driveCtrlr.getStartButtonPressed()

        # Climb Controls
        if self._driveCtrlr.getYButton():
            robotState.climbState = ClimberState.CLIMB_UP
        elif self._driveCtrlr.getAButton():
            robotState.climbState = ClimberState.CLIMB_DOWN
        else:
            robotState.climbState = ClimberState.DISABLED

        # Turret Controls
        robotState.kickerEject = (
            self._mechCtrlr.getBButton() or self._mechCtrlr.getLeftTriggerAxis() > 0.3
        )
        robotState.kickShooter = self._mechCtrlr.getRightBumper()
        robotState.revSpeed = self._mechCtrlr.getRightTriggerAxis()
        robotState.turretManualSetpoint = self._mechCtrlr.getPOV()
        robotState.turretSwitchMode = self._mechCtrlr.getYButtonPressed()
        robotState.turretSwitchTarget = self._mechCtrlr.getXButtonPressed()

        # Intake Controls
        robotState.indexerEject = (
            self._mechCtrlr.getBButton() or self._mechCtrlr.getLeftTriggerAxis() > 0.3
        )
        robotState.initialIntake = self._mechCtrlr.getAButton()
        robotState.intakeEject = self._mechCtrlr.getBButton()
        robotState.intakeEject = self._mechCtrlr.getLeftTriggerAxis() > 0.3
        robotState.intakeIndexer = self._mechCtrlr.getRightBumper()
        robotState.intakeModeLeftBumperPressed = self._mechCtrlr.getLeftBumperPressed()
        robotState.intakePos = self._mechCtrlr.getBackButtonPressed()
        robotState.intakePosYAxis = self._mechCtrlr.getRightY()

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass

    def _calculateDrive(self, maxSpeed: meters_per_second) -> ChassisSpeeds:
        self._circularDriveScalar.setMagnitude(maxSpeed)

        # if not self.getBoolean("proxy_control_mode", default=False):
        vx, vy = self._circularDriveScalar(
            x=-self._driveCtrlr.getLeftY(), y=-self._driveCtrlr.getLeftX()
        )
        if not self.polarity:
            vx, vy = -vx, -vy
        # else:
        #     vx, vy = self._circularDriveScalar(
        #         x=self.getFloat("proxy_drive_x", default=0),
        #         y=self.getFloat("proxy_drive_y", default=0),
        #     )

        omega = self._linearDriveScalar(-self._driveCtrlr.getRightX())

        return ChassisSpeeds(vx, vy, omega)
