from subsystemManager import SubsystemManager, Subsystems
from subsystems.cameras import CameraManager
from subsystems.inputs import Inputs
from subsystems.intake import Intake
from subsystems.LEDSignals import LEDSignals
from subsystems.subsystem import RobotState
from subsystems.swerveDrive import SwerveDrive
from subsystems.turretSystem import Shooter, Turret
from subsystems.utils import timeData
from wpilib import TimedRobot
from wpimath.units import inchesToMeters, meters


class Robot(TimedRobot):
    subsystems: SubsystemManager

    def robotInit(self) -> None:
        WHEEL_DISTANCE: meters = inchesToMeters(10.875)

        self.subsystems = SubsystemManager(
            subsystems=Subsystems(
                intake=Intake(10, 30, 9),
                ledSignals=LEDSignals(deviceID=0),
                shooter=Shooter(kickerId=18, revTopId=12, revBottomId=11),
                swerveDrive=SwerveDrive.symmetricDrive(
                    xPos=WHEEL_DISTANCE, yPos=WHEEL_DISTANCE
                ),
                turret=Turret(yawMotorID=14, pitchMotorID=13),
            ),
            inputs=Inputs(),
            cameras=CameraManager(),
            time=timeData,
            robotState=RobotState.empty(),
        )

    def robotPeriodic(self) -> None:
        self.subsystems.robotPeriodic()

    def autonomousInit(self) -> None:
        self.subsystems.init()

    def autonomousPeriodic(self) -> None:
        self.subsystems.autonomousPeriodic()

    def teleopInit(self) -> None:
        self.subsystems.init()

    def teleopPeriodic(self) -> None:
        self.subsystems.teleopPeriodic()

    def teleopExit(self) -> None:
        self.disabledInit()

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.subsystems.disabled()

    def testPeriodic(self) -> None:
        self.subsystems.disabled()
