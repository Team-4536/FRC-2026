from subsystemManager import SubsystemManager
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.turretSystem import Shooter, Turret
from subsystems.motor import RevMotor
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from subsystems.autoSubsystem import AutoSubsystem
from subsystems.intake import Intake
from wpilib import TimedRobot
import wpilib


class Robot(TimedRobot):
    def robotInit(self) -> None:
        self.subsystems: SubsystemManager = SubsystemManager(
            inputs=Inputs(),
            ledSignals=LEDSignals(deviceID=0),
            swerveDrive=SwerveDrive(),
            time=TimeData(),
            autos=AutoSubsystem(),
            turret=Turret(yawMotorID=14, pitchMotorID=13),
            shooter=Shooter(kickerId=18, revTopId=12, revBottomId=11),
            intake=Intake(10, 30, 9),
            cameras=CameraManager(),
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
