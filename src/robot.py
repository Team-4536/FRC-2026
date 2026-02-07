from subsystemManager import SubsystemManager
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from subsystems.intake import Intake
from subsystems.motor import RevMotor
from wpilib import TimedRobot


class Robot(TimedRobot):
    def robotInit(self) -> None:
        self.subsystems: SubsystemManager = SubsystemManager(
            inputs=Inputs(),
            ledSignals=LEDSignals(deviceID=0),
            swerveDrive=SwerveDrive(),
            time=TimeData(),
            intake=Intake(10, 99, 9),
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
