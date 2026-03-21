from subsystems.LEDSignals import LEDSignals
from subsystems.autoSubsystem import AutoSubsystem
from subsystemManager import SubsystemManager, Subsystems
from subsystems.cameras import CameraManager
from subsystems.inputs import Inputs
from subsystems.intake import Intake

from subsystems.subsystem import RobotState
from subsystems.swerveDrive import SwerveDrive
from subsystems.tester import Tester
from subsystems.turretSystem import Shooter, Turret
from subsystems.utils import timeData
from wpilib import TimedRobot, getTime
from wpimath.units import inchesToMeters, meters, seconds
from subsystems.limelights import llCams
from ntcore import NetworkTableInstance


class Robot(TimedRobot):
    subsystemManager: SubsystemManager

    def robotInit(self) -> None:
        WHEEL_DISTANCE: meters = inchesToMeters(10.875)

        self.timeStart: seconds = getTime()
        self.timeRunnig: seconds = 0

        self.subsystemManager = SubsystemManager(
            subsystems=Subsystems(
                intake=Intake(10, 17, 9),
                ledSignals=LEDSignals(deviceID=0),
                shooter=Shooter(kickerID=18, revTopID=12, revBottomID=11),
                swerveDrive=SwerveDrive.symmetricDrive(
                    xPos=WHEEL_DISTANCE, yPos=WHEEL_DISTANCE
                ),
                turret=Turret(yawMotorID=14, pitchMotorID=13),
            ),
            inputs=Inputs(),
            autos=AutoSubsystem(),
            cameras=CameraManager(),
            time=timeData,
            tests=Tester(),
            llCam=llCams(),
            robotState=RobotState.empty(),
        )

    def robotPeriodic(self) -> None:
        self.subsystemManager.robotPeriodic()
        self.timeRunnig = getTime() - self.timeStart
        NetworkTableInstance.getDefault().getTable("telemetry").putNumber(
            "TIME RUNNING", self.timeRunnig
        )

    def autonomousInit(self) -> None:
        self.subsystemManager.init()

    def autonomousPeriodic(self) -> None:
        self.subsystemManager.autonomousPeriodic()

    def teleopInit(self) -> None:
        self.subsystemManager.init()

    def teleopPeriodic(self) -> None:
        self.subsystemManager.teleopPeriodic()

    def teleopExit(self) -> None:
        self.disabledInit()

    def testInit(self) -> None:
        self.subsystemManager.init()

    def testPeriodic(self) -> None:
        self.subsystemManager.testPeriodic()

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.subsystemManager.disabled()

    def testInit(self) -> None:
        llCams()
