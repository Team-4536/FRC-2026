from subsystemManager import SubsystemManager, Subsystems
from subsystems.autoSubsystem import AutoSubsystem
from subsystems.cameras import CameraManager
from subsystems.climber import Climber
from subsystems.inputs import Inputs
from subsystems.intake import Intake
from subsystems.limelights import llCams
from subsystems.subsystem import RobotState
from subsystems.swerveDrive import SwerveDrive
from subsystems.tester import Tester
from subsystems.turretSystem import Shooter, Turret
from subsystems.utils import timeData
from wpilib import TimedRobot
from wpimath.units import inchesToMeters, meters


class Robot(TimedRobot):
    subsystemManager: SubsystemManager

    def robotInit(self) -> None:
        WHEEL_DISTANCE: meters = inchesToMeters(10.875)
        robotState = RobotState.empty()

        self.subsystemManager = SubsystemManager(
            subsystems=Subsystems(
                intake=Intake(10, 17, 9),
                shooter=Shooter(kickerID=18, revTopID=12, revBottomID=11),
                swerveDrive=SwerveDrive.symmetricDrive(
                    FL_DriveID=2,
                    FR_DriveID=4,
                    BL_DriveID=6,
                    BR_DriveID=8,
                    FL_AzimuthID=1,
                    FR_AzimuthID=3,
                    BL_AzimuthID=5,
                    BR_AzimuthID=7,
                    FL_EncoderID=21,
                    FR_EncoderID=22,
                    BL_EncoderID=23,
                    BR_EncoderID=24,
                    xPos=WHEEL_DISTANCE,
                    yPos=WHEEL_DISTANCE,
                ),
                turret=Turret(yawMotorID=14, pitchMotorID=13),
                climb=Climber(motorID=15),
            ),
            inputs=Inputs(),
            autos=AutoSubsystem(robotState=robotState),
            cameras=CameraManager(),
            time=timeData,
            tests=Tester(),
            llCam=llCams(),
            robotState=robotState,
        )

    def robotPeriodic(self) -> None:
        self.subsystemManager.robotPeriodic()

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
