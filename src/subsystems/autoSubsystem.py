from enum import Enum
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import matchData
from typing import List
from wpilib import SendableChooser, SmartDashboard


class AutoSubsystem:
    # Declare Variables
    autoRoutineChooser: SendableChooser = SendableChooser()
    routineFinished: bool = False
    routineKeys: List[str] = list()
    currentPath: int = 0

    def __init__(self, robotState: RobotState):
        super().__init__()

        config: RobotConfig = RobotConfig.fromGUISettings()
        pathFlipped: bool = matchData.isRed()

        AutoBuilder.configure(
            robotState.odometry.getEstimatedPosition,
            robotState.odometry.resetPose,
            placeholder,
            placeholder,
            PPHolonomicDriveController(
                PIDConstants(0.00019, 0, 0), PIDConstants(0.15, 0, 0)
            ),
            config,
            pathFlipped,
        )

    def phaseInit(self, robotState: RobotState) -> None:
        pass

    def periodic(self, robotState: RobotState) -> None:
        pass

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass
