from commands2 import Subsystem as WPISubsystem
from enum import Enum
from pathplannerlib.auto import (
    AutoBuilder,
    PathPlannerAuto,
    EventTrigger,
    NamedCommands,
)
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import matchData
from typing import List
from wpilib import SendableChooser, SmartDashboard
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d


class AutoSubsystem(Subsystem):
    # Declare Variables
    autoRoutineChooser: SendableChooser = SendableChooser()
    routineFinished: bool = False
    routineKeys: List[str] = list()
    currentPath: int = 0

    def __init__(self, robotState: RobotState):
        super().__init__()

        config: RobotConfig = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            pose_supplier=robotState.odometry.getEstimatedPosition,
            reset_pose=robotState.odometry.resetPose,
            robot_relative_speeds_supplier=lambda: robotState.robotRelChassisSpeeds,
            output=lambda speeds, _: self.updateFieldSpeeds(speeds, robotState),
            controller=PPHolonomicDriveController(
                PIDConstants(0.00019, 0, 0), PIDConstants(0.15, 0, 0)
            ),
            robot_config=config,
            should_flip_path=matchData.isRed,
            drive_subsystem=WPISubsystem(),  # Pass in a dummy subsystem
        )
        AUTO_FORWARD = PathPlannerAuto("Forward")
        AUTO_BACKWARD = PathPlannerAuto("Backward")

        self.autoRoutineChooser = AutoBuilder.buildAutoChooser("Forward")

        SmartDashboard.putData("Auto Routine Chooser", self.autoRoutineChooser)

    def phaseInit(self, robotState: RobotState) -> None:
        self.selectedAuto = self.autoRoutineChooser.getSelected()
        self.selectedAuto.initialize()

    def periodic(self, robotState: RobotState) -> None:
        self.selectedAuto.execute()
        if self.selectedAuto.isFinished():
            self.selectedAuto.end()

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass

    def updateFieldSpeeds(self, speeds: ChassisSpeeds, robotState: RobotState) -> None:
        robotState.fieldSpeeds = speeds.fromRobotRelativeSpeeds(speeds, robotState.gyro)
