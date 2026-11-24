from commands2 import Subsystem as WPISubsystem, cmd, Command
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

# class AutoIntakeCommand(Command):
#     def initialize(self):
#         return super().initialize()

#     def execute(self):
#         return super().execute()

#     def end(self, interrupted: bool):
#         return super().end(interrupted)


class SubsystemUpdater:
    def __init__(
        self,
        robotState: RobotState,
        runIntake: bool | None = None,
        lowerIntake: bool | None = None,
        revShooter: bool | None = None,
        shootShooter: bool | None = None,
    ) -> None:
        self.robotState = robotState
        self.runIntake = runIntake
        self.lowerIntake = lowerIntake
        self.shootShooter = shootShooter
        self.revShooter = revShooter

    def update(self) -> None:
        if self.runIntake is not None:
            self.robotState.initialIntake = self.runIntake
            self.robotState.intakeIndexer = self.runIntake

        if self.lowerIntake is not None:
            self.robotState.intakePosYAxis = 0.85 * self.lowerIntake

        if self.revShooter is not None:
            self.robotState.revSpeed = self.revShooter
            print("revving")

        if self.shootShooter is not None:
            self.robotState.kickShooter = self.shootShooter
            self.robotState.intakeIndexer = self.shootShooter


class AutoSubsystem(Subsystem):
    # Declare Variables
    autoRoutineChooser: SendableChooser = SendableChooser()
    routineFinished: bool = False
    routineKeys: List[str] = list()
    currentPath: int = 0
    isDone: bool = False
    selectedAuto: Command

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

        self.autoRoutineChooser: SendableChooser = AutoBuilder.buildAutoChooser(
            "Forward"
        )

        SmartDashboard.putData("Auto Routine Chooser", self.autoRoutineChooser)

        EventTrigger("rev_shooter").whileTrue(
            cmd.print_("||| ")
            # cmd.runEnd(
            #     SubsystemUpdater(robotState, revShooter=True).update,
            #     SubsystemUpdater(robotState, revShooter=False).update,
            # )
        )
        EventTrigger("shoot_shooter").whileTrue(
            cmd.runEnd(
                SubsystemUpdater(robotState, shootShooter=True).update,
                SubsystemUpdater(robotState, shootShooter=False).update,
            )
        )
        EventTrigger("lower_intake").onTrue(
            cmd.runOnce(SubsystemUpdater(robotState, lowerIntake=True).update).andThen(
                cmd.waitSeconds(1).andThen(
                    cmd.runOnce(SubsystemUpdater(robotState, lowerIntake=False).update)
                )
            )
        )
        EventTrigger("run_intake").whileTrue(
            cmd.runEnd(
                SubsystemUpdater(robotState, runIntake=True).update,
                SubsystemUpdater(robotState, runIntake=False).update,
            )
        )

    def phaseInit(self, robotState: RobotState) -> None:
        self.selectedAuto = self.autoRoutineChooser.getSelected()
        self.selectedAuto.initialize()

    def periodic(self, robotState: RobotState) -> None:
        if self.selectedAuto.isFinished():
            self.isDone = True
            self.selectedAuto.end(interrupted=False)
            SubsystemUpdater(
                robotState,
                runIntake=False,
                lowerIntake=False,
                revShooter=False,
                shootShooter=False,
            ).update

        if not self.isDone:
            self.selectedAuto.execute()

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass

    def updateFieldSpeeds(self, speeds: ChassisSpeeds, robotState: RobotState) -> None:
        robotState.fieldSpeeds = speeds.fromRobotRelativeSpeeds(speeds, robotState.gyro)

    # def updateSubsystems(
    #     self,
    #     robotState: RobotState,
    #     runIntake: bool | None = None,
    #     lowerIntake: bool | None = None,
    #     revShooter: bool | None = None,
    #     shootShooter: bool | None = None,
    # ) -> None:
    #     if not runIntake == None:
    #         robotState.initialIntake = runIntake
    #         robotState.intakeIndexer = runIntake

    #     if not lowerIntake == None:
    #         robotState.intakePosYAxis = 0.85 * lowerIntake

    #     if not revShooter == None:
    #         robotState.revSpeed = revShooter

    #     if not shootShooter == None:
    #         robotState.kickShooter = shootShooter
    #         robotState.intakeIndexer = shootShooter


# def runIntake(robotState: RobotState, runIntake: bool) -> None:
#     robotState.initialIntake = runIntake
#     robotState.intakeIndexer = runIntake


# def lowerIntake(robotState: RobotState, lowerIntake: bool) -> None:
#     robotState.intakePosYAxis = 0.85 * lowerIntake


# def revShooter(robotState: RobotState, revShooter: bool) -> None:
#     robotState.revSpeed = revShooter


# def shootShooter(robotState: RobotState, shootShooter: bool) -> None:
#     robotState.kickShooter = shootShooter
#     robotState.intakeIndexer = shootShooter


#     class IntakeUpdater:
#     def __init__(
#         self, robotState: RobotState, runIntake: bool = False, lowerIntake: bool = False
#     ) -> None:
#         self.robotState = robotState
#         self.runIntake = runIntake
#         self.lowerIntake = lowerIntake

#     def execute(self) -> None:
#         self.robotState.initialIntake = self.runIntake
#         self.robotState.intakePosYAxis = 0.85 * self.lowerIntake

# class ShooterUpdater:
#     def __init__(
#         self,
#         robotState: RobotState,
#         revShooter: bool = False,
#         kickShooter: bool = False,
#     ) -> None:
#         self.robotState = robotState
#         self.revShooter = revShooter
#         self.kickShooter = kickShooter

#     def execute(self) -> None:
#         self.robotState.revSpeed = self.revShooter
#         self.robotState.kickShooter = self.kickShooter
