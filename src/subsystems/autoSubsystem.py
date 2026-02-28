from enum import Enum
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.autoStages import AutoStages, FollowTrajectory, OperateIntake
from typing import List
from subsystems.autoStages import (
    AutoStages,
    FollowTrajectory,
    OperateIntake,
    OperateTurret,
)
import wpilib


class AutoRoutines(Enum):
    DO_NOTHING = "Do Nothing"
    TEST_INTAKE = "Test Intake"
    DRIVE_FORWARD_BACK_TEST = "Drive Forward Back Test"
    WONKY = "Wonky"
    GET_BALLS_AND_BRING_BACK_RIGHT = "Get Balls And Bring Back Right"
    GET_BALLS_AND_BRING_BACK_LEFT = "Get Balls And Bring Back Left"
    FORWARD = "Forward"
    FORWARD_AND_INTAKE = "Forward And Intake"


class AutoSubsystem(Subsystem):
    # Declare Variables
    autoRoutineChooser: wpilib.SendableChooser = wpilib.SendableChooser()
    autoSideChooser: wpilib.SendableChooser = wpilib.SendableChooser()

    def __init__(self):
        super().__init__()

        AUTO_SIDE_BLUE = "blue"
        AUTO_SIDE_RED = "red"

        self.autoRoutineChooser.setDefaultOption(
            AutoRoutines.DO_NOTHING.value,
            AutoRoutines.DO_NOTHING,
        )
        for routine in AutoRoutines:
            self.autoRoutineChooser.addOption(routine.value, routine)

        self.autoSideChooser.setDefaultOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        self.autoSideChooser.addOption(AUTO_SIDE_RED, AUTO_SIDE_RED)

        wpilib.SmartDashboard.putData("auto routine chooser", self.autoRoutineChooser)

        wpilib.SmartDashboard.putData("auto side chooser", self.autoSideChooser)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        print(self.autoRoutineChooser.getSelected(), "value to test")
        self.routine: dict[str, List[AutoStages]] = routineChooser(
            self.autoRoutineChooser.getSelected(),
            self.autoSideChooser.getSelected() == "red",
        )

        print(self.routine, "self.routine")

        self.currentPath = 0
        self.routineFinished = False
        self.routineKeys = list(self.routine.keys())

        wpilib.SmartDashboard.putStringArray("routineKeys", self.routineKeys)

        if self.routine:
            for path in self.routine[self.routineKeys[self.currentPath]]:
                robotState = path.autoInit(robotState)

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:

        self.routineFinished = self.currentPath >= len(self.routineKeys)

        if not self.routineFinished:
            for path in self.routine[self.routineKeys[self.currentPath]]:
                path.run(robotState)
            if self.routine[self.routineKeys[self.currentPath]][0].isDone():
                self.currentPath += 1
                self.routineFinished = self.currentPath >= len(self.routineKeys)
                if not self.routineFinished:
                    for path in self.routine[self.routineKeys[self.currentPath]]:
                        robotState = path.autoInit(robotState)

        robotState.intakeMode = False

        robotState.intakePosYAxis = 0.0

        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


def routineChooser(
    selectedRoutine: AutoRoutines, isFlipped: bool
) -> dict[str, List[AutoStages]]:
    routine: dict[str, List[AutoStages]] = dict()
    match selectedRoutine:
        case AutoRoutines.DO_NOTHING:
            pass
        case AutoRoutines.GET_BALLS_AND_BRING_BACK_RIGHT:
            routine["Under Right Trench"] = [
                FollowTrajectory(
                    "under right trench",
                    isFlipped,
                )
            ]
            routine["Right To Balls"] = [
                FollowTrajectory(
                    "right to balls",
                    isFlipped,
                ),
                # OperateIntake(),
            ]
            routine["Back Under Left Trench"] = [
                FollowTrajectory(
                    "back under left trench",
                    isFlipped,
                )
            ]

        case AutoRoutines.GET_BALLS_AND_BRING_BACK_LEFT:
            routine["Under Left Trench"] = [
                FollowTrajectory(
                    "under left trench",
                    isFlipped,
                )
            ]
            routine["Left To Balls"] = [
                FollowTrajectory(
                    "left to balls",
                    isFlipped,
                ),
                # OperateIntake(),
            ]
            routine["Back Under Right Trench"] = [
                FollowTrajectory(
                    "back under right trench",
                    isFlipped,
                )
            ]

        case AutoRoutines.FORWARD:
            routine["Forward"] = [
                FollowTrajectory(
                    "Drive Forward Test",
                    isFlipped,
                )
            ]
        case AutoRoutines.TEST_INTAKE:
            routine["Drop & Run Intake"] = [OperateIntake(10.5)]
        case AutoRoutines.DRIVE_FORWARD_BACK_TEST:
            routine["Forward"] = [
                FollowTrajectory(
                    "Forward",
                    isFlipped,
                ),
            ]
            routine["Backward"] = [
                FollowTrajectory(
                    "Backward",
                    isFlipped,
                )
            ]
        case AutoRoutines.WONKY:
            routine["Wonky 1"] = [
                FollowTrajectory(
                    "Wonky 1",
                    isFlipped,
                )
            ]
            routine["Wonky 2"] = [
                FollowTrajectory(
                    "Wonky 2",
                    isFlipped,
                )
            ]
        case AutoRoutines.FORWARD_AND_INTAKE:
            routine["Forward"] = [
                FollowTrajectory(
                    "Drive Forward Test",
                    isFlipped,
                ),
                OperateIntake(5),
            ]

    return routine
