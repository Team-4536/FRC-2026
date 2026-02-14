from enum import Enum
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.autoStages import AutoStages, FollowTrajectory, OperateIntake
from typing import List
import wpilib


class AutoRoutines(Enum):
    DO_NOTHING = "Do Nothing"
    DRIVE_FORWARD_TEST = "Drive Forward Test"
    DRIVE_FORWARD_BACK_TEST = "Drive Forward Back Test"
    WONKY = "Wonky"
    UNDER_RIGHT_TRENCH = "Under Right Trench"
    UNDER_LEFT_TRENCH = "Under Left Trench"
    RIGHT_TO_BALLS = "Right To Balls"
    LEFT_TO_BALLS = "Left To Balls"
    BACK_UNDER_RIGHT_TRENCH = "Back Under Right Trench"
    BACK_UNDER_LEFT_TRENCH = "Back Under Left Trench"


class AutoSubsystem(Subsystem):
    # Declare Variables
    autoRoutineChooser: wpilib.SendableChooser = wpilib.SendableChooser()
    autoSideChooser: wpilib.SendableChooser = wpilib.SendableChooser()

    def __init__(self):
        super().__init__()

        AUTO_SIDE_BLUE = "blue"
        AUTO_SIDE_RED = "red"

        self.autoRoutineChooser.setDefaultOption(
            AutoRoutines.DRIVE_FORWARD_TEST.value,
            AutoRoutines.DRIVE_FORWARD_TEST,
        )
        for routine in AutoRoutines:
            self.autoRoutineChooser.addOption(routine.value, routine)

        self.autoSideChooser.setDefaultOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        self.autoSideChooser.addOption(AUTO_SIDE_RED, AUTO_SIDE_RED)

        wpilib.SmartDashboard.putData("auto routine chooser", self.autoRoutineChooser)

        wpilib.SmartDashboard.putData("auto side chooser", self.autoSideChooser)

    def phaseInit(self) -> None:
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
                path.autoInit()

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
                        path.autoInit()

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
        case AutoRoutines.DRIVE_FORWARD_TEST:
            routine["Drive Forward Test"] = [
                FollowTrajectory(
                    "Drive Forward Test",
                    isFlipped,
                )
            ]

        case AutoRoutines.UNDER_RIGHT_TRENCH:
            routine["Under Right Trench"] = [
                FollowTrajectory(
                    "under right trench",
                    isFlipped,
                )
            ]
        case AutoRoutines.UNDER_LEFT_TRENCH:
            routine["Under Left Trench"] = [
                FollowTrajectory(
                    "under left trench",
                    isFlipped,
                )
            ]

        case AutoRoutines.RIGHT_TO_BALLS:
            routine["Right To Balls"] = [
                FollowTrajectory(
                    "right to balls",
                    isFlipped,
                )
            ]
        case AutoRoutines.LEFT_TO_BALLS:
            routine["Right To Balls"] = [
                FollowTrajectory(
                    "right to balls",
                    isFlipped,
                )
            ]

        case AutoRoutines.BACK_UNDER_RIGHT_TRENCH:
            routine["Back Under Right Trench"] = [
                FollowTrajectory(
                    "back under right trench",
                    isFlipped,
                )
            ]
        case AutoRoutines.BACK_UNDER_LEFT_TRENCH:
            routine["Back Under Left Trench"] = [
                FollowTrajectory(
                    "back under left trench",
                    isFlipped,
                )
            ]

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

    return routine
