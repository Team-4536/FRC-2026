from enum import Enum
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.autoStages import AutoStages, FollowTrajectory
import wpilib


class AutoRoutines(Enum):
    DO_NOTHING = "Do Nothing"
    DRIVE_FORWARD_TEST = "Drive Forward Test"
    DRIVE_FORWARD_BACK_TEST = "Drive Forward Back Test"


class AutoSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        AUTO_SIDE_BLUE = "blue"
        AUTO_SIDE_RED = "red"

        self.autoRoutineChooser = wpilib.SendableChooser()
        self.autoRoutineChooser.setDefaultOption(
            AutoRoutines.DRIVE_FORWARD_TEST.value,
            AutoRoutines.DRIVE_FORWARD_TEST,
        )
        for option in AutoRoutines:
            self.autoRoutineChooser.addOption(option.value, option)

        wpilib.SmartDashboard.putData("auto routine chooser", self.autoRoutineChooser)

        self.autoSideChooser = wpilib.SendableChooser()
        self.autoSideChooser.setDefaultOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        self.autoSideChooser.addOption(AUTO_SIDE_RED, AUTO_SIDE_RED)
        wpilib.SmartDashboard.putData("auto side chooser", self.autoSideChooser)

    def phaseInit(self) -> None:
        print(self.autoRoutineChooser.getSelected().value, "value to test")
        self.routine: dict[str, AutoStages] = routineChooser(
            self.autoRoutineChooser.getSelected().value,
            self.autoSideChooser.getSelected() == "red",
        )

        print(self.routine, "self.routine")

        self.currentPath = 0
        self.routineFinished = False
        self.routineKeys = list(self.routine.keys())

        # print(self.routineKeys, "&^&^&^&^&^&^&^")

        wpilib.SmartDashboard.putStringArray("routineKeys", self.routineKeys)

        if self.routine:
            self.routine[self.routineKeys[self.currentPath]].autoInit()

        # print(self.autoRoutineChooser.getSelected().value, ")(*&^%$#@!)")

    def periodic(self, robotState: RobotState) -> RobotState:  # TODO: Finish

        self.routineFinished = self.currentPath >= len(self.routineKeys)

        if not self.routineFinished:
            self.robotState = self.routine[self.routineKeys[self.currentPath]].run(
                robotState
            )
            if self.routine[self.routineKeys[self.currentPath]].isDone():
                self.currentPath += 1
                self.routineFinished = self.currentPath >= len(self.routineKeys)
                if not self.routineFinished:
                    self.routine[self.routineKeys[self.currentPath]].autoInit()

        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


def routineChooser(selectedRoutine: str, isFlipped: bool):
    routine: dict[str, AutoStages] = dict()

    wpilib.SmartDashboard.putString("routine", selectedRoutine)

    match selectedRoutine:
        case AutoRoutines.DO_NOTHING.value:
            pass
        case AutoRoutines.DRIVE_FORWARD_TEST.value:
            routine["Drive Forward Test"] = FollowTrajectory(
                "Drive Forward Test",
                isFlipped,
            )
        case AutoRoutines.DRIVE_FORWARD_BACK_TEST.value:
            routine["Forward"] = FollowTrajectory(
                "Forward",
                isFlipped,
            )
            routine["Backward"] = FollowTrajectory(
                "Backward",
                isFlipped,
            )

    return routine
