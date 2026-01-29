from enum import Enum
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.autoStages import AutoStages
import wpilib


class AutoRoutines(Enum):
    DO_NOTHING = "Do Nothing"
    DRIVE_FORWARD_TEST = "Drive Forward Test"


class AutoSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        AUTO_SIDE_BLUE = "blue"
        AUTO_SIDE_RED = "red"

        self.autoRoutineChooser = wpilib.SendableChooser()
        self.autoRoutineChooser.setDefaultOption(
            AutoRoutines.DO_NOTHING.value,
            AutoRoutines.DO_NOTHING.value,
        )
        for option in AutoRoutines:
            self.autoRoutineChooser.addOption(option.value, option.value)

        wpilib.SmartDashboard.putData("auto routine chooser", self.autoRoutineChooser)

        self.autoSideChooser = wpilib.SendableChooser()
        self.autoSideChooser.setDefaultOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        self.autoSideChooser.addOption(AUTO_SIDE_RED, AUTO_SIDE_RED)
        wpilib.SmartDashboard.putData("auto side chooser", self.autoSideChooser)

    def init(self) -> None:
        self.routine: dict[str, AutoStages] = routineChooser(
            self.autoRoutineChooser.getSelected(),
            self.autoSideChooser.getSelected() == "blue",
        )

        self.currentPath = 0
        self.routineFinished = False
        self.autoKeys = list(self.routine)

        if self.routine:
            self.routine[self.autoKeys[self.currentPath]].autoInit()

    def periodic(self, robotState: RobotState) -> RobotState:  # TODO: Finish

        self.routineFinished = self.currentPath >= len(self.autoKeys)

        if not self.routineFinished:
            robotState = self.routine[self.autoKeys[self.currentPath]].run(robotState)
            if self.routine[self.autoKeys[self.currentPath]].isDone(robotState):
                self.currentPath += 1
                self.routineFinished = self.currentPath >= len(self.autoKeys)
                if not self.routineFinished:
                    self.routine[self.autoKeys[self.currentPath]].autoInit()

        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


def routineChooser(selectedRoutine: str, isFlipped: bool):
    routine: dict[str, AutoStages] = dict()

    if selectedRoutine == AutoRoutines.DO_NOTHING:
        pass

    return routine
