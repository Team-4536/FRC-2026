from enum import Enum
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.autoStages import AutoStages, FollowTrajectory
import wpilib


class AutoRoutines(Enum):
    DO_NOTHING = "Do Nothing"
    DRIVE_FORWARD_TEST = "Drive Forward Test"
    DRIVE_FORWARD_BACK_TEST = "Drive Forward Back Test"
    WONKY = "Wonky"


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
        self.routine: dict[str, AutoStages] = routineChooser(
            self.autoRoutineChooser.getSelected(),
            self.autoSideChooser.getSelected() == "red",
        )

        print(self.routine, "self.routine")

        self.currentPath = 0
        self.routineFinished = False
        self.routineKeys = list(self.routine.keys())

        wpilib.SmartDashboard.putStringArray("routineKeys", self.routineKeys)

        if self.routine:
            self.routine[self.routineKeys[self.currentPath]].autoInit()

    def periodic(self, robotState: RobotState) -> RobotState:

        self.routineFinished = self.currentPath >= len(self.routineKeys)

        if not self.routineFinished:
            robotState = self.routine[self.routineKeys[self.currentPath]].run(
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


def routineChooser(selectedRoutine: AutoRoutines, isFlipped: bool):
    routine: dict[str, AutoStages] = dict()

    match selectedRoutine:
        case AutoRoutines.DO_NOTHING:
            pass
        case AutoRoutines.DRIVE_FORWARD_TEST:
            routine["Drive Forward Test"] = FollowTrajectory(
                "Drive Forward Test",
                isFlipped,
            )
        case AutoRoutines.DRIVE_FORWARD_BACK_TEST:
            routine["Forward"] = FollowTrajectory(
                "Forward",
                isFlipped,
            )
            routine["Backward"] = FollowTrajectory(
                "Backward",
                isFlipped,
            )
        case AutoRoutines.WONKY:
            routine["Wonky 1"] = FollowTrajectory(
                "Wonky 1",
                isFlipped,
            )
            routine["Wonky 2"] = FollowTrajectory(
                "Wonky 2",
                isFlipped,
            )

    return routine
