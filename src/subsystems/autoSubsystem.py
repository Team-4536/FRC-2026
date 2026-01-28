from enum import Enum
from subsystems.subsystem import Subsystem
import wpilib

class AutoRoutines(Enum):
    DO_NOTHING = "Do Nothing"

class AutoSubsystem(Subsystem):
    def __init__(self):

        super().__init__()
        AUTO_SIDE_RED = "red"
        AUTO_SIDE_BLUE = "blue"

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
