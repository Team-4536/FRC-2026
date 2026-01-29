from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from subsystems.autoSubsystem import AutoSubsystem
from typing import List, NamedTuple

robotState: RobotState = None  # type: ignore


class SubsystemManager(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData
    autos: AutoSubsystem

    def init(self) -> None:
        for s in self.dependantSubsytems:
            s.init()
        self.inputs.init()

    def robotInit(self) -> None:
        self.time.init()

    def robotPeriodic(self) -> None:
        self.time.periodic(self.robotState)
        self.robotState.publish()

    def autonomousPeriodic(self) -> None:
        global robotState
        robotState = self.autos.periodic(self.robotState)
        self._periodic(self.robotState)

    def teleopPeriodic(self) -> None:
        global robotState
        robotState = self.inputs.periodic(self.robotState)
        self._periodic(self.robotState)

    def _periodic(self, robotState: RobotState) -> None:
        for s in self.dependantSubsytems:
            s.periodic(robotState)
        self.time.periodic(robotState)

    def disabled(self) -> None:
        for s in self:
            s.disabled()

    @property
    def dependantSubsytems(
        self,
    ) -> List[
        Subsystem
    ]:  # dependant subsystems (I know how to remove this but I just didnt have enough time to)
        return [
            self.ledSignals,
            self.swerveDrive,
            self.autos,
        ]

    @property
    def robotState(self) -> RobotState:
        global robotState

        if robotState == None:
            robotState = self.inputs.robotState

        return robotState
