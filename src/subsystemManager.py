from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import NamedTuple, Sequence

robotState: RobotState = None  # type: ignore


class SubsystemManager(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData

    def init(self) -> None:
        for s in self.dependantSubsytems:
            s.phaseInit()
        self.inputs.phaseInit()

    def robotPeriodic(self) -> None:
        self.robotState.publish()
        for s in self:
            s.publish()

    def autonomousPeriodic(self) -> None:
        global robotState
        robotState = self.inputs.periodic(self.robotState)
        self._periodic(self.robotState)

    def teleopPeriodic(self) -> None:
        global robotState
        robotState = self.inputs.periodic(self.robotState)
        self._periodic(self.robotState)

    def _periodic(self, state: RobotState) -> None:
        global robotState
        for s in self.dependantSubsytems:
            robotState = s.periodic(state)

    def disabled(self) -> None:
        for s in self:
            s.disabled()

    @property
    def dependantSubsytems(self) -> Sequence[Subsystem]:  # dependant subsystems
        return [
            self.ledSignals,
            self.swerveDrive,
            self.time,
        ]

    @property
    def robotState(self) -> RobotState:
        global robotState

        if not robotState:
            robotState = self.inputs.robotState

        return robotState
