from subsystems.inputs import Inputs
from subsystems.subsystemExample import SubsystemExample
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import List, NamedTuple
from subsystems.climbing import Climbing
from subsystems.turret import Turret 

robotState: RobotState = None  # type: ignore


class SubsystemManager(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData
    subsystemExample: SubsystemExample 
    climbing: Climbing
    turret: Turret
    

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

    def _periodic(self, robotState: RobotState) -> None:
        for s in self.dependantSubsytems:
            s.periodic(robotState)

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
            self.time,
            self.subsystemExample,
        ]

    @property
    def robotState(self) -> RobotState:
        global robotState

        if not robotState:
            robotState = self.inputs.robotState

        return robotState
