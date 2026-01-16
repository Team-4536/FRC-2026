from subsystems.desiredState import DesiredState
from subsystems.inputs import Inputs
from subsystems.shooter import Shooter
from subsystems.LEDSignals import LEDSignals
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import List, NamedTuple


class SubsystemManager(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData
    shooter: Shooter

    def init(self) -> None:
        for s in self:
            s.init()

    def periodic(self) -> None:
        ds = self.desiredState
        for s in self._dependant:
            s.periodic(ds)
        self.publish()

    def disable(self) -> None:
        for s in self:
            s.disabled()

    def publish(self) -> None:
        for s in self:
            s.publish()

    @property
    def _dependant(self) -> List[Subsystem]:
        return [
            self.ledSignals,
            self.swerveDrive,
            self.time,
            self.shooter,
        ]

    @property
    def desiredState(self) -> DesiredState:
        self.inputs.periodic(self.inputs.desiredState)
        return self.inputs.desiredState
