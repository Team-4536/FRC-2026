from mb.desiredState import DesiredState
from mb.inputs import Inputs
from mb.LEDSignals import LEDSignals
from mb.subsystem import Subsystem
from mb.swerveDrive import SwerveDrive
from mb.utils import TimeData
from typing import List, NamedTuple


class Subsystems(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData

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
        ]

    @property
    def desiredState(self) -> DesiredState:
        self.inputs.periodic(self.inputs.desiredState)
        return self.inputs.desiredState
