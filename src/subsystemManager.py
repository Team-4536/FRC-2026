from dataclasses import dataclass, fields
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import DesiredState, CurrentState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import ClassVar, Iterable, List

import wpimath.kinematics  # EXAMPLE


class AutoStages(Subsystem):
    def init(self) -> None:
        self.desiredState = DesiredState(wpimath.kinematics.ChassisSpeeds(), 0)

    def periodic(self, desiredState: DesiredState, currentState: CurrentState) -> None:
        pass

    def disabled(self) -> None:
        pass


@dataclass
class Subsystems:
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData

    PLACEHOLDER_AUTO_STAGES: ClassVar[AutoStages] = AutoStages()  # example

    @property
    def dependant(self) -> List[Subsystem]:  # dependant subsystems
        return [self.ledSignals, self.swerveDrive]


class SubsystemManager:
    def __init__(self, subsystems: Subsystems):
        self._sub = subsystems
        self.currentState = CurrentState()

    def __iter__(self):
        for f in fields(self._sub):
            yield getattr(self._sub, f.name)

    def init(self) -> None:
        for s in self:
            s.init()

    def robotPeriodic(self) -> None:
        self._sub.time.periodic(self._sub.inputs.desiredState, self.currentState)

    def autonomousPeriodic(self) -> None:
        self._sub.PLACEHOLDER_AUTO_STAGES.periodic(self._sub.inputs.desiredState, self.currentState)
        self._periodic(self._sub.PLACEHOLDER_AUTO_STAGES.desiredState)

    def teleopPeriodic(self) -> None:
        self._sub.inputs.periodic(self._sub.inputs.desiredState, self.currentState)
        self._periodic(self._sub.inputs.desiredState)

    def _periodic(self, desiredState: DesiredState) -> None:
        for s in self._dependantSubsystems:
            s.periodic(desiredState, self.currentState)

    def disable(self) -> None:
        for s in self:
            s.disabled()

    def publish(self) -> None:
        for s in self:
            s.publish()

    @property
    def _dependantSubsystems(self) -> Iterable[Subsystem]:
        return iter(self._sub.dependant)
