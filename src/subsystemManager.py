from dataclasses import dataclass, fields
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import ClassVar, Iterable, List

import wpimath.kinematics  # EXAMPLE


class AutoStages(Subsystem):
    def init(self) -> None:
        self.robotState = RobotState(wpimath.kinematics.ChassisSpeeds(), 0)

    def periodic(self, robotState: RobotState) -> None:
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

    def __iter__(self):
        for f in fields(self._sub):
            yield getattr(self._sub, f.name)

    def init(self) -> None:
        for s in self:
            s.init()

    def robotPeriodic(self) -> None:
        self._sub.time.periodic(self._sub.inputs.robotState)
        self._sub.inputs.robotState.publish()

    def autonomousPeriodic(self) -> None:
        self._sub.PLACEHOLDER_AUTO_STAGES.periodic(self._sub.inputs.robotState)
        self._periodic(self._sub.PLACEHOLDER_AUTO_STAGES.robotState)

    def teleopPeriodic(self) -> None:
        self._sub.inputs.periodic(self._sub.inputs.robotState)
        self._periodic(self._sub.inputs.robotState)

    def _periodic(self, robotState: RobotState) -> None:
        for s in self._dependantSubsystems:
            s.periodic(robotState)

    def disabled(self) -> None:
        for s in self:
            s.disabled()

    @property
    def _dependantSubsystems(self) -> Iterable[Subsystem]:
        return iter(self._sub.dependant)

    @property
    def robotState(self) -> RobotState:
        return self._sub.inputs.robotState
