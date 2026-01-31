from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from subsystems.intake import Intake
from typing import List, NamedTuple

import wpimath.kinematics  # FOR THE EXAMPLE


class AutoStages(Subsystem):
    def init(self) -> None:
        self.robotState = RobotState(
            wpimath.kinematics.ChassisSpeeds(), 0, False, False, False
        )

    def periodic(self, robotState: RobotState) -> RobotState:
        return robotState

    def disabled(self) -> None:
        pass


robotState: RobotState | None = None


class SubsystemManager(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData
    intake: Intake

    PLACEHOLDER_AUTO_STAGES: AutoStages = AutoStages()  # example

    def init(self) -> None:
        for s in self.dependantSubsytems:
            s.init()
        self.PLACEHOLDER_AUTO_STAGES.init()

    def robotInit(self) -> None:
        self.time.init()

    def robotPeriodic(self) -> None:
        self.time.periodic(self.robotState)
        self.robotState.publish()

    def autonomousPeriodic(self) -> None:
        global robotState
        robotState = self.PLACEHOLDER_AUTO_STAGES.periodic(self.robotState)
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
        ]

    @property
    def robotState(self) -> RobotState:
        global robotState

        if robotState == None:
            robotState = self.inputs.robotState

        return robotState
