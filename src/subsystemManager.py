from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from subsystems.autoSubsystem import AutoSubsystem
from subsystems.intake import Intake
from typing import List, NamedTuple
from wpimath.estimator import SwerveDrive4PoseEstimator
from typing import NamedTuple, Sequence

robotState: RobotState = None  # type: ignore


class SubsystemManager(NamedTuple):
    inputs: Inputs  # NOT A DEPENDANT SUBSYSTEM
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData
    autos: AutoSubsystem
    intake: Intake

    def init(self) -> None:
        global robotState
        for s in self.dependantSubsytems:
            robotState = s.phaseInit(self.robotState)
        self.autos.phaseInit(self.robotState)
        self.inputs.phaseInit(self.robotState)

    def robotPeriodic(self) -> None:
        self.robotState.publish()
        for s in self:
            s.publish()

    def autonomousPeriodic(self) -> None:
        global robotState
        robotState = self.autos.periodic(self.robotState)
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
            self.intake,
        ]

    @property
    def robotState(self) -> RobotState:
        global robotState

        if not robotState:
            robotState = self.inputs.robotState
            robotState.odometry = SwerveDrive4PoseEstimator(
                self.swerveDrive._kinematics,
                self.swerveDrive._gyro.getRotation2d(),
                self.swerveDrive._modules.modulePositions,
                self.swerveDrive.initPos,
            )

        return robotState
