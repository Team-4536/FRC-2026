from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import List, NamedTuple
from subsystems.cameras import CameraManager
from wpimath.estimator import SwerveDrive4PoseEstimator

robotState: RobotState = None  # type: ignore


class SubsystemManager(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData
    cameras: CameraManager

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
        robotState = self.inputs.periodic(self.robotState)  # replace with auto manager
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
            self.cameras,
        ]

    @property
    def robotState(self) -> RobotState:
        global robotState

        if robotState == None:
            robotState = self.inputs.robotState
            robotState.odometry = SwerveDrive4PoseEstimator(
                self.swerveDrive._kinematics,
                self.swerveDrive.gyro.getRotation2d(),
                self.swerveDrive._modules.modulePositions,
                self.swerveDrive.initPos,
            )

        return robotState
