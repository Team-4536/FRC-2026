from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import matchData
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import seconds


class Tester(Subsystem):
    testDrive: bool = True
    testIntake: bool = True
    testTurret: bool = True
    testClimb: bool = True

    currentlyTesting: str = "Not Testing"
    _lastTesing: str = ""

    def __init__(self) -> None:
        super().__init__(table="Testing", inst=False)

        self.publishBoolean("drive_tests", self.testDrive)
        self.publishBoolean("intake_tests", self.testIntake)
        self.publishBoolean("turret_tests", self.testTurret)
        self.publishBoolean("climber_tests", self.testClimb)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self.testDrive = self.getBoolean("drive_tests", None, inst=False, default=False)
        self.testIntake = self.getBoolean(
            "intake_tests", None, inst=False, default=False
        )
        self.testTurret = self.getBoolean(
            "turret_tests", None, inst=False, default=False
        )
        self.testClimb = self.getBoolean("climb_tests", None, inst=False, default=False)

        self.currentlyTesting = "Started"

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        time: seconds = matchData.timeSincePhaseInit
        offset: seconds = -1

        if self.testDrive:
            offset += self._testDrive(robotState, time - offset)

        if self.testIntake:
            offset += self._testIntake(robotState, time - offset)

        if self.testTurret:
            offset += self._testTurret(robotState, time - offset)

        if self.testClimb:
            offset += self._testClimb(robotState, time - offset)

        if time - offset > 0:
            self.currentlyTesting = "Done"

        return robotState

    def _testDrive(self, state: RobotState, time: seconds) -> int:
        if 0 < time and time < 3:
            self.currentlyTesting = "Drive"
            state.fieldSpeeds = ChassisSpeeds(-0.5, 0, 0)
        elif 4 < time and time < 6:
            state.fieldSpeeds = ChassisSpeeds(0.5, 0, 0.5)
        elif 7 < time and time < 8:
            state.fieldSpeeds = ChassisSpeeds(0, 0, -1)
        else:
            state.fieldSpeeds = ChassisSpeeds(0, 0, 0)

        return 9

    def _testIntake(self, state: RobotState, time: seconds) -> int:
        if 0 < time and time < 2:
            self.currentlyTesting = "Intake"
            state.initialIntake = True
        else:
            state.initialIntake = False

        return 3

    def _testTurret(self, state: RobotState, time: seconds) -> int:
        return 0

    def _testClimb(self, state: RobotState, time: seconds) -> int:
        return 0

    def disabled(self):
        pass

    def publish(self):
        if self.currentlyTesting != self._lastTesing:
            self.publishString("currently_testing", self.currentlyTesting)
            self._lastTesing = self.currentlyTesting
