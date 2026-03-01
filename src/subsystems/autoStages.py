from math import tau
from pathplannerlib.config import ModuleConfig, RobotConfig, DCMotor  # pyright: ignore
from pathplannerlib.path import (  # pyright: ignore
    PathPlannerPath,
    PathPlannerTrajectory,
)
from subsystems.robotState import RobotState
from wpilib import getTime
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import (
    feetToMeters,
    lbsToKilograms,  # pyright: ignore
    meters_per_second,
    radians_per_second,
)


def loadTrajectory(filename: str, isFlipped: bool) -> PathPlannerTrajectory:

    nominalVoltage = 12.0
    stallTorque = 2.6
    stallCurrent = 105.0
    freeCurrent = 1.8
    freeSpeed: radians_per_second = (5676 * tau) / 60

    wheelRadiusMeters = 0.0508
    maxVelocity: meters_per_second = 3
    wheelCOF = 1
    motor = DCMotor(nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed)
    currentLimit = 40

    topLeftWheelCords = Translation2d(-0.276225, 0.276225)
    topRightWheelCords = Translation2d(0.276225, 0.276225)
    bottomLeftWheelCords = Translation2d(-0.276225, -0.276225)
    bottomRightWheelCords = Translation2d(0.276225, -0.276225)

    robotMassKG = lbsToKilograms(100)  # TODO: change later
    robotMOI = (1 / 12) * robotMassKG * 2 * feetToMeters(1) ** 2  # TODO: change later
    moduleConfig = ModuleConfig(
        wheelRadiusMeters, maxVelocity, wheelCOF, motor, currentLimit, 4
    )
    moduleOffsets = [
        topLeftWheelCords,
        topRightWheelCords,
        bottomLeftWheelCords,
        bottomRightWheelCords,
    ]

    robotConfig = RobotConfig(robotMassKG, robotMOI, moduleConfig, moduleOffsets)

    path = PathPlannerPath.fromPathFile(filename)

    if isFlipped:
        path = path.flipPath()

    startingPose = path.getStartingHolonomicPose()
    startingRotation = Rotation2d(20)  # TODO: understand
    if startingPose:
        startingRotation = startingPose.rotation()

    return path.generateTrajectory(ChassisSpeeds(), startingRotation, robotConfig)


class AutoStages:
    def __init__(self):
        pass

    def autoInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def run(self, robotState: RobotState) -> RobotState:
        return robotState

    def end(self, robotState: RobotState) -> RobotState:
        return robotState

    def isDone(self) -> bool:
        return False


class FollowTrajectory(AutoStages):
    # Declare Variables

    pathDone: bool
    trajectory: PathPlannerTrajectory
    robotState: RobotState
    startTime: float
    pathTime: float

    def __init__(self, pathName: str, isFlipped: bool):
        self.trajectory = loadTrajectory(pathName, isFlipped)
        self.robotState = RobotState.empty()

        self.pathDone = False

    def autoInit(self, robotState: RobotState) -> RobotState:
        self.startTime = getTime()
        robotState.autosGyroResetToggle = True
        robotState.autosGyroReset = (
            self.trajectory.getInitialPose().rotation().degrees()
        )
        robotState.autosInitPose = self.trajectory.getInitialPose()
        return robotState

    def run(self, robotState: RobotState):

        self.robotState = robotState

        self.pathTime = getTime() - self.startTime

        targetState = self.trajectory.sample(self.pathTime)

        self.robotState.fieldSpeeds = targetState.fieldSpeeds

        return self.robotState

    def end(self, robotState: RobotState) -> RobotState:
        return robotState

    def isDone(self) -> bool:
        currXPos = self.robotState.odometry.getEstimatedPosition().x
        currYPos = self.robotState.odometry.getEstimatedPosition().y
        currRotation = (
            self.robotState.odometry.getEstimatedPosition().rotation().radians()
        )
        endXPos = self.trajectory.getEndState().pose.x
        endYPos = self.trajectory.getEndState().pose.y
        endRotation = self.trajectory.getEndState().pose.rotation().radians()
        posError = 0.02  # TODO: change later
        rotationError = 0.1  # TODO: change later

        if self.pathTime > self.trajectory.getTotalTimeSeconds():
            self.pathDone = True
            return True
        if max(currXPos, endXPos) - min(currXPos, endXPos) > posError:
            return False
        if max(currYPos, endYPos) - min(currYPos, endYPos) > posError:
            return False
        if (
            max(currRotation, endRotation) - min(currRotation, endRotation)
            > rotationError
        ):
            return False

        self.pathDone = True
        return True


class OperateIntake(AutoStages):
    robotState: RobotState
    startTime: float
    pathTime: float
    runTime: float
    pathDone: bool

    def __init__(self, runTime: float = 0):
        self.pathDone = False
        self.runTime = runTime

    def autoInit(self, robotState: RobotState) -> RobotState:
        self.startTime = getTime()

        return robotState

    def run(self, robotState: RobotState) -> RobotState:
        self.robotState = robotState
        self.pathTime = getTime() - self.startTime

        if self.pathTime < 0.5:  # TODO: make this not work like this
            self.robotState.intakePosYAxis = 0.5
        else:
            self.robotState.intakePosYAxis = 0
            self.robotState.initialIntake = True
            self.robotState.intakeIndexer = True

        return self.robotState

    def end(self, robotState: RobotState) -> RobotState:
        robotState.intakePosYAxis = 0
        robotState.initialIntake = False
        robotState.intakeIndexer = False

        return self.robotState

    def isDone(self) -> bool:

        if self.pathTime < self.runTime or self.pathTime < 0.5:
            return False

        self.pathDone = True
        return True


class OperateTurret(AutoStages):
    robotState: RobotState
    startTime: float
    pathTime: float
    runTime: float
    pathDone: bool

    def __init__(self, unload: bool = False, runTime: float = 0):
        self.pathDone = False
        self.runTime = runTime
        self.unload = unload

    def autoInit(self, robotState: RobotState) -> RobotState:
        self.startTime = getTime()

        return robotState

    def run(self, robotState: RobotState) -> RobotState:
        self.robotState = robotState
        self.pathTime = getTime() - self.startTime

        self.robotState.forceDynamicTurret = True
        self.robotState.revSpeed = 1
        self.robotState.kickShooter = self.unload

        return self.robotState

    def end(self, robotState: RobotState) -> RobotState:
        self.robotState.revSpeed = 0
        self.robotState.kickShooter = False

        return self.robotState

    def isDone(self) -> bool:

        if self.pathTime < self.runTime:
            return False

        self.pathDone = True
        return True
