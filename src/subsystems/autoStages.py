from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory
from pathplannerlib.config import ModuleConfig, RobotConfig, DCMotor
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from desiredState import DesiredState
import math
import wpilib


def loadTrajectory(filename: str, isFlipped: bool) -> PathPlannerTrajectory:

    nominalVoltage = 12.0
    stallTorque = 2.6
    stallCurrent = 105.0
    freeCurrent = 1.8
    freeSpeed: RPS = (5676 * 2 * math.pi) / 60

    wheelRadiusMeters = 0.0508
    maxVelocity: MPS = 2
    wheelCOF = 1
    motor = DCMotor(nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed)
    currentLimit = 40

    topLeftWheelCords = Translation2d(-0.276225, 0.276225)
    topRightWheelCords = Translation2d(0.276225, 0.276225)
    bottomLeftWheelCords = Translation2d(-0.276225, -0.276225)
    bottomRightWheelCords = Translation2d(0.276225, -0.276225)

    robotMassKG = 0  # change later
    robotMOI = 0  # change later
    moduleConfig = ModuleConfig(
        wheelRadiusMeters, maxVelocity, wheelCOF, motor, currentLimit, 1
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
        path.flipPath

    startingRotation = path.getStartingHolonomicPose().getRotation()

    return path.generateTrajectory(ChassisSpeeds(), startingRotation, robotConfig)


class AutoStages:
    def __init__(self):
        pass

    def autoInit(self):
        pass

    def run(self):
        pass

    def isDone(self):
        pass


class FollowTrajectory(AutoStages):
    def __init__(self, trajectoryName: str, isFlipped: bool):
        self.trajectory = loadTrajectory(trajectoryName, isFlipped)
        self.desiredState = DesiredState
        self.isDone = False

    def autoInit(self):
        self.startTime = wpilib.getTime()

    def run(self):

        self.currentTime = wpilib.getTime() - self.startTime

        self.targetState = self.trajectory.sample(self.currentTime)

        self.desiredState.fieldSpeeds = self.targetState.fieldSpeeds

        return self.desiredState

    def isDone(self):
        currXPos = 0  # update with odemetry later
        currYPos = 0  # update with odemetry later
        currRotation = 0  # update with odemetry later
        endXPos = self.trajectory.getEndState.pose
        endYPos = self.trajectory.getEndState.pose
        endRotation = self.trajectory.getEndState.pose
        posError = 0.3  # change later
        rotationError = 0.3  # change later

        if max(currXPos, endXPos) - min(currXPos, endXPos) > posError:
            return False
        if max(currYPos, endYPos) - min(currYPos, endYPos) > posError:
            return False
        if (
            max(currRotation, endRotation) - min(currRotation, endRotation)
            > rotationError
        ):
            return False

        self.isDone = True
        return True
