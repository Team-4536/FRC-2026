from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory
from pathplannerlib.config import ModuleConfig, RobotConfig, DCMotor
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from wpimath.units import feetToMeters, lbsToKilograms
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds
from subsystems.robotState import RobotState
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

    robotMassKG = lbsToKilograms(100)  # change later
    robotMOI = (1 / 12) * robotMassKG * 2 * feetToMeters(1) ** 2  # change later
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
        path = path.flipPath()

    startingPose = path.getStartingHolonomicPose()
    startingRotation = Rotation2d()
    if startingPose:
        startingRotation = startingPose.rotation()

    return path.generateTrajectory(ChassisSpeeds(), startingRotation, robotConfig)


class AutoStages:
    def __init__(self):
        pass

    def autoInit(self):
        pass

    def run(self, robotState: RobotState) -> RobotState:
        return robotState

    def isDone(self) -> bool:
        return False


class FollowTrajectory(AutoStages):
    def __init__(self, pathName: str, isFlipped: bool):
        self.trajectory = loadTrajectory(pathName, isFlipped)
        self.robotState = RobotState
        self.done = False

    def autoInit(self):
        self.startTime = wpilib.getTime()

    def run(self, robotState: RobotState):

        self.robotState = robotState

        self.pathTime = wpilib.getTime() - self.startTime

        targetState = self.trajectory.sample(self.pathTime)

        self.robotState.fieldSpeeds = targetState.fieldSpeeds

        print(str(targetState.fieldSpeeds) + "***********************")

        return self.robotState

    def isDone(self) -> bool:
        if self.robotState.pose == None:
            self.robotState.pose = Pose2d()

        print(self.robotState.pose, "robot pose")

        currXPos = self.robotState.pose.x
        currYPos = self.robotState.pose.y
        currRotation = self.robotState.pose.rotation().radians()
        endXPos = self.trajectory.getEndState().pose.x
        endYPos = self.trajectory.getEndState().pose.y
        endRotation = self.trajectory.getEndState().pose.rotation().radians()
        posError = 0.3  # change later
        rotationError = 0.3  # change later

        print(endXPos, endYPos, endRotation, "end pos")
        print(self.trajectory.getTotalTimeSeconds())

        if self.pathTime > self.trajectory.getTotalTimeSeconds():
            self.done = True
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

        self.done = True
        return True
