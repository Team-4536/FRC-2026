from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory
from pathplannerlib.config import ModuleConfig, RobotConfig, DCMotor
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
import math


def loadTrajectory(filename: str, flipped: bool) -> PathPlannerTrajectory:

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

    robotMassKG = 0
    robotMOI = 0
    moduleConfig = ModuleConfig(
        wheelRadiusMeters, maxVelocity, wheelCOF, motor, currentLimit, 1
    )
    moduleOffsets = [topLeftWheelCords, topRightWheelCords, bottomLeftWheelCords, bottomRightWheelCords]

    robotConfig = RobotConfig(robotMassKG, robotMOI, moduleConfig, moduleOffsets)

    path = PathPlannerPath.fromPathFile(filename)

    if flipped:
        path.flipPath

    startingRotation = path.getStartingHolonomicPose().getRotation()

    return path.generateTrajectory(ChassisSpeeds(), startingRotation, robotConfig)
