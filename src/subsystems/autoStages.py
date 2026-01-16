from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory
from pathplannerlib.config import ModuleConfig, RobotConfig, DCMotor
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from wpimath.geometry import Translation2d


def loadTrajectory(filename: str, flipped: bool) -> PathPlannerTrajectory:

    nominalVoltage = 12.0
    stallTorque = 2.6
    stallCurrent = 105.0
    freeCurrent = 1.8
    freeSpeed: RPS = 94.6

    wheelRadiusMeters = 0.0508
    maxVelocity: MPS = 2
    wheelCOF = 1
    motor = DCMotor(nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed)
    currentLimit = 40

    robotMassKG = 0
    robotMOI = 0
    moduleConfig = ModuleConfig(
        wheelRadiusMeters, maxVelocity, wheelCOF, motor, currentLimit, 1
    )
    moduleOffsets = [
        Translation2d(-0.276225, 0.276225),
        Translation2d(0.276225, 0.276225),
        Translation2d(-0.276225, -0.276225),
        Translation2d(0.276225, -0.276225),
    ]

    robotConfig = RobotConfig(robotMassKG, robotMOI, moduleConfig, moduleOffsets)

    path = PathPlannerPath.fromPathFile(filename)

    if flipped:
        path.flipPath

    return path.generateTrajectory(_, _, robotConfig)
