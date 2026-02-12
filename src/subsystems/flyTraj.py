import pathplannerlib
from pathplannerlib import pathfinders, pathfinding, trajectory
from pathplannerlib.path import PathPlannerTrajectory, PathConstraints
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.config import ModuleConfig, RobotConfig, DCMotor
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState
from wpimath.geometry import Translation2d, Pose2d
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from wpimath.units import feetToMeters, lbsToKilograms
import math
import wpilib
from wpimath.kinematics import ChassisSpeeds

class FlyTraj(Subsystem):

    def __init__(self):
        self.manager = LocalADStar()

    def phaseInit(self):
        self.manager.setDynamicObstacles(list(tuple(Translation2d(2,2), Translation2d(4,4))))
        self.state = 0

    def periodic(self, robotState: RobotState):
    
        if robotState.flyTest and self.state == 0:
            self.manager.setStartPosition(robotState.pose)
            self.manager.setGoalPosition(Translation2d(0,0,0))
            self.state = 1
        
        if robotState.flyTest and self.state == 1:
            p = self.manager.getCurrentPath(PathConstraints(5.0, 2.0, 0.3, 0.05, 12, True), Translation2d(0,0,0))
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
            self.t = p.generateTrajectory(ChassisSpeeds(), p.getStartingHolonomicPose().rotation(), robotConfig)
            self.state = 3

        if self.state == 3:
            startTime = wpilib.getTime()
            self.state = 4

        if self.state == 4:
            time = wpilib.getTime() - startTime
            goalState = self.t.sample(time)
            robotState.fieldSpeeds = goalState.fieldSpeeds

            if time > self.t.getTotalTimeSeconds():
                self.state = 0

        return robotState

        

    def disabled(self):
        pass

    def publish(self):
        pass

        