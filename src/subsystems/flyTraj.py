import pathplannerlib
from pathplannerlib import pathfinders, pathfinding, trajectory
from pathplannerlib.path import PathPlannerTrajectory, PathConstraints, PathPlannerPath
from pathplannerlib.pathfinders import LocalADStar, GoalEndState
from pathplannerlib.config import ModuleConfig, RobotConfig, DCMotor
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.units import meters_per_second as MPS
from wpimath.units import radians_per_second as RPS
from wpimath.units import feetToMeters, lbsToKilograms
import math
import wpilib
from wpimath.kinematics import ChassisSpeeds


class FlyTraj(Subsystem):

    def __init__(self):
        #self.manager = LocalADStar()
        self.manager = pathfinding.Pathfinding()
        self.finder = pathfinders.LocalADStar()
        #self.publishFloat("test",1.0)
        
        
        print("tick-")

    def phaseInit(self, robotState: RobotState)->RobotState:
        print("tick+")
        self.manager.ensureInitialized()
        self.manager.setPathfinder(self.finder)
        self.state = 0
        
        return robotState

    def periodic(self, robotState: RobotState)->RobotState:
        if robotState.flyTest and self.state == 0:
            print("stage = 0")
            #self.manager.setStartPosition(Translation2d(robotState.pose.X(), robotState.pose.Y()))
            #self.manager.setGoalPosition(Translation2d(0,0))
            self.state = 1
        
        elif robotState.flyTest and self.state == 1:

            # self.publishBoolean("A button works", True, "telemetry")

            # if self.manager.isNewPathAvailable():
                print("stage = 1")
                self.manager.setStartPosition(start_position=Translation2d(robotState.odometry.getEstimatedPosition().X(), robotState.odometry.getEstimatedPosition().Y()))
                # self.manager.setStartPosition(start_position=Translation2d(7, 7))
                self.manager.setGoalPosition(goal_position=Translation2d(1,1))
                self.p: PathPlannerPath = self.manager.getCurrentPath(PathConstraints(5.0, 2.5, 0.7, 0.35, 12, True), GoalEndState(0, Rotation2d(0)))
                
                if self.p is not None:
                    self.state = 2
                    print("path type not none")
                
        elif robotState.flyTest and self.state == 2:   
                # print("**&&**", type(p))
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
                #self.t = p.generateTrajectory(ChassisSpeeds(), robotState.pose.rotation(), robotConfig)
                self.t = self.p.generateTrajectory(ChassisSpeeds(), Rotation2d(0), robotConfig)
                robotState.flyTestGoal = self.t.getEndState().pose
                print("stage = 2")
                self.state = 3        

        elif robotState.flyTest and self.state == 3:
            print("stage = 3")
            self.startTime = wpilib.getTime()
            self.totalTIme = self.t.getTotalTimeSeconds()
            print("start point", self.t.getInitialPose())
            print("end point", self.t.getEndState().pose)
            print("total time", self.totalTIme)
            self.state = 4

        elif robotState.flyTest and self.state == 4:
            time = wpilib.getTime() - self.startTime
            goalState = self.t.sample(time)
            robotState.fieldSpeeds = goalState.fieldSpeeds
            # print(goalState.fieldSpeeds)


        else:
            self.state = 0

        return robotState

        

    def disabled(self):
        pass

    def publish(self):
        
        pass

        