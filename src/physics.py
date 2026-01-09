from pyfrc.physics.core import PhysicsEngine as PhysicsEngineBase
from robot import Robot
from wpimath.geometry import Rotation2d, Transform2d
from wpimath.kinematics import ChassisSpeeds


class PhysicsEngine(PhysicsEngineBase):
    def __init__(self, controller, robot: Robot):
        super().__init__(controller)

        self.ctl = controller
        self.robot = robot

        self.ctl.move_robot(Transform2d(x=1, y=3, rotation=Rotation2d(0)))

    def update_sim(self, now: float, tm_diff: float):
        fieldSpeeds = self.robot.inputs.fieldSpeeds
        pose = self.ctl.get_pose()
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds=fieldSpeeds, robotAngle=pose.rotation()
        )

        self.ctl.drive(
            ChassisSpeeds.discretize(continuousSpeeds=chassisSpeeds, dt=tm_diff),
            tm_diff,
        )
