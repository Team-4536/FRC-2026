from pyfrc.physics.core import PhysicsEngine as PhysicsEngineBase
from pyfrc.physics.core import PhysicsInterface
from robot import Robot
from wpimath.geometry import Rotation2d, Transform2d
from wpimath.kinematics import ChassisSpeeds


class PhysicsEngine(PhysicsEngineBase):
    def __init__(self, controller: PhysicsInterface, robot: Robot) -> None:
        super().__init__(controller)

        self.ctrlr = controller
        self.robot = robot

        self.ctrlr.move_robot(Transform2d(x=1, y=3, rotation=Rotation2d(0)))

    def update_sim(self, now: float, tm_diff: float) -> None:
        fieldSpeeds = self.robot.subsystems.robotState.fieldSpeeds
        pose = self.ctrlr.get_pose()
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds=fieldSpeeds, robotAngle=pose.rotation()
        )

        self.ctrlr.drive(
            ChassisSpeeds.discretize(continuousSpeeds=chassisSpeeds, dt=tm_diff),
            tm_diff,
        )
