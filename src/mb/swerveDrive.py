from mb.subsystem import Subsystem
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState


class SwerveDrive(Subsystem):
    def __init__(self) -> None:
        self.modules = None
        self.kinematics = SwerveDrive4Kinematics()

    def drive(self, fieldSpeeds: ChassisSpeeds) -> None:
        moduleStates = self.kinematics.toSwerveModuleStates(fieldSpeeds)

        moduleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(moduleStates, 1)

        for module, state in zip(self.modules, moduleStates):
            pass
