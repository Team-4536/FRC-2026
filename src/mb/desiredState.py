from wpimath.kinematics import ChassisSpeeds


class DesiredState:
    def fieldSpeeds(self) -> ChassisSpeeds:
        raise "fieldSpeeds method required"
