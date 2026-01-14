from subsystems.desiredState import DesiredState
from subsystems.subsystem import Subsystem
from motor import RevMotor
from phoenix6.hardware.cancoder import CANcoder
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Rotation2d
import math
#gearing is type L3
#6.12:1 ratio

class SwerveModule():
    
    def init(self, driveMotor: RevMotor, rotMotor: RevMotor, encoder: CANcoder) -> None:
        self.driveMotor = driveMotor
        self.rotMotor = rotMotor
        self.encoder = encoder
        self.gearRatio = 6.12
        self.position = self.driveMotor.getEncoder().getPosition()
        self.rotation = self.rotMotor.getEncoder().getPosition()

    @property
    def xDistance(self) -> int:
        return (self.position * 0.098425) / self.gearRatio
    
    @property
    def angle(self) -> int:
        return Rotation2d((self.rotation / self.gearRatio) * (2 * math.pi))
    
    @property
    def modulePosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.xDistance, self.angle)

    def periodic(self, ds: DesiredState) -> None:
        pass

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass

    
class SwerveModules():
    
    def init(self, frontLeft: SwerveModule, frontRight: SwerveModule, backLeft: SwerveModule, backRight: SwerveModule) -> None:
        self.frontLeft = frontLeft
        self.frontRight = frontRight
        self.backLeft = backLeft
        self.backRight = backRight
        

    def periodic(self, ds: DesiredState) -> None:
        pass

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


class SwerveDrive(Subsystem):
    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        pass

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass