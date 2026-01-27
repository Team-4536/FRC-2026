from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.desiredState import DesiredState
from ntcore import NetworkTableInstance
import math

# motor vars do not include real device id at this time (1-13)
# all RPM values must be tested on the robot and adjusted acoringly


class Shooter(Subsystem):

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.hubDistance = 2  # hypotenuse of odometry and the Hub position
        self.wheelRadius = 0.1  # radius of the wheels build decides to use
        self.turretAngle = 70  # replace with the turret subsystem's outputed variable

        self.revingMotor = RevMotor(deviceID=8)
        self.shooterMotor = RevMotor(deviceID=9)
        super().__init__()

    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        self.table.putNumber("revShooter", ds.revShooter)
        self.table.putNumber("shootShooter", ds.shootShooter)

        if ds.revShooter > 0.1:
            self.revingMotor.setVelocity(self._calculateVelocity())
        elif ds.shootShooter and self.revingMotor.getEncoder().getVelocity() == 1:
            self.shooterMotor.setVelocity(self._calculateVelocity())
        else:
            self.disabled()

    def _calculateVelocity(self) -> float:
        self.velocityMps = math.sqrt(
            (9.81 * self.turretAngle**2)
            / (
                2
                * math.cos(self.turretAngle)
                * (self.hubDistance * math.tan(self.turretAngle) - 0.9652)
            )
        )
        return self.velocityMps / self.wheelRadius

    def disabled(self) -> None:
        self.shooterMotor.setVelocity(0)
        self.revingMotor.setVelocity(0)

    def publish(self):
        pass
