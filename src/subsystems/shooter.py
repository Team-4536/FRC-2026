from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.desiredState import DesiredState
from ntcore import NetworkTableInstance

# motor vars do not include real device id at this time (1-13)
# all RPM values must be tested on the robot and adjusted acoringly


class Shooter(Subsystem):

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.hubDistance = 2  # this will later be equal to the hypotenuse of Odometry and the Hub position

        self.revingMotor = RevMotor(deviceID=8)
        self.shooterMotor = RevMotor(deviceID=9)
        super().__init__()

    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        self.table.putNumber("revShooter", ds.revShooter)
        self.table.putNumber("shootShooter", ds.shootShooter)

        if ds.revShooter > 0.1:
            self.revingMotor.setVelocity(
                (100 * (ds.revShooter + 1)) * self.hubDistance
            )  # relationship between distace and revingMotor must be calibrated
        elif ds.shootShooter and self.revingMotor.getEncoder().getVelocity == 1:
            self.shooterMotor.setVelocity(120)
        else:
            self.disabled()

    def disabled(self) -> None:
        self.shooterMotor.setVelocity(0)
        self.revingMotor.setVelocity(0)

    def publish(self):
        pass
