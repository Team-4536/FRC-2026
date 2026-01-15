from subsystems.subsystem import Subsystem
from motor import RevMotor
from inputs import Inputs as input

# motor vars do not include real device id at this time (1-13)
# all RPM values must be tested on the robot and adjusted acoringly


class Shooter(Subsystem):

    def __init__(self, _mechCtrl):
        self.revingMotor = RevMotor(deviceID=8)
        self.shooterMotor = RevMotor(deviceID=9)

    def init(self) -> None:
        pass

    def periodic(self, ds) -> None:
        if ds.revShooter > 0.1:
            self.revingMotor.setVelocity(100 * (ds.revShooter + 1))
        elif ds.shootShooter:
            self.shooterMotor.setVelocity(120)
        else:
            self.disabled()

    def disabled(self) -> None:
        self.shooterMotor.setVelocity(0)
        self.revingMotor.setVelocity(0)
