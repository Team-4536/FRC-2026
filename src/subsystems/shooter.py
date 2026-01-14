from subsystems.subsystem import Subsystem
from motor import RevMotor
from inputs import Inputs as input


class shooter(Subsystem):

    def __init__(
        self, _mechCtrl
    ):  # motor vars do not include real device id at this time (1-13) *please fix later
        self.revingMotor = RevMotor(8)
        self.shooterMotor = RevMotor(9)

    def periodic(self):
        if input.revShooter > 0.1:
            self.revingMotor.setVelocity(100) * (
                input.revShooter() + 1
            )  # value must be calibrated on bot
            return True
        elif input.shootShooter():
            self.shooterMotor.setVelocity(120)
        else:
            self.shooterMotor.setVelocity(0)
            self.revingMotor.setVelocity(0)
            return False
