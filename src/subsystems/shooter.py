from subsystems.subsystem import Subsystem
from motor import RevMotor
from inputs import Inputs as input


class shooter(Subsystem):

    def __init__(self, _mechCtrl):
        self.shooterMotor = RevMotor(
            8
        )  # this is not the real device id *please fix later

    def rev(self):
        self.shooterMotor.setVelocity(200)  # value must be calibrated on bot

    def periodic(self):
        if input.revShooter:
            self.rev()
