from subsystem import Subsystem
from desiredState import DesiredState
from motor import RevMotor

class Climber(Subsystem):
    def __init__(self):
        self.climberLeft = RevMotor(deviceID= 15)
        self.climberRight = RevMotor(deviceID= 16)
        pass 
    def init(self):
        self.speedLeft= 0
    def periodic(self, ds: DesiredState):
        pass
    def disabled(self):
        pass
    def publish(self):
        pass
