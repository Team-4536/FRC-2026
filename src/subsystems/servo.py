from subsystems.subsystem import Subsystem
from rev import ServoHub
from subsystems.robotState import RobotState
class TestingServo(Subsystem):

    def _init_(self):
        self.servoHub = ServoHub(deviceID= 3)
        self.servo1 = self.servoHub.getServoChannel(2)
        pass
    def init(self):
        pass
    def periodic(self, rs: RobotState):
        self.servo1.setPowered(True)
        pass
    def disabled(self):
        #self.servo1.setPowered(False)
        pass 
    def publish(self):
        pass












