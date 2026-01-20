from subsystem import Subsystem
from desiredState import DesiredState
from motor import RevMotor
import random


#class shooter(Subsystem):
#    def __init__(self):
#        self.mymotor = RevMotor(deviceID=5) #change motor id when its known
#        pass 
#
#    def init(self) -> None:
#        self._warn(self.init)
#
#    def periodic(self, ds: DesiredState) -> None:
#        self._warn(self.periodic)
#
#        if ds.on:
#            self.mymotor.set(1.0) #set to desired speed when known #1.0 is VERY fast
#        else:
#            self.mymotor.set(0.0)
#
#    def disabled(self) -> None:
#        self._warn(self.disabled)
#
#    def publish(self) -> None:
#        self._warn(self.publish)

     
class Shooter():
    def __init__(self, ds):
        self.mymotor = RevMotor(deviceID=5)  # Ensure motor is initialized

        speed = [0.0, 0.2, 0.4, 0.5, 0.6, 0.8, 1.0]     # corresponding speeds (0.0 to 1.0)
        self.distance = random.uniform(0, 223)
        print(str(self.distance) + " distance") 

        if self.distance < 0.1:
            self.desired_speed = speed[0]
        elif self.distance < 150:
            self.desired_speed = speed[3]
        elif self.distance < 224: #the max feild size is 223 inches
            self.desired_speed = speed[6]  
        else:
            self.desired_speed = speed[0]

        if ds.on:
            self.mymotor.setVelocity(self.desired_speed)
        else:
            self.mymotor.setVelocity(0.0)