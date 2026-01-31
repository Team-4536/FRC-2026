from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.desiredState import DesiredState


class Climbing(Subsystem): 
    def __init__(self):
        self.subsystemMotor1 = RevMotor(deviceID=15)
        self.subsystemMotor2 = RevMotor(deviceID=16)
        
          
    
    def init(self):
       pass

    def periodic(self, ds:DesiredState):
        if ds.extended:
            if ds.buttonUp:
                self.subsystemMotor1.setVelocity(0)
                self.subsystemMotor2.setVelocity(0)
            elif ds.buttonDown:
                self.subsystemMotor1.setVelocity(-1)
                self.subsystemMotor2.setVelocity(-1)
            else:
               self.subsystemMotor1.setVelocity(0)
               self.subsystemMotor1.setVelocity(0)     
        elif ds.contracted:
            if ds.buttonUp:
                self.subsystemMotor1.setVelocity(1)
                self.subsystemMotor2.setVelocity(1)
            elif ds.buttonDown:
                self.subsystemMotor1.setVelocity(0)
                self.subsystemMotor2.setVelocity(0)
            else:
               self.subsystemMotor1.setVelocity(0)
               self.subsystemMotor1.setVelocity(0)     
        elif ds.buttonUp:
            self.subsystemMotor1.setVelocity(1)
            self.subsystemMotor2.setVelocity(1)
        elif ds.buttonDown:
            self.subsystemMotor1.setVelocity(-1)
            self.subsystemMotor2.setVelocity(-1)
        else:
            self.subsystemMotor1.setVelocity(0)
            self.subsystemMotor2.setVelocity(0)

            


    def disabled(self) -> None:
        self.subsystemMotor1.setVelocity(0)
        self.subsystemMotor2.setVelocity(0)

    def publish(self):
        pass