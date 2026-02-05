from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState


class Climbing(Subsystem): 
    def __init__(self):
        self.MotorArm1 = RevMotor(deviceID=15) #identifying the motors of the arms 
        self.MotorArm2 = RevMotor(deviceID=16)
        
          #clarify how many arms and how we want them to climb
          #they cannot climb at the samw time
    
    def init(self):
       pass

    def periodic(self, ds:RobotState):
        if ds.extended: #if the arm is extented to the maximum amount 
            if ds.buttonUp:
                self.MotorArm1.setVelocity(0)
                self.MotorArm2.setVelocity(0)
            elif ds.buttonDown:
                self.MotorArm1.setVelocity(-1)
                self.MotorArm2.setVelocity(-1)
            else:
               self.MotorArm1.setVelocity(0)
               self.MotorArm2.setVelocity(0)     
        elif ds.contracted: #if ars are contracted to the max amount 
            if ds.buttonUp:
                self.MotorArm1.setVelocity(1)
                self.MotorArm2.setVelocity(1)
            elif ds.buttonDown:
                self.MotorArm1.setVelocity(0)
                self.MotorArm2.setVelocity(0)
            else:
               self.MotorArm1.setVelocity(0)
               self.MotorArm2.setVelocity(0)     
        elif ds.buttonUp: #if not at a limit just listen to the buttoms being pressed
            self.MotorArm1.setVelocity(1)
            self.MotorArm2.setVelocity(1)
        elif ds.buttonDown:
            self.MotorArm1.setVelocity(-1)
            self.MotorArm2.setVelocity(-1)
        else:
            self.MotorArm1.setVelocity(0)
            self.MotorArm2.setVelocity(0)

            


    def disabled(self) -> None:
        self.MotorArm1.setVelocity(0)
        self.MotorArm1.setVelocity(0)

    def publish(self):
        self.MotorArm1.setVelocity()
        self.MotorArm2.setVelocity()
        pass