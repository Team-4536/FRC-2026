# from subsystems.subsystem import Subsystem
# from subsystems.motor import RevMotor
# from subsystems.robotState import RobotState

# class Turret(Subsystem):
#     def __init__(self):
#         self.subsystemMotor3 = RevMotor(deviceID=13)


#     def init(self):
#         pass

    # def periodic(self, ds:RobotState):
    #     if ds.limitLeft:
    #         if ds.buttonLeft:
    #             self.subsystemMotor3.setVelocity(0)
    #         elif ds.buttonRight:  
    #             self.subsystemMotor3.setVelocity(1)
    #         else: 
    #             self.subsystemMotor3.setVelocity(0) 
    #     elif ds.limitRight:
    #         if ds.buttonLeft:
    #             self.subsystemMotor3.setVelocity(-1)
    #         elif ds.buttonRight:  
    #             self.subsystemMotor3.setVelocity(0)
    #         else: 
    #             self.subsystemMotor3.setVelocity(0) 
    #     else:          
    #         if ds.buttonLeft:
    #             self.subsystemMotor3.setVelocity(-1)
    #         elif ds.buttonRight:
    #             self.subsystemMotor3.setVelocity(1)    
    #         else:
    #             self.subsystemMotor3.setVelocity(0) 

    # def disabled(self):
    #     self.subsystemMotor3.setVelocity(0)

    # def publish(self):
    #     pass
