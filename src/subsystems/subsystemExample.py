from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState

class SubsystemExample(Subsystem): #to run in subsystem manager 

    isDisabled: bool = True # we arent running these through the robot so its here instead of robotState
    isStarted: bool = False 
    message: str = ""
    origMessage: str = None 

    def __init__(self, message: str = "hello there"): #init the og message basd on the variable passed 
        super().__init__() #to init the parent subsystem class
        self.origMessage = message       

    def phaseInit(self) -> None: #doesnt run when switching to disabled 
        if(self.isDisabled == True and self.isStarted == False): #runs for every chage in phase
            self.isStarted = True 
            self.isDisabled = False
            self.message = self.origMessage            

    def periodic(self, rs: RobotState) -> RobotState: #stays running hen not disabled
        self.message = self.message + "."
        return rs

    def disabled(self) -> None:
        if(self.isStarted == True and self.isDisabled == False):
            self.isStarted = False
            self.isDisabled = True
            self.message = "done"

    def publish(self):
        self.publishString("message", self.message)
