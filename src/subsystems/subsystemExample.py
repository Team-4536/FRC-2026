from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState

class SubsystemExample(Subsystem):

    isDisabled: bool = True
    isStarted: bool = False
    message: str = ""
    origMessage: str = None

    def __init__(self, message: str = "helloWorld"):
        super().__init__()
        self.origMessage = message
       

    def phaseInit(self) -> None:
        if(self.isDisabled == True and self.isStarted == False):
            self.isStarted = True 
            self.isDisabled = False
            self.message = self.origMessage
           

    def periodic(self, rs: RobotState) -> RobotState:
        self.message = self.message + "."
        return rs

    def disabled(self) -> None:
        if(self.isStarted == True and self.isDisabled == False):
            self.isStarted = False
            self.isDisabled = True
            self.message = "done"

    def publish(self):
        self.publishString("message", self.message)
