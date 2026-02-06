from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState

# all RPM values must be tested on the robot and adjusted acoringly

class SubsystemExample(Subsystem):

    def __init__(self, motorDevID: int):
        super().__init__(self)
        self.subsystemMotor = RevMotor(deviceID=motorDevID) 
       

    def phaseInit(self) -> None:
        pass

    def periodic(self, rs: RobotState) -> RobotState:
        self.desMotorState = rs.desMotorSpeed
        if rs.desMotorSpeed > 0.1:
            self.subsystemMotor.setVelocity(120)
        else:
            self.disabled()

        return rs

    def disabled(self) -> None:
        self.subsystemMotor.setVelocity(0)

    def publish(self):
        self.publishFloat("revMotor", self.desMotorState)
