from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState
from subsystems.motor import RevMotor

class Climber(Subsystem):
    def __init__(self, motorID: int):
        super().__init__()
        self.climberMotor: RevMotor = RevMotor(deviceID=motorID)
        self.climberMotor.configure(config=RevMotor.CLIMBER_CONFIG)
        # self.climberLimit = self.climberMotor._ctrlr.getForwardLimitSwitch()
        # self.climbEncoder = self.climberMotor.getEncoder()
        # self.climbEncoder.setPosition(0)
        pass

    def phaseInit(
        self, robotState: RobotState
    ) -> RobotState:
        
        self.climberMotor.setVoltage(0)

        # if self.climberLimit:
        #     self.climbEncoder.setPosition(0)
        
        return robotState

    def periodic(
        self, robotState: RobotState
    ) -> RobotState:

    
   
        if robotState.climbDown:
                self.climberMotor.setVoltage(-3)

        
        elif robotState.climbUp:
            self.climberMotor.setVoltage(3)

        else:
            self.climberMotor.setVoltage(0)


        return robotState

    def robotPeriodic(self, robotState: RobotState) -> RobotState:
        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


