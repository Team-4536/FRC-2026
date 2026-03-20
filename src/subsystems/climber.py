from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState
from subsystems.motor import RevMotor

class CLimber(Subsystem):
    def __init__(self):
        super().__init__()
        self.climberMotor: RevMotor = RevMotor(deviceID=15)
        self.climberLimit = self.climberMotor._ctrlr.getForwardLimitSwitch()
        self.climbEncoder = self.climberMotor.getEncoder()
        self.climbEncoder.setPosition(0)
        pass

    def phaseInit(
        self, robotState: RobotState
    ) -> RobotState:
        
        self.climberMotor.setVoltage(0)

        if self.climberLimit:
            self.climbEncoder.setPosition(0)
        
        return robotState

    def periodic(
        self, robotState: RobotState
    ) -> RobotState:

        if self.climberLimit:
            self.climbEncoder.setPosition(0)
        
        if self.climbEncoder.getPosition() >= 0:
            if robotState.climbDown:
                self.climberMotor.setVoltage(-3)

        
        if robotState.climbUp:
            self.climberMotor.setVoltage(3)


        return robotState

    def robotPeriodic(self, robotState: RobotState) -> RobotState:
        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


