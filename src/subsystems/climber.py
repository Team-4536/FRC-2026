from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem


class Climber(Subsystem):
    def __init__(self, motorID: int):
        super().__init__()
        self.climberMotor: RevMotor = RevMotor(deviceID=motorID)
        self.climberMotor.configure(config=RevMotor.CLIMBER_CONFIG)
        # self.climbLimit =
        # self.climberLimit = self.climberMotor._ctrlr.getForwardLimitSwitch()
        # self.climbEncoder = self.climberMotor.getEncoder()
        # self.climbEncoder.setPosition(0)
        self.publishFloat("BAM: climber down throttle", -0.7)
        self.publishFloat("BAM: climber up throttle", 0.7)
        pass

    def phaseInit(self, robotState: RobotState) -> RobotState:

        self.climberMotor.setVoltage(0)

        # if self.climberLimit:
        #     self.climbEncoder.setPosition(0)

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:

        if robotState.climbDown:
            self.climberMotor.setThrottle(
                self.getFloat("BAM: climber down throttle", default=-0.7)
            )
            print("climbing up")

        elif robotState.climbUp:
            self.climberMotor.setThrottle(
                self.getFloat("BAM: climber up throttle", default=0.7)
            )
            print("climbing down")

        else:
            self.climberMotor.setThrottle(0)

        return robotState

    def robotPeriodic(self, robotState: RobotState) -> RobotState:
        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass
