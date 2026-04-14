from phoenix6.units import volt as voltage
from subsystems.motor import RevMotor
from subsystems.robotState import ClimberState, RobotState
from subsystems.subsystem import Subsystem


class Climber(Subsystem):
    climberMotor: RevMotor

    CLIMB_SPEED: voltage = 8.5

    def __init__(self, motorID: int) -> None:
        super().__init__()

        self.climberMotor = RevMotor(deviceID=motorID)
        self.climberMotor.configure(config=RevMotor.CLIMBER_CONFIG)

    def phaseInit(self, robotState: RobotState) -> None:
        self.climberMotor.stopMotor()

    def periodic(self, robotState: RobotState) -> None:
        match robotState.climbState:
            case ClimberState.CLIMB_UP:
                self.climberMotor.setVoltage(self.CLIMB_SPEED)
            case ClimberState.CLIMB_DOWN:
                self.climberMotor.setVoltage(-self.CLIMB_SPEED)
            case ClimberState.DISABLED:
                self.climberMotor.stopMotor()

    def disabled(self) -> None:
        self.climberMotor.stopMotor()

    def publish(self) -> None:
        pass
