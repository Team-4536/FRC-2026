from enum import Enum
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem


class Climber(Subsystem):
    # defines all motors that are used in the subsystem
    def __init__(self, MotorID: int):
        super().__init__()
        self.Motor = RevMotor(deviceID=MotorID)
        self.Button = null  # TODO map button to inputs

    def periodic(self, robotState: RobotState):
        if self.Button:
            self.Motor.setThrottle(5)
        else:
            self.Motor.setThrottle(0)

    # values when the robot is disabled
    def disabled(self):
        self.Motor.setThrottle(0)

    # everything for tables and sims except it crashes
    def publish(self):
        self.publishFloat("Climber Motor Speed", 0)
