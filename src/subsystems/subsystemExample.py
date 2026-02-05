from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from ntcore import NetworkTableInstance

# motor vars do not include real device id at this time (1-13)
# all RPM values must be tested on the robot and adjusted acoringly


class SubsystemExample(Subsystem):

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")


        self.subsystemMotor = RevMotor(deviceID=8)
       

    def init(self) -> None:
        pass

    def periodic(self, ds: RobotState) -> None:
        self.table.putNumber("revMotorr", ds.revMotor)
        

        if ds.revMotor > 0.1:
            self.subsystemMotor.setVelocity(120)
        else:
            self.disabled()

    def disabled(self) -> None:
        self.subsystemMotor.setVelocity(0)

    def publish(self):
        pass
