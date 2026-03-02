from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from rev import ServoHub, ServoHubConfig, ResetMode, ServoChannel



class climber(Subsystem):

    def __init__(self, climbMotorID: int):

        self.climbMotor = RevMotor(deviceID=climbMotorID)
        self.servoController = ServoHub(deviceID=1)

        self.config = ServoHubConfig()
        self.servoController.configure(self.config, resetMode=ResetMode.kNoResetSafeParameters)
        self.servoChannel = ServoChannel()

        self.supportServo = self.servoController.getServoChannel(channelId=)


        pass




    