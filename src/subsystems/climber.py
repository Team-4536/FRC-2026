from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from rev import ServoHub, ServoHubConfig, ResetMode, ServoChannel
from subsystems.robotState import RobotState



class Climber(Subsystem):

    def __init__(self, climbMotorID: int):
        self.climbMotor = RevMotor(deviceID=climbMotorID)
        self.climbMotor.configure(config=RevMotor.DRIVE_CONFIG)
        self.climbEncoder = self.climbMotor.getEncoder()

        self.servoController = ServoHub(deviceID=1)
        self.config = ServoHubConfig()
        self.servoController.configure(self.config, resetMode=ResetMode.kNoResetSafeParameters)

        self.supportServo = self.servoController.getServoChannel(channelId=ServoChannel.ChannelId.kChannelId0) #TODO: get correct
        self.hookServo = self.servoController.getServoChannel(channelId=ServoChannel.ChannelId.kChannelId1) #TODO: get correct
        pass

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self.climberPos = 0
        self.state = 0
        self.downState = 0
        self.climbEncoder.setPosition(0)
        self.supportServo.setPulseWidth(500)
        self.hookServo.setPulseWidth(500)

        return robotState
    
    def periodic(self, robotState: RobotState) -> RobotState:
        
        if robotState.climb:
            if self.state == 0:
                self.supportServo.setPulseWidth(2500)
                self.hookServo.setPulseWidth(2500)
                self.state = 1

            elif self.state == 1:
                self.climbMotor.setVelocity(60)
                self.climberPos = self.climbEncoder.getPosition()
                
                if self.climberPos > 5:
                    self.climbMotor.setVelocity(0)
                    self.state = 2

            elif self.state == 3:
                self.climbMotor.setVelocity(-60)
                self.climberPos = self.climbEncoder.getPosition()

                if self.climberPos <0.5:
                    self.climbMotor.setVelocity(0)
                    self.state = 4

            elif self.state == 4:
                pass


        
        elif robotState.downClimb:
                
                if self.climberPos < 5:
                    self.climbMotor.setVelocity(60)
                    self.climberPos = self.climbEncoder.getPosition()
                elif self.hookServo.getPulseWidth() == 2500:
                    self.supportServo.setPulseWidth(500)
                    self.hookServo.setPulseWidth(500)
                elif self.climberPos > 5:
                    self.climbMotor.setVelocity(-60)

                else:
                    self.state = 0
                    

            
       

        return robotState




    