from motor import RevMotor
from inputs import Inputs
from desiredState import DesiredState
from phoenix6.hardware import CANcoder
from wpimath.units import rotationsToRadians, degreesToRadians





class Turret():
    #cancoder more like cantcoder
    def __init__(self):
        self.turretMotor = RevMotor(12) #get right ID, motor for turning horizontally
        self.turretMotorVertical = RevMotor(17) #
        self.turretEncoder = CANcoder(20) #get right ID
        self.turretEncoderVertical = CANcoder(25)
        self.gear= 12/100 #TODO: find correct drive gearing
        pass
    def init(self):
        self.turretPosition = rotationsToRadians(self.turretEncoder.get_absolute_position() * self.gear)
        pass
    def periodic(self, desiredState: DesiredState, camRot):
        self.turretRotation = degreesToRadians(camRot % 360)

        self.desiredRotation = desiredState.turretSetPoint

        if not self.desiredRotation == -1:
            rot = degreesToRadians(self.desiredRotation)
            turretMotorGoal = (self.desiredRotation - rot) * self.gear
            self.turretMotor.setPosition(turretMotor)


        
        self.turretMotor.setVelocity(self.turretMotor, desiredState.turretSpeed)
        self.turretMotorVertical.setVelocity(self.turretMotor, desiredState.turretSpeed)

        pass
    def disabled(self):
        pass