from motor import RevMotor
from inputs import Inputs
from desiredState import DesiredState
from phoenix6.hardware import CANcoder
from wpimath.units import rotationsToRadians, degreesToRadians, rotationsToDegrees
import math





class Turret():
    #cancoder more like cantcoder
    def __init__(self):
        self.turretMotor = RevMotor(12) #get right ID, motor for turning horizontally
        self.turretMotorVertical = RevMotor(17) #
        self.turretEncoder = self.turretMotor.getEncoder()
        self.turretEncoderVertical = CANcoder(25) #sensor for the vertical motor
        self.gear= 12 #TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation

    def init(self):
        #self.turretPosition = rotationsToRadians(self.turretEncoder.get_absolute_position() * self.gear) #returns number of rotations in radians
        self.home = False 
        

    def periodic(self, desiredState: DesiredState, camRot):
        #camrot is in degrees

        if self.home == False:
            self.turretMotor.setVelocity(5) # change the speed to desired speed

            
        if desiredState.limitSwitchA == True:
                self.turretEncoder.setPosition(0)
                self.home == True

        self.turretRotation = rotationsToDegrees(self.turretEncoder * self.gear)
        self.desiredRotation = desiredState.turretSetPoint #

        if not self.desiredRotation == -1:
            rot = degreesToRadians(self.desiredRotation)
            turretMotorGoal = (self.turretRotation - rot) * self.gear
            self.turretMotor.setPosition(rot)


        
        self.turretMotor.setVelocity(self.turretMotor, desiredState.turretSpeed)
        self.turretMotorVertical.setVelocity(self.turretMotor, desiredState.turretSpeed)
            
        pass
    def disabled(self):
        pass