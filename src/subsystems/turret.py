from motor import RevMotor
from inputs import Inputs
from desiredState import DesiredState
from phoenix6.hardware import CANcoder
from wpimath.units import rotationsToRadians



class Turret():
    def __init__(self, motor: RevMotor, encoder: CANcoder):
        self.turretMotor = motor
        self.turretEncoder = encoder
        self.gear= 12/100 #TODO: find correct drive gearing
        pass
    def init(self):
        self.turretPosition = rotationsToRadians(self.turretEncoder.get_absolute_position() * self.gear)
        pass
    def periodic(self, desiredState: DesiredState):
        
        self.turretMotor.setVelocity(self.turretMotor, desiredState.turretSpeed)
        pass
    def disabled(self):
        pass