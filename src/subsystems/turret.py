from motor import RevMotor
from inputs import Inputs
from desiredState import DesiredState
from phoenix6.hardware import CANcoder
from wpimath.units import rotationsToRadians, degreesToRadians, rotationsToDegrees
import math
import navx
import numpy as np

MAX_ROTATION = 3 * math.pi / 2
TURRET_GAP = math.tau - MAX_ROTATION


class Turret:
    # cancoder more like cantcoder
    def __init__(self):
        self.turretMotor = RevMotor(12)  # get right ID, motor for turning horizontally
        self.setPoint = 0  # in relation to the field
        self.lastGyroPos = 0  # for calculating change
        self.turretMotorVertical = RevMotor(17)  #
        self.turretEncoder = CANcoder(
            20
        )  # get right ID, sensor for the horizontal motor
        self.turretEncoderVertical = CANcoder(25)  # sensor for the vertical motor
        self.gear = 12  # TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation

    def periodic(self, ds: DesiredState):
        # camrot is in degrees
        self.setPoint = ds.turretSetPoint
        self.maintainSetpoint(ds.yaw)
        self.dontOverdoIt()
        self.turretMotor.setPosition(self.setPoint * self.gear)

    def disabled(self):
        pass

    def getGyroChange(self, robotYaw):
        change = robotYaw - self.lastGyroPos
        self.lastGyroPos = robotYaw
        return change

    def maintainSetpoint(self, robotYaw):
        self.setPoint -= self.getGyroChange(robotYaw)

    def dontOverdoIt(self):
        if self.setPoint > MAX_ROTATION + TURRET_GAP / 2:
            self.setPoint = 0
        elif self.setPoint > MAX_ROTATION:
            self.setPoint = MAX_ROTATION
