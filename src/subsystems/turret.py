from motor import RevMotor
from rev import SparkRelativeEncoder
from inputs import Inputs
from desiredState import DesiredState
from phoenix6.hardware import CANcoder
from wpimath.units import rotationsToRadians, degreesToRadians, rotationsToDegrees
import math
from math import tau as TAU
import navx
import numpy as np
from wpimath.geometry import Pose3d


MAX_ROTATION = 3 * math.pi / 2
TURRET_GAP = math.tau - MAX_ROTATION
GEARING = 12  # TODO: find correct drive gearing, gear for both motors. means you turn 12 times to make a full rotation


class Turret:
    # cancoder more like cantcoder
    def __init__(self):
        self.turretMotor = RevMotor(12)  # get right ID, motor for turning horizontally
        self.setPoint = 0  # in relation to the field
        self.turretMotorVertical = RevMotor(17)  #
        self.turretEncoder = CANcoder(
            20
        )  # get right ID, sensor for the horizontal motor
        self.turretEncoderVertical = CANcoder(25)  # sensor for the vertical motor
        self.feildRelativePos = 0
        self.odom = TurretOdometry()

    def periodic(self, ds: DesiredState):
        # camrot is in degrees
        self.odom.updateWithEncoder(ds, self.turretEncoder)
        self.setPoint = ds.turretSetPoint
        self.targetPoint()
        self.maintainSetpoint(ds.yaw)  # these go last
        self.dontOverdoIt()
        self.turretMotor.setPosition(self.setPoint * GEARING)

    def disabled(self):
        pass

    def maintainSetpoint(self, robotYaw):
        self.setPoint -= self.odom.getGyroChange(
            robotYaw
        )  # modulo only returns positive

    def dontOverdoIt(self):
        if self.setPoint > MAX_ROTATION + TURRET_GAP / 2:
            self.setPoint = 0
        elif self.setPoint > MAX_ROTATION:
            self.setPoint = MAX_ROTATION

    def targetPoint(self, pointPose: Pose3d, robotPose: Pose3d):  # make velocity later

        xDiff = pointPose.X - robotPose.X
        yDiff = pointPose.Y - robotPose.Y
        self.setPoint = math.atan(xDiff / yDiff)


class TurretOdometry:
    def __init__(self):
        self.feildRelativeRot = 0
        self.wrappedFRR = 0  # wrapped field relative rotation
        self.pos: Pose3d = Pose3d()

    def updateWithEncoder(self, ds: DesiredState, encoder: SparkRelativeEncoder):
        roboYaw = ds.yaw
        turretRaw = encoder.getPosition()
        turretRot = rotationsToRadians(turretRaw) / GEARING
        self.wrappedFRR = (turretRot % TAU) * np.sign(turretRot)
        self.feildRelativeRot = roboYaw - turretRot

    def getGyroChange(self, yaw):
        wrappedRoboYaw = (yaw % TAU) * np.sign(yaw)

        return self.wrappedFRR - wrappedRoboYaw
