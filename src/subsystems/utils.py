from math import atan2, copysign, cos, hypot, sin
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from typing import Tuple
from wpilib import getTime
from dataclasses import dataclass, fields, MISSING
from math import pi as PI, tau as TAU
from subsystems.robotState import BATTERY_VOLTS
import numpy as np
import math
from wpimath.geometry import Translation2d
from wpimath.units import (
    seconds,
    metersToFeet,
    radians,
    meters,
    inchesToMeters,
    revolutions_per_minute as RPM,
    meters_per_second as MPS,
)
import numpy as np

FIELD_WIDTH: meters = inchesToMeters(317.7)
FIELD_LEN: meters = inchesToMeters(651.2)


def lerp(x: float, y: float, t: float) -> float:
    return x + t * (y - x)


class TimeData(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        time = getTime()
        self.dt: seconds = 0
        self.timeSinceInit: seconds = 0
        self.timeSincePhaseInit: seconds = 0
        self.prevTime: seconds = time
        self.initTime: seconds = time
        self.phaseInitTime: seconds = time

    def phaseInit(self, robotState: RobotState) -> RobotState:
        time = getTime()
        self.timeSincePhaseInit = 0
        self.phaseInitTime = time

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        time = getTime()
        self.dt = time - self.prevTime
        self.timeSinceInit = time - self.initTime
        self.timeSincePhaseInit = time - self.phaseInitTime
        self.prevTime = time

        return robotState

    def disabled(self) -> None:
        time = getTime()
        self.dt = time - self.prevTime
        self.timeSinceInit = time - self.initTime
        self.timeSincePhaseInit = 0
        self.prevTime = time

    def publish(self) -> None:
        self.publishFloat("delta_time", self.dt)
        self.publishFloat("time_since_init", self.timeSinceInit)
        self.publishFloat("time_since_phase_init", self.timeSincePhaseInit)


class Scalar:
    def __init__(
        self, deadzone: float = 0.1, exponent: float = 1, magnitude: float = 1
    ) -> None:
        self.deadzone = deadzone
        self._exponent = exponent
        self._magnitude = magnitude

    def scale(self, x: float) -> float:
        if abs(x) <= self.deadzone:
            return 0
        else:
            delta = abs(x) - self.deadzone
            sign = self._magnitude * copysign(1, x)
            return float(sign * (delta / self._scale) ** self.exponent)

    def __call__(self, x: float) -> float:
        return self.scale(x)

    @property
    def deadzone(self) -> float:
        return self._deadzone

    @deadzone.setter
    def deadzone(self, deadzone: float) -> None:
        self._deadzone = abs(deadzone)
        self._scale = 1 - deadzone

    def setDeadzone(self, deadzone: float) -> None:
        self.deadzone = deadzone

    @property
    def exponent(self) -> float:
        return self._exponent

    @exponent.setter
    def exponent(self, exponent: float) -> None:
        self._exponent = abs(exponent)

    def setExponent(self, exponent: float) -> None:
        self.exponent = exponent

    @property
    def magnitude(self) -> float:
        return self._magnitude

    @magnitude.setter
    def magnitude(self, magnitude: float) -> None:
        self._magnitude = abs(magnitude)

    def setMagnitude(self, magnitude: float) -> None:
        self.magnitude = magnitude


class CircularScalar:
    def __init__(
        self, deadzone: float = 0.1, exponent: float = 1, magnitude: float = 1
    ) -> None:
        self.linearScalar = Scalar(deadzone, exponent, magnitude)

    def scale(self, *, x: float, y: float) -> Tuple[float, float]:
        if y == 0 and not x == 0:
            return self.linearScalar(x), y
        elif x == 0 and not y == 0:
            return x, self.linearScalar(y)

        mag = hypot(x, y)
        mag = self.linearScalar(mag)

        angle = atan2(y, x)

        x = mag * cos(angle)
        y = mag * sin(angle)

        return x, y

    def __call__(self, *, x: float, y: float) -> Tuple[float, float]:
        return self.scale(x=x, y=y)

    @property
    def deadzone(self) -> float:
        return self.linearScalar.deadzone

    def setDeadzone(self, deadzone: float) -> None:
        self.linearScalar.setDeadzone(deadzone)

    @property
    def exponent(self) -> float:
        return self.linearScalar.deadzone

    def setExponent(self, exponent: float) -> None:
        self.linearScalar.setExponent(exponent)

    @property
    def magnitude(self) -> float:
        return self.linearScalar.magnitude

    def setMagnitude(self, magnitude: float) -> None:
        self.linearScalar.setMagnitude(magnitude)


def getTangentalVelocity(posFromCenter: Translation2d, speed: MPS) -> Translation2d:
    # TODO idk if this works
    tangentAngle: radians = math.atan(posFromCenter.y / posFromCenter.x) + PI / 2

    tangentVelocity: Translation2d = Translation2d(
        speed * math.cos(tangentAngle), speed * math.sin(tangentAngle)
    )

    return tangentVelocity


def getContributedRotation(tangentVel: Translation2d, angle: radians) -> MPS:
    tangentAngle = tangentVel.angle().radians()
    speed = tangentVel.distance(Translation2d())
    contributedVector: radians = math.cos(tangentAngle - angle)

    return speed * contributedVector


def RPMToMPS(speed: RPM, circ: meters) -> MPS:
    return speed / 60 * circ


def MPSToRPM(speed: MPS, circ: meters) -> RPM:
    return speed / circ * 60


def scaleTranslation2D(translation: Translation2d, scalar) -> Translation2d:

    angle = translation.angle().radians()
    hyp = translation.distance(Translation2d())
    xScale = hyp * math.cos(angle)
    yScale = hyp * math.sin(angle)

    return Translation2d(xScale, yScale)


def wrapAngle(angle: radians) -> radians:

    # mod only returns positive like a bum
    if angle == 0:
        return 0

    wrappedAngle: radians = angle % (TAU * np.sign(angle))
    return wrappedAngle


def RPMToVolts(rpm: RPM, maxRPM) -> float:

    return rpm / (maxRPM / BATTERY_VOLTS)
