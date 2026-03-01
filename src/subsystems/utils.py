from math import atan2, copysign, cos, hypot, pi, sin, tau
from numpy import sign
from phoenix6.units import volt as voltage
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from typing import Tuple
from wpilib import Timer
from wpimath.geometry import Translation2d
from wpimath.units import (
    inchesToMeters,
    meters_per_second,
    meters,
    radians,
    revolutions_per_minute,
    seconds,
)

FIELD_WIDTH: meters = inchesToMeters(317.7)
FIELD_LEN: meters = inchesToMeters(651.2)
BATTERY_VOLTS: float = 12


class TimeData(Subsystem):
    _matchTime: Timer
    _phaseTime: Timer

    _prevTime: seconds = 0
    _deltaTime: seconds = 0

    def __init__(self) -> None:
        super().__init__()

        self._matchTime = Timer()
        self._phaseTime = Timer()

        self._matchTime.start()

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self._phaseTime.reset()
        self._phaseTime.start()
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        time = self._matchTime.get()
        self._deltaTime = time - self._prevTime
        self._prevTime = time
        return robotState

    def disabled(self) -> None:
        self._phaseTime.stop()
        self._phaseTime.reset()

    def publish(self) -> None:
        self.publishFloat("delta_time", self.dt)
        self.publishFloat("time_since_init", self.timeSinceInit)
        self.publishFloat("time_since_phase_init", self.timeSincePhaseInit)

    @property
    def dt(self) -> seconds:
        return self._deltaTime

    @property
    def timeSinceInit(self) -> seconds:
        return self._matchTime.get()

    @property
    def timeSincePhaseInit(self) -> seconds:
        return self._phaseTime.get()


timeData: TimeData = TimeData()


class _MatchTime:
    @property
    def dt(self) -> seconds:
        return timeData.dt

    @property
    def timeSinceInit(self) -> seconds:
        return timeData.timeSinceInit

    @property
    def timeSincePhaseInit(self) -> seconds:
        return timeData.timeSincePhaseInit


matchTime: _MatchTime = _MatchTime()


class Scalar:
    _deadzone: float
    _exponent: float
    _magnitude: float
    _scale: float

    def __init__(
        self, deadzone: float = 0.1, exponent: float = 1, magnitude: float = 1
    ) -> None:
        self.deadzone = deadzone
        self.exponent = exponent
        self.magnitude = magnitude

    def scale(self, x: float) -> float:
        if abs(x) <= self._deadzone:
            return 0
        else:
            delta = abs(x) - self._deadzone
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
        self._deadzone = deadzone

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
    _linearScalar: Scalar

    def __init__(
        self, deadzone: float = 0.1, exponent: float = 1, magnitude: float = 1
    ) -> None:
        self._linearScalar = Scalar(deadzone, exponent, magnitude)

    def scale(self, *, x: float, y: float) -> Tuple[float, float]:
        if y == 0 and not x == 0:
            return self._linearScalar(x), y
        elif x == 0 and not y == 0:
            return x, self._linearScalar(y)

        mag = hypot(x, y)
        mag = self._linearScalar(mag)

        angle = atan2(y, x)

        x = mag * cos(angle)
        y = mag * sin(angle)

        return x, y

    def __call__(self, *, x: float, y: float) -> Tuple[float, float]:
        return self.scale(x=x, y=y)

    @property
    def deadzone(self) -> float:
        return self._linearScalar.deadzone

    def setDeadzone(self, deadzone: float) -> None:
        self._linearScalar.setDeadzone(deadzone)

    @property
    def exponent(self) -> float:
        return self._linearScalar.exponent

    def setExponent(self, exponent: float) -> None:
        self._linearScalar.setExponent(exponent)

    @property
    def magnitude(self) -> float:
        return self._linearScalar.magnitude

    def setMagnitude(self, magnitude: float) -> None:
        self._linearScalar.setMagnitude(magnitude)


def lerp(x: float, y: float, t: float) -> float:
    return x + t * (y - x)


def getTangentAngle(posFromCenter: Translation2d) -> radians:
    # TODO idk if this works
    tangentAngle: radians = atan2(posFromCenter.y, posFromCenter.x) + pi / 2

    return tangentAngle


def getContributedRotation(
    tangentAngle: radians, angle: radians, speed: meters_per_second
) -> meters_per_second:
    contributedVector: float = cos(tangentAngle - angle)

    return speed * contributedVector


def RPMToMPS(speed: revolutions_per_minute, circ: meters) -> meters_per_second:
    return speed / 60 * circ


def MPSToRPM(speed: meters_per_second, circ: meters) -> revolutions_per_minute:
    return speed / circ * 60


def scaleTranslation2D(translation: Translation2d, scalar: float) -> Translation2d:
    angle = translation.angle().radians()
    hyp = translation.distance(Translation2d())
    xScale = hyp * cos(angle)
    yScale = hyp * sin(angle)

    return Translation2d(xScale, yScale)


def wrapAngle(angle: radians) -> radians:
    if angle == 0:
        return 0

    return angle % (tau * sign(angle))


def RPMToVolts(rpm: revolutions_per_minute, maxRPM: revolutions_per_minute) -> voltage:
    return rpm / (maxRPM / BATTERY_VOLTS)
