from math import atan2, copysign, cos, hypot, sin
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from typing import Tuple
from wpilib import getTime
from wpimath.units import seconds


def lerp(x: float, y: float, t: float) -> float:
    return x + t * (y - x)


class TimeData(Subsystem):
    dt: seconds = 0
    timeSinceInit: seconds = 0
    timeSincePhaseInit: seconds = 0
    prevTime: seconds
    initTime: seconds
    phaseInitTime: seconds

    def __init__(self) -> None:
        super().__init__()

        time = getTime()
        self.prevTime = time
        self.initTime = time
        self.phaseInitTime = time

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
    _deadzone: float
    _exponent: float
    _magnitude: float
    _scale: float

    def __init__(
        self, deadzone: float = 0.1, exponent: float = 1, magnitude: float = 1
    ) -> None:
        self._deadzone = deadzone
        self._exponent = exponent
        self._magnitude = magnitude

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
