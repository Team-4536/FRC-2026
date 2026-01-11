from desiredState import DesiredState
from math import atan2, copysign, cos, hypot, sin
from subsystem import Subsystem
from typing import Tuple
from wpilib import getTime


class TimeData(Subsystem):
    def __init__(self) -> None:
        self.init()

    def init(self) -> None:
        time = getTime()

        self.dt: float = 0
        self.timeSinceInit: float = 0
        self.prevTime = time
        self.initTime = time

    def periodic(self, ds: DesiredState) -> None:
        time = getTime()

        self.dt = time - self.prevTime
        self.timeSinceInit = time - self.initTime
        self.prevTime = time

    def disable(self) -> None:
        time = getTime()

        self.dt = 0
        self.timeSinceInit = time - self.initTime


class Scalar:
    def __init__(self, deadzone: float = 0.1, exponent: float = 1, magnitude: float = 1) -> None:
        self.deadzone = deadzone
        self._exponent = exponent
        self._magnitude = magnitude

    def scale(self, input: float) -> float:
        if abs(input) <= self.deadzone:
            return 0
        else:
            delta = abs(input) - self.deadzone
            sign = self._magnitude * copysign(1, input)
            return float(sign * (delta / self._scale) ** self.exponent)

    def __call__(self, input: float) -> float:
        return self.scale(input)

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
    def __init__(self, deadzone: float = 0.1, exponent: float = 1, magnitude: float = 1) -> None:
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
