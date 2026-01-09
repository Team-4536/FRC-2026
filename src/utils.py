from math import atan2, copysign, cos, hypot, sin
from subsystem import Subsystem
from typing import Tuple
from wpilib import getTime


class TimeData(Subsystem):
    def __init__(self) -> None:
        self.init()

    def init(self) -> None:
        time = getTime()

        self.dt = 0
        self.timeSinceInit = 0
        self.prevTime = time
        self.initTime = time

    def periodic(self) -> None:
        time = getTime()

        self.dt = time - self.prevTime
        self.timeSinceInit = time - self.initTime
        self.prevTime = time

    def disable(self) -> None:
        time = getTime()

        self.dt = 0
        self.timeSinceInit = time - self.initTime


class Scalar:
    def __init__(self, deadzone: float = 0.1, exponent: float = 1) -> None:
        self.deadzone = deadzone
        self.exponent = exponent

    def scale(self, input: float) -> float:
        if abs(input) <= self.deadzone:
            return 0
        else:
            delta = abs(input) - self.deadzone
            sign = copysign(1, input)
            return sign * (delta / (1 - self.deadzone)) ** self.exponent

    def setDeadzone(self, deadzone: float) -> None:
        self.deadzone = deadzone

    def setExponent(self, exponent: float) -> None:
        self.exponent = exponent

    def __call__(self, input: float) -> float:
        return self.scale(input)


class CircularScalar:
    def __init__(self, deadzone: float = 0.1, exponent: int = 1) -> None:
        self.scalar = Scalar(deadzone, exponent)

    def scale(self, x: float, y: float) -> Tuple[float, float]:
        if y == 0 and not x == 0:
            self.scalar.scale(x)
        elif x == 0 and not y == 0:
            self.scalar.scale(y)

        mag = hypot(x, y)
        mag = self.scalar.scale(mag)

        angle = atan2(y, x)

        x = mag * cos(angle)
        y = mag * sin(angle)

        return x, y

    def setDeadzone(self, deadzone: float) -> None:
        self.scalar.setDeadzone(deadzone)

    def setExponent(self, exponent: float) -> None:
        self.scalar.setExponent(exponent)

    def __call__(self, x: float, y: float) -> Tuple[float, float]:
        return self.scale(x, y)
