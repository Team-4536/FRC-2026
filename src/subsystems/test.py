from wpimath.units import (
    degreesToRadians,
    inchesToMeters,
)
from wpimath.geometry import Translation2d

print(degreesToRadians(45))
print(inchesToMeters(Translation2d(11, 11).norm()))
