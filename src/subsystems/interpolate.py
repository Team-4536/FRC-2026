import numpy
import bisect
from typing import List

def calcX(x: List[float], d: float) -> List[int]:

    x_low = bisect.bisect_left(x, d)
    x_high = bisect.bisect_right(x, d)

    if x_high == x_low:
        return [(x_high), (x_high+1)]

    return [x_low, x_high]

x = [1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0]

vel = numpy.array([2,3,4,5,6,7,8,9,10,11])

pitch = numpy.array([3,4,5,6,7,8,9,10,11,12])

d = 4.7

xVar = calcX(x, d)




t = (d-xVar[0])/(xVar[1] - xVar[0])

# print(t)

print(vel[xVar[0]]-1)
print(vel[xVar[1]]-1)

velocity = ((vel[xVar[0]-1]*(1-t))+(vel[xVar[1]-1]*(t)))
robotPitch = ((pitch[xVar[0]-1]*(1-t))+(pitch[xVar[1]-1]*(t)))