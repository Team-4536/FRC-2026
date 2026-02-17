from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from typing import Optional
from warnings import warn
from wpilib import CAN

NONE: int = 255


class LEDSignals(Subsystem):
    def __init__(self, deviceID: int) -> None:
        super().__init__()

        self.apiID = 0
        self.can = CAN(deviceID)

    def phaseInit(self, robotstate: RobotState) -> None:
        pass

    def periodic(self, robotState: RobotState) -> RobotState:
        return robotState

    def disabled(self) -> None:
        self.update(b8=0)  # EXAMPLE

    def update(
        self,
        *,
        b1: Optional[int] = None,
        b2: Optional[int] = None,
        b3: Optional[int] = None,
        b4: Optional[int] = None,
        b5: Optional[int] = None,
        b6: Optional[int] = None,
        b7: Optional[int] = None,
        b8: Optional[int] = None,
    ) -> None:
        rawBytes = [b1, b2, b3, b4, b5, b6, b7, b8]
        bytes = [b if b is not None else NONE for b in rawBytes]

        try:
            byteArray = bytearray(bytes)
            self.can.writePacket(byteArray, self.apiID)

            self.publishString("CAN_WRITE_PACKET_STATUS", "None")

            for i, b in enumerate(bytes):
                if rawBytes[i] is not None:
                    self.publishInteger(f"byte_{i + 1}", b)
                else:
                    self.publishInteger(f"byte_{i + 1}", -1)
        except (ValueError, Exception) as e:
            warn(f"Error: {e}")
            self.publishString("CAN_WRITE_PACKET_STATUS", str(e))
