from subsystem import Subsystem
from subsystems.desiredState import DesiredState
from typing import Optional
from warnings import warn
from wpilib import CAN


class LEDSignals(Subsystem):
    def __init__(self, deviceID: int) -> None:
        super().__init__()

        self.apiID = 0
        self.can = CAN(deviceID)

    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        pass

    def publish(self) -> None:
        pass

    def update(
        self,
        b1: Optional[int] = None,
        b2: Optional[int] = None,
        b3: Optional[int] = None,
        b4: Optional[int] = None,
        b5: Optional[int] = None,
        b6: Optional[int] = None,
        b7: Optional[int] = None,
        b8: Optional[int] = None,
    ) -> None:
        bytes = [b for b in [b1, b2, b3, b4, b5, b6, b7, b8] if b is not None]

        try:
            byteArray = bytearray(bytes)
            self.can.writePacket(byteArray, self.apiID)
        except (ValueError, Exception) as e:
            warn(f"Error: {e}")

    def disabled(self) -> None:
        self.update(b8=0)  # EXAMPLE
