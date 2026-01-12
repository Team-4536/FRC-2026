from mb.desiredState import DesiredState
from mb.subsystem import Subsystem
from typing import List, Optional
from warnings import warn
from wpilib import CAN


NONE: int = 255


class LEDSignals(Subsystem):
    def __init__(self, deviceID: int) -> None:
        super().__init__()

        self.apiID = 0
        self.can = CAN(deviceID)

    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        b1: int = int(ds.fieldSpeeds.vx * 256 / 6)
        self.publishDouble(f"_", float(ds.fieldSpeeds.vx))
        b2: int = int(ds.fieldSpeeds.vy * 256 / 6)
        self.publishDouble(f"_", float(ds.fieldSpeeds.vy))

        self.update(b1=b1, b2=b2)

    def disabled(self) -> None:
        self.update(b8=0)  # EXAMPLE

    def publish(self) -> None:
        pass

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
        rawBytes: List[Optional[int]] = [b1, b2, b3, b4, b5, b6, b7, b8]
        bytes: List[int] = [b if b is not None else NONE for b in rawBytes]

        try:
            byteArray = bytearray(bytes)
            self.can.writePacket(byteArray, self.apiID)

            self.publishString("CAN_WRITE_PACKET_STATUS", "None")

            for i, b in enumerate(bytes):
                if rawBytes[i] is not None:
                    self.publishInt(f"byte_{i + 1}", b)
                else:
                    self.publishInt(f"byte_{i + 1}", -1)
        except (ValueError, Exception) as e:
            warn(f"Error: {e}")
            self.publishString("CAN_WRITE_PACKET_STATUS", str(e))
