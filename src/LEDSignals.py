from subsystem import Subsystem
from wpilib import CAN


class LEDSignals(Subsystem):
    def __init__(self, deviceId: int) -> None:
        self.API_ID = 0

        self.can: CAN = CAN(deviceId)

    def init(self) -> None:
        pass

    def periodic(self) -> None:
        pass

    def update(
        self,
        b1: bytearray = None,
        b2: bytearray = None,
        b3: bytearray = None,
        b4: bytearray = None,
        b5: bytearray = None,
        b6: bytearray = None,
        b7: bytearray = None,
        b8: bytearray = None,
    ) -> None:
        byteArray = bytearray(8)
        bytes = [b1, b2, b3, b4, b5, b6, b7, b8]

        for i, byte in enumerate(bytes):
            if byte:
                byteArray[i] = byte

        try:
            self.can.writePacket(byteArray, self.API_ID)

        except Exception as e:
            pass

    def disable(self) -> None:
        self.update(b8=bytearray())  # example
