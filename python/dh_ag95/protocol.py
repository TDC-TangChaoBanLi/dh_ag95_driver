from dataclasses import dataclass
from typing import Iterable

K_READ = 0x00
K_WRITE = 0x01

K_FUNC_FORCE = 0x05
K_FUNC_POSITION = 0x06
K_FUNC_INITIALIZATION = 0x08
K_FUNC_FEEDBACK = 0x0F
K_FUNC_IO_MODE = 0x10
K_FUNC_CAN_ID = 0x12
K_FUNC_FIRMWARE_VERSION = 0x13
K_FUNC_CAN_BAUD_RATE = 0x14
K_FUNC_OBJECT_DROPPED = 0x15

K_SUB_INIT_FEEDBACK = 0x01
K_SUB_INITIALIZE = 0x02
K_SUB_FORCE_INTERNAL = 0x02
K_SUB_FORCE_EXTERNAL = 0x03
K_SUB_POSITION = 0x02
K_SUB_STATUS = 0x01
K_SUB_DROP_ENABLE = 0x01
K_SUB_DROP_FEEDBACK = 0x02


@dataclass
class Ag95Frame:
    id: int
    function: int
    sub_function: int
    rw: int
    value: int = 0
    reserve: int = 0


class ProtocolError(RuntimeError):
    pass


class Ag95Protocol:
    @staticmethod
    def value_to_bytes(value: int) -> bytes:
        return int(value).to_bytes(4, byteorder="little", signed=True)

    @staticmethod
    def bytes_to_value(data: Iterable[int]) -> int:
        return int.from_bytes(bytes(data), byteorder="little", signed=True)

    @staticmethod
    def to_can_payload(frame: Ag95Frame) -> bytes:
        return bytes([frame.function, frame.sub_function, frame.rw, frame.reserve]) + Ag95Protocol.value_to_bytes(frame.value)

    @staticmethod
    def from_can_payload(can_id: int, data: bytes) -> Ag95Frame:
        if len(data) != 8:
            raise ProtocolError("CAN payload must be 8 bytes")
        return Ag95Frame(can_id & 0xFF, data[0], data[1], data[2], Ag95Protocol.bytes_to_value(data[4:8]), data[3])

    @staticmethod
    def to_official_serial(frame: Ag95Frame) -> bytes:
        return bytes([0xFF, 0xFE, 0xFD, 0xFC, frame.id]) + Ag95Protocol.to_can_payload(frame) + bytes([0xFB])

    @staticmethod
    def from_official_serial(data: bytes) -> Ag95Frame:
        if len(data) != 14 or data[:4] != b"\xff\xfe\xfd\xfc" or data[-1] != 0xFB:
            raise ProtocolError("invalid official serial frame")
        return Ag95Protocol.from_can_payload(data[4], data[5:13])
