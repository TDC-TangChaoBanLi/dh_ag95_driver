import time
from abc import ABC, abstractmethod
from typing import Optional

from .config import OfficialSerialConfig, SocketCanConfig, PcanBasicConfig, ModbusRtuConfig
from .protocol import Ag95Frame, Ag95Protocol, ProtocolError


class TransportError(RuntimeError):
    pass


class TimeoutError(TransportError):
    pass


class Transport(ABC):
    @abstractmethod
    def open(self) -> None: ...
    @abstractmethod
    def close(self) -> None: ...
    @abstractmethod
    def send(self, frame: Ag95Frame) -> None: ...
    @abstractmethod
    def receive(self, timeout_s: float) -> Ag95Frame: ...


class OfficialSerialTransport(Transport):
    def __init__(self, config: OfficialSerialConfig):
        self.config = config
        self.ser = None

    def open(self) -> None:
        import serial
        self.ser = serial.Serial(self.config.port, self.config.baudrate, bytesize=8, parity="N", stopbits=1, timeout=0)

    def close(self) -> None:
        if self.ser:
            self.ser.close()
            self.ser = None

    def send(self, frame: Ag95Frame) -> None:
        if not self.ser:
            raise TransportError("serial port not open")
        self.ser.write(Ag95Protocol.to_official_serial(frame))
        self.ser.flush()

    def receive(self, timeout_s: float) -> Ag95Frame:
        if not self.ser:
            raise TransportError("serial port not open")
        deadline = time.monotonic() + timeout_s
        header = b"\xff\xfe\xfd\xfc"
        buf = bytearray()
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                time.sleep(0.001)
                continue
            if len(buf) < 4:
                if b[0] == header[len(buf)]:
                    buf.append(b[0])
                else:
                    buf = bytearray(b) if b == header[:1] else bytearray()
            else:
                buf.append(b[0])
                if len(buf) == 14:
                    return Ag95Protocol.from_official_serial(bytes(buf))
        raise TimeoutError("timeout waiting for official serial frame")


class SocketCanTransport(Transport):
    def __init__(self, config: SocketCanConfig):
        self.config = config
        self.bus = None

    def open(self) -> None:
        import can
        self.bus = can.interface.Bus(channel=self.config.interface_name, interface="socketcan")

    def close(self) -> None:
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def send(self, frame: Ag95Frame) -> None:
        if not self.bus:
            raise TransportError("CAN bus not open")
        import can
        msg = can.Message(arbitration_id=frame.id, is_extended_id=False, data=Ag95Protocol.to_can_payload(frame))
        self.bus.send(msg)

    def receive(self, timeout_s: float) -> Ag95Frame:
        if not self.bus:
            raise TransportError("CAN bus not open")
        msg = self.bus.recv(timeout_s)
        if msg is None:
            raise TimeoutError("timeout waiting for CAN frame")
        if len(msg.data) != 8:
            raise ProtocolError("received CAN frame with DLC != 8")
        return Ag95Protocol.from_can_payload(msg.arbitration_id, bytes(msg.data))


class PcanBasicTransport(Transport):
    """PEAK PCAN-Basic transport via python-can.

    This is the recommended Python backend when a PEAK PCAN-USB adapter appears as
    a PCAN device instead of a serial port. On Windows, install the PEAK driver and
    PCAN-Basic runtime. On Linux, this can also work with python-can's pcan backend,
    but SocketCAN is usually simpler if the adapter appears as can0.
    """
    def __init__(self, config: PcanBasicConfig):
        self.config = config
        self.bus = None

    def open(self) -> None:
        import can
        try:
            self.bus = can.Bus(interface="pcan", channel=self.config.channel, bitrate=self.config.bitrate)
        except TypeError:
            self.bus = can.interface.Bus(interface="pcan", channel=self.config.channel, bitrate=self.config.bitrate)

    def close(self) -> None:
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def send(self, frame: Ag95Frame) -> None:
        if not self.bus:
            raise TransportError("PCAN bus not open")
        import can
        msg = can.Message(arbitration_id=frame.id & 0x7FF, is_extended_id=False, data=Ag95Protocol.to_can_payload(frame))
        self.bus.send(msg)

    def receive(self, timeout_s: float) -> Ag95Frame:
        if not self.bus:
            raise TransportError("PCAN bus not open")
        msg = self.bus.recv(timeout_s)
        if msg is None:
            raise TimeoutError("timeout waiting for PCAN CAN frame")
        if len(msg.data) != 8:
            raise ProtocolError("received PCAN frame with DLC != 8")
        return Ag95Protocol.from_can_payload(msg.arbitration_id, bytes(msg.data))


class ModbusRtuTransport(Transport):
    def __init__(self, config: ModbusRtuConfig):
        self.config = config
    def open(self) -> None:
        raise NotImplementedError("Modbus RTU transport is reserved but not implemented")
    def close(self) -> None: pass
    def send(self, frame: Ag95Frame) -> None:
        raise NotImplementedError("Modbus RTU transport is reserved but not implemented")
    def receive(self, timeout_s: float) -> Ag95Frame:
        raise NotImplementedError("Modbus RTU transport is reserved but not implemented")
