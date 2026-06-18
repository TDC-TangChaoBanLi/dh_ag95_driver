import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from .config import Ag95Config, TransportType, CanBaudRateIndex
from .protocol import (
    Ag95Frame, K_READ, K_WRITE, K_FUNC_INITIALIZATION, K_SUB_INIT_FEEDBACK, K_SUB_INITIALIZE,
    K_FUNC_FORCE, K_SUB_FORCE_INTERNAL, K_SUB_FORCE_EXTERNAL, K_FUNC_POSITION, K_SUB_POSITION,
    K_FUNC_FEEDBACK, K_SUB_STATUS, K_FUNC_IO_MODE, K_FUNC_CAN_ID, K_FUNC_FIRMWARE_VERSION,
    K_FUNC_CAN_BAUD_RATE, K_FUNC_OBJECT_DROPPED, K_SUB_DROP_ENABLE, K_SUB_DROP_FEEDBACK,
)
from .transports import OfficialSerialTransport, SocketCanTransport, SlcanTransport, PcanBasicTransport, ModbusRtuTransport, TimeoutError


class GripperStatus(IntEnum):
    MOVING_OR_DEFAULT = 0
    REACHED_POSITION = 2
    GRASPED_OBJECT = 3
    UNKNOWN = -1


@dataclass
class Ag95State:
    position_percent: int = -1
    force_internal_percent: int = -1
    force_external_percent: int = -1
    raw_status: int = -1
    status: GripperStatus = GripperStatus.UNKNOWN
    initialized: bool = False
    reached: bool = False
    grasped: bool = False
    moving: bool = False


@dataclass
class FirmwareVersion:
    raw: int
    b0: int
    b1: int
    b2: int
    b3: int
    def __str__(self) -> str:
        return f"{self.b3}.{self.b2}.{self.b1}.{self.b0}"


class Ag95Gripper:
    def __init__(self, config: Optional[Ag95Config] = None, transport=None):
        self.config = config or Ag95Config()
        self.transport = transport or self._make_transport()
        self._last_command = 0.0

    def _make_transport(self):
        if self.config.transport_type == TransportType.OFFICIAL_SERIAL:
            return OfficialSerialTransport(self.config.official_serial)
        if self.config.transport_type == TransportType.SOCKETCAN:
            return SocketCanTransport(self.config.socketcan)
        if self.config.transport_type == TransportType.SLCAN:
            return SlcanTransport(self.config.slcan)
        if self.config.transport_type == TransportType.PCANBASIC:
            return PcanBasicTransport(self.config.pcanbasic)
        if self.config.transport_type == TransportType.MODBUS_RTU:
            return ModbusRtuTransport(self.config.modbus_rtu)
        raise ValueError(f"unknown transport {self.config.transport_type}")

    def connect(self) -> None:
        self.transport.open()
        # Drain stale frames and let the bus settle before sending commands.
        time.sleep(0.05)
        try:
            while True:
                self.transport.receive(0.01)
        except TimeoutError:
            pass
        self._last_command = time.monotonic() - self.config.command_interval_ms / 1000.0
        if self.config.auto_initialize:
            self.initialize(wait=True)

    def disconnect(self) -> None:
        self.transport.close()

    def _interval(self):
        dt = self.config.command_interval_ms / 1000.0
        elapsed = time.monotonic() - self._last_command
        if elapsed < dt:
            time.sleep(dt - elapsed)
        self._last_command = time.monotonic()

    def transact(self, frame: Ag95Frame, retries: int = 4) -> Ag95Frame:
        last_err = None
        for attempt in range(retries + 1):
            try:
                self._interval()
                self.transport.send(frame)
                deadline = time.monotonic() + self.config.read_timeout_ms / 1000.0
                # Read frames until we find a matching response or time out.
                # Non-matching frames (unsolicited status/event frames from the
                # gripper) are discarded so the correct response is not missed.
                while time.monotonic() < deadline:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    # Block for the full remaining time — when a frame arrives
                    # we check it immediately and either return (match) or
                    # discard (non-match) and go back to waiting.
                    try:
                        resp = self.transport.receive(remaining)
                    except TimeoutError:
                        break  # deadline reached
                    if (resp.id, resp.function, resp.sub_function, resp.rw) == (frame.id, frame.function, frame.sub_function, frame.rw):
                        return resp
                    # non-matching → discard, re-enter receive with updated remaining
                raise TimeoutError("timeout waiting for matching CAN frame")
            except (TimeoutError, RuntimeError) as e:
                last_err = e
                if attempt < retries:
                    time.sleep(0.1)
        raise last_err  # type: ignore[misc]

    def read_register(self, function: int, sub_function: int, id_override: Optional[int] = None) -> int:
        return self.transact(Ag95Frame(id_override if id_override is not None else self.config.gripper_id, function, sub_function, K_READ, 0)).value

    def _send_only(self, frame: Ag95Frame) -> None:
        """Send a frame without waiting for a response (fire-and-forget).

        The AG95 gripper does not echo write commands, so write_register uses this
        to avoid unnecessary timeouts.
        """
        self._interval()
        self.transport.send(frame)

    def write_register(self, function: int, sub_function: int, value: int, id_override: Optional[int] = None) -> int:
        frame = Ag95Frame(id_override if id_override is not None else self.config.gripper_id, function, sub_function, K_WRITE, value)
        self._send_only(frame)
        return value

    @staticmethod
    def _check_range(name: str, value: int, lo: int, hi: int):
        if value < lo or value > hi:
            raise ValueError(f"{name} must be in [{lo}, {hi}]")

    def set_initialization_feedback_enabled(self, enabled: bool):
        self.write_register(K_FUNC_INITIALIZATION, K_SUB_INIT_FEEDBACK, 1 if enabled else 0)

    def get_initialization_feedback_enabled(self) -> bool:
        return self.read_register(K_FUNC_INITIALIZATION, K_SUB_INIT_FEEDBACK) != 0

    def initialize(self, wait: bool = False, timeout_s: float = 10.0):
        self.write_register(K_FUNC_INITIALIZATION, K_SUB_INITIALIZE, 0)
        if wait:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                try:
                    if self.is_initialized():
                        return
                except TimeoutError:
                    pass  # gripper may be busy calibrating, retry after sleep
                time.sleep(0.05)
            raise TimeoutError("timeout waiting for initialization")

    def is_initialized(self) -> bool:
        return self.read_register(K_FUNC_INITIALIZATION, K_SUB_INITIALIZE) == 1

    def set_force(self, percent: int, sub_function: int = K_SUB_FORCE_INTERNAL):
        self._check_range("force", percent, 20, 100)
        if sub_function not in (K_SUB_FORCE_INTERNAL, K_SUB_FORCE_EXTERNAL):
            raise ValueError("force sub_function must be 0x02 or 0x03")
        self.write_register(K_FUNC_FORCE, sub_function, percent)

    def get_force(self, sub_function: int = K_SUB_FORCE_INTERNAL) -> int:
        if sub_function not in (K_SUB_FORCE_INTERNAL, K_SUB_FORCE_EXTERNAL):
            raise ValueError("force sub_function must be 0x02 or 0x03")
        return int(self.read_register(K_FUNC_FORCE, sub_function))

    def set_position(self, percent: int):
        self._check_range("position", percent, 0, 100)
        self.write_register(K_FUNC_POSITION, K_SUB_POSITION, percent)

    def get_position(self) -> int:
        return int(self.read_register(K_FUNC_POSITION, K_SUB_POSITION))

    def get_status(self) -> GripperStatus:
        raw = self.read_register(K_FUNC_FEEDBACK, K_SUB_STATUS)
        return GripperStatus(raw) if raw in (0, 2, 3) else GripperStatus.UNKNOWN

    def read_state(self) -> Ag95State:
        s = Ag95State()
        for attr, fn in [
            ("position_percent", self.get_position),
            ("force_internal_percent", lambda: self.get_force(K_SUB_FORCE_INTERNAL)),
            ("force_external_percent", lambda: self.get_force(K_SUB_FORCE_EXTERNAL)),
        ]:
            try: setattr(s, attr, fn())
            except Exception: pass
        try: s.initialized = self.is_initialized()
        except Exception: pass
        try:
            s.raw_status = self.read_register(K_FUNC_FEEDBACK, K_SUB_STATUS)
            s.status = GripperStatus(s.raw_status) if s.raw_status in (0,2,3) else GripperStatus.UNKNOWN
        except Exception: pass
        s.reached = s.status == GripperStatus.REACHED_POSITION
        s.grasped = s.status == GripperStatus.GRASPED_OBJECT
        s.moving = s.status == GripperStatus.MOVING_OR_DEFAULT
        return s

    def set_io_mode_enabled(self, enabled: bool):
        self.write_register(K_FUNC_IO_MODE, 0x09, 1 if enabled else 0)

    def get_io_mode_enabled(self) -> bool:
        return self.read_register(K_FUNC_IO_MODE, 0x09) != 0

    def set_io_parameter(self, sub_function: int, value: int):
        if sub_function in (0x01, 0x02, 0x05, 0x06): self._check_range("I/O position", value, 0, 100)
        elif sub_function in (0x03, 0x07, 0x0A, 0x0B): self._check_range("I/O force", value, 20, 100)
        elif sub_function in (0x04, 0x08, 0x09): self._check_range("I/O bool/index", value, 0, 1)
        else: raise ValueError("unsupported I/O sub_function")
        self.write_register(K_FUNC_IO_MODE, sub_function, value)

    def get_io_parameter(self, sub_function: int) -> int:
        if sub_function < 1 or sub_function > 0x0B:
            raise ValueError("unsupported I/O sub_function")
        return int(self.read_register(K_FUNC_IO_MODE, sub_function))

    def set_can_id(self, new_id: int, use_broadcast_id: bool = False):
        self._check_range("CAN ID", new_id, 1, 255)
        self.write_register(K_FUNC_CAN_ID, 0x01, new_id, 0 if use_broadcast_id else None)

    def get_can_id(self, use_broadcast_id: bool = False) -> int:
        return int(self.read_register(K_FUNC_CAN_ID, 0x01, 0 if use_broadcast_id else None))

    def get_firmware_version(self) -> FirmwareVersion:
        raw = self.read_register(K_FUNC_FIRMWARE_VERSION, 0x01) & 0xFFFFFFFF
        return FirmwareVersion(raw, raw & 0xFF, (raw >> 8) & 0xFF, (raw >> 16) & 0xFF, (raw >> 24) & 0xFF)

    def set_can_baudrate(self, index: CanBaudRateIndex | int):
        v = int(index)
        self._check_range("CAN baudrate index", v, 0, 5)
        self.write_register(K_FUNC_CAN_BAUD_RATE, 0x01, v)

    def get_can_baudrate(self) -> CanBaudRateIndex:
        return CanBaudRateIndex(self.read_register(K_FUNC_CAN_BAUD_RATE, 0x01))

    def set_drop_detection_enabled(self, enabled: bool):
        self.write_register(K_FUNC_OBJECT_DROPPED, K_SUB_DROP_ENABLE, 1 if enabled else 0)

    def get_drop_detection_enabled(self) -> bool:
        return self.read_register(K_FUNC_OBJECT_DROPPED, K_SUB_DROP_ENABLE) != 0

    def acknowledge_drop_feedback(self):
        self.write_register(K_FUNC_OBJECT_DROPPED, K_SUB_DROP_FEEDBACK, 0)

    def receive_event(self, timeout_s: float = 0.1):
        try:
            return self.transport.receive(timeout_s)
        except TimeoutError:
            return None
