"""High-level AG-95 gripper controller with background receive thread and
optional write-command coalescing."""

import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from .config import Ag95Config, TransportType, CanBaudRateIndex
from .protocol import (
    Ag95Frame, K_READ, K_WRITE,
    K_FUNC_INITIALIZATION, K_SUB_INIT_FEEDBACK, K_SUB_INITIALIZE,
    K_FUNC_FORCE, K_SUB_FORCE_INTERNAL, K_SUB_FORCE_EXTERNAL,
    K_FUNC_POSITION, K_SUB_POSITION,
    K_FUNC_FEEDBACK, K_SUB_STATUS,
    K_FUNC_IO_MODE, K_FUNC_CAN_ID, K_FUNC_FIRMWARE_VERSION,
    K_FUNC_CAN_BAUD_RATE, K_FUNC_OBJECT_DROPPED,
    K_SUB_DROP_ENABLE, K_SUB_DROP_FEEDBACK,
)
from .transports import (
    OfficialSerialTransport, SocketCanTransport,
    PcanBasicTransport, ModbusRtuTransport, TimeoutError,
)


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
    """High-level AG-95 gripper controller.

    Manages communication lifecycle (connect / disconnect), command
    transactions with automatic retry, and a background receive thread that
    collects unsolicited frames (e.g. drop-detection events) into an internal
    event queue.
    """

    def __init__(self, config: Optional[Ag95Config] = None, transport=None):
        self.config = config or Ag95Config()
        self.transport = transport or self._make_transport()
        self._last_command = 0.0

        # Background receive thread
        self._recv_thread: Optional[threading.Thread] = None
        self._recv_running = threading.Event()

        # Event queue for unsolicited frames
        self._event_queue: deque[Ag95Frame] = deque()
        self._event_lock = threading.Lock()
        self._event_cv = threading.Condition(self._event_lock)

        # Pending transaction (transact ↔ receive thread)
        self._pending_frame: Optional[Ag95Frame] = None
        self._pending_response: Optional[Ag95Frame] = None
        self._pending_done = threading.Event()
        self._transact_lock = threading.Lock()

        # Write coalescing cache: key = (function << 8) | sub_function
        self._write_cache: dict[int, dict] = {}

    # ------------------------------------------------------------------
    # Transport factory
    # ------------------------------------------------------------------

    def _make_transport(self):
        if self.config.transport_type == TransportType.OFFICIAL_SERIAL:
            return OfficialSerialTransport(self.config.official_serial)
        if self.config.transport_type == TransportType.SOCKETCAN:
            return SocketCanTransport(self.config.socketcan)
        if self.config.transport_type == TransportType.PCANBASIC:
            return PcanBasicTransport(self.config.pcanbasic)
        if self.config.transport_type == TransportType.MODBUS_RTU:
            return ModbusRtuTransport(self.config.modbus_rtu)
        raise ValueError(f"unknown transport {self.config.transport_type}")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the transport, drain stale frames, start background receive
        thread, and optionally auto-initialize."""
        self.transport.open()
        time.sleep(0.05)
        # Drain stale frames
        try:
            while True:
                self.transport.receive(0.01)
        except TimeoutError:
            pass

        # Start receive thread
        self._recv_running.set()
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

        self._last_command = time.monotonic() - self.config.command_interval_ms / 1000.0
        if self.config.auto_initialize:
            self.initialize(wait=True)

    def disconnect(self) -> None:
        """Flush pending writes, stop the receive thread, and close the transport."""
        self.flush()
        self._recv_running.clear()
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=2.0)
            self._recv_thread = None
        # Abort any pending transaction
        with self._transact_lock:
            self._pending_frame = None
        self.transport.close()

    # ------------------------------------------------------------------
    # Receive thread
    # ------------------------------------------------------------------

    def _receive_loop(self) -> None:
        """Background thread: continuously read frames from the transport.
        Matching frames are delivered to transact(); unsolicited frames go to
        the event queue."""
        while self._recv_running.is_set():
            try:
                frame = self.transport.receive(
                    self.config.recv_thread_timeout_ms / 1000.0)
            except TimeoutError:
                continue
            except Exception:
                if self._recv_running.is_set():
                    time.sleep(0.01)
                continue

            # Check for pending transaction match
            delivered = False
            with self._transact_lock:
                if (self._pending_frame is not None
                        and self._frame_matches(self._pending_frame, frame)):
                    self._pending_response = frame
                    self._pending_done.set()
                    delivered = True
            if delivered:
                continue

            # Unsolicited → event queue
            with self._event_lock:
                if len(self._event_queue) < self.config.max_event_queue_size:
                    self._event_queue.append(frame)
                self._event_cv.notify_all()

    @staticmethod
    def _frame_matches(expected: Ag95Frame, received: Ag95Frame) -> bool:
        return (received.id == expected.id
                and received.function == expected.function
                and received.sub_function == expected.sub_function
                and received.rw == expected.rw)

    # ------------------------------------------------------------------
    # Timing
    # ------------------------------------------------------------------

    def _interval(self) -> None:
        """Enforce the minimum command interval."""
        dt = self.config.command_interval_ms / 1000.0
        elapsed = time.monotonic() - self._last_command
        if elapsed < dt:
            time.sleep(dt - elapsed)
        self._last_command = time.monotonic()

    # ------------------------------------------------------------------
    # Core transaction
    # ------------------------------------------------------------------

    def transact(self, frame: Ag95Frame) -> Ag95Frame:
        """Send a frame and wait for the matching response via the receive
        thread.  Applies command interval and retries."""
        last_err = None
        for attempt in range(self.config.max_retries + 1):
            try:
                self._interval()
                self._pending_done.clear()
                with self._transact_lock:
                    self._pending_frame = frame
                    self._pending_response = None
                self.transport.send(frame)

                if self._pending_done.wait(
                        timeout=self.config.read_timeout_ms / 1000.0):
                    with self._transact_lock:
                        resp = self._pending_response
                        self._pending_frame = None
                    if resp is not None:
                        return resp

                with self._transact_lock:
                    self._pending_frame = None
                raise TimeoutError("timeout waiting for matching CAN frame")

            except (TimeoutError, RuntimeError) as e:
                last_err = e
                if attempt < self.config.max_retries:
                    time.sleep(0.05)
        raise last_err  # type: ignore[misc]

    # ------------------------------------------------------------------
    # Low-level register access
    # ------------------------------------------------------------------

    def read_register(self, function: int, sub_function: int,
                      id_override: Optional[int] = None) -> int:
        """Read a register.  Flushes pending coalesced writes first."""
        self._flush_pending_writes()
        fid = id_override if id_override is not None else self.config.gripper_id
        return self.transact(
            Ag95Frame(fid, function, sub_function, K_READ, 0)).value

    def _send_write_immediate(self, function: int, sub_function: int,
                              value: int, id_override: Optional[int]) -> int:
        """Send a write command immediately (no coalescing)."""
        fid = id_override if id_override is not None else self.config.gripper_id
        frame = Ag95Frame(fid, function, sub_function, K_WRITE, value)
        if self.config.wait_write_echo:
            return self.transact(frame).value
        # Fire-and-forget: the receive thread discards the echo
        self._interval()
        self.transport.send(frame)
        return value

    def write_register(self, function: int, sub_function: int,
                       value: int, id_override: Optional[int] = None) -> int:
        """Write a register.  Behaviour depends on config:
        - coalesce_writes enabled → value may be deferred (see flush())
        - wait_write_echo = True → waits for device echo
        - wait_write_echo = False → fire-and-forget
        """
        fid = id_override if id_override is not None else self.config.gripper_id

        if not self.config.coalesce_writes or self.config.coalesce_window_ms <= 0:
            return self._send_write_immediate(function, sub_function, value,
                                              id_override)

        key = (function << 8) | sub_function
        cache = self._write_cache.setdefault(key, {
            "id": fid, "function": function, "sub_function": sub_function,
            "last_sent_value": None, "last_send_time": 0.0,
            "pending": False, "pending_value": 0, "pending_since": 0.0,
        })
        now = time.monotonic()

        # Skip duplicate
        if (self.config.skip_duplicate_writes and not cache["pending"]
                and cache["last_sent_value"] == value):
            return value

        # Already have a pending write
        if cache["pending"]:
            if cache["pending_value"] == value:
                cache["pending_since"] = now
                return value
            if (now - cache["pending_since"]
                    < self.config.coalesce_window_ms / 1000.0):
                cache["pending_value"] = value
                cache["pending_since"] = now
                return value
            self._flush_pending_writes()

        # Recently sent same value
        if (cache["last_sent_value"] == value
                and now - cache["last_send_time"]
                < self.config.coalesce_window_ms / 1000.0):
            return value

        # Defer
        cache["pending"] = True
        cache["pending_value"] = value
        cache["pending_since"] = now
        return value

    def _flush_pending_writes(self) -> None:
        """Send all deferred coalesced writes."""
        keys = [k for k, v in self._write_cache.items() if v["pending"]]
        for key in keys:
            cache = self._write_cache.get(key)
            if cache is None or not cache["pending"]:
                continue
            self._send_write_immediate(
                cache["function"], cache["sub_function"],
                cache["pending_value"], cache["id"])
            cache["last_sent_value"] = cache["pending_value"]
            cache["last_send_time"] = time.monotonic()
            cache["pending"] = False

    def flush(self) -> None:
        """Flush all pending coalesced writes immediately."""
        self._flush_pending_writes()

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    @staticmethod
    def _check_range(name: str, value: int, lo: int, hi: int) -> None:
        if value < lo or value > hi:
            raise ValueError(f"{name} must be in [{lo}, {hi}]")

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def set_initialization_feedback_enabled(self, enabled: bool) -> None:
        self._send_write_immediate(K_FUNC_INITIALIZATION, K_SUB_INIT_FEEDBACK,
                                   1 if enabled else 0, None)

    def get_initialization_feedback_enabled(self) -> bool:
        return self.read_register(K_FUNC_INITIALIZATION,
                                  K_SUB_INIT_FEEDBACK) != 0

    def initialize(self, wait: bool = False, timeout_s: float = 10.0) -> None:
        """Send the initialization command.  Disables auto-feedback first,
        then polls until the gripper reports initialization complete."""
        # Disable auto-feedback; we poll instead.
        self._send_write_immediate(K_FUNC_INITIALIZATION,
                                   K_SUB_INIT_FEEDBACK, 0, None)
        # Send init command.
        self._send_write_immediate(K_FUNC_INITIALIZATION,
                                   K_SUB_INITIALIZE, 0, None)
        if not wait:
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                if self.is_initialized():
                    return
            except TimeoutError:
                pass
            time.sleep(0.05)
        raise TimeoutError("timeout waiting for initialization")

    def is_initialized(self) -> bool:
        return self.read_register(K_FUNC_INITIALIZATION,
                                  K_SUB_INITIALIZE) == 1

    # ------------------------------------------------------------------
    # Force & Position
    # ------------------------------------------------------------------

    def set_force(self, percent: int,
                  sub_function: int = K_SUB_FORCE_INTERNAL) -> None:
        self._check_range("force", percent, 20, 100)
        if sub_function not in (K_SUB_FORCE_INTERNAL, K_SUB_FORCE_EXTERNAL):
            raise ValueError("force sub_function must be 0x02 or 0x03")
        self.write_register(K_FUNC_FORCE, sub_function, percent)

    def get_force(self, sub_function: int = K_SUB_FORCE_INTERNAL) -> int:
        if sub_function not in (K_SUB_FORCE_INTERNAL, K_SUB_FORCE_EXTERNAL):
            raise ValueError("force sub_function must be 0x02 or 0x03")
        return int(self.read_register(K_FUNC_FORCE, sub_function))

    def set_position(self, percent: int) -> None:
        self._check_range("position", percent, 0, 100)
        self.write_register(K_FUNC_POSITION, K_SUB_POSITION, percent)

    def get_position(self) -> int:
        return int(self.read_register(K_FUNC_POSITION, K_SUB_POSITION))

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def get_status(self) -> GripperStatus:
        raw = self.read_register(K_FUNC_FEEDBACK, K_SUB_STATUS)
        return GripperStatus(raw) if raw in (0, 2, 3) else GripperStatus.UNKNOWN

    def get_all_state(self) -> Ag95State:
        """Read position, internal/external force, init state, and status
        in a single convenience call (5 independent CAN transactions)."""
        s = Ag95State()
        for attr, fn in [
            ("position_percent", self.get_position),
            ("force_internal_percent",
             lambda: self.get_force(K_SUB_FORCE_INTERNAL)),
            ("force_external_percent",
             lambda: self.get_force(K_SUB_FORCE_EXTERNAL)),
        ]:
            try:
                setattr(s, attr, fn())
            except Exception:
                pass
        try:
            s.initialized = self.is_initialized()
        except Exception:
            pass
        try:
            s.raw_status = self.read_register(K_FUNC_FEEDBACK, K_SUB_STATUS)
            s.status = (GripperStatus(s.raw_status)
                        if s.raw_status in (0, 2, 3)
                        else GripperStatus.UNKNOWN)
        except Exception:
            pass
        s.reached = s.status == GripperStatus.REACHED_POSITION
        s.grasped = s.status == GripperStatus.GRASPED_OBJECT
        s.moving = s.status == GripperStatus.MOVING_OR_DEFAULT
        return s

    # ------------------------------------------------------------------
    # I/O Mode
    # ------------------------------------------------------------------

    def set_io_mode_enabled(self, enabled: bool) -> None:
        self.write_register(K_FUNC_IO_MODE, 0x09, 1 if enabled else 0)

    def get_io_mode_enabled(self) -> bool:
        return self.read_register(K_FUNC_IO_MODE, 0x09) != 0

    def set_io_parameter(self, sub_function: int, value: int) -> None:
        if sub_function in (0x01, 0x02, 0x05, 0x06):
            self._check_range("I/O position", value, 0, 100)
        elif sub_function in (0x03, 0x07, 0x0A, 0x0B):
            self._check_range("I/O force", value, 20, 100)
        elif sub_function in (0x04, 0x08, 0x09):
            self._check_range("I/O bool/index", value, 0, 1)
        else:
            raise ValueError("unsupported I/O sub_function")
        self.write_register(K_FUNC_IO_MODE, sub_function, value)

    def get_io_parameter(self, sub_function: int) -> int:
        if sub_function < 1 or sub_function > 0x0B:
            raise ValueError("unsupported I/O sub_function")
        return int(self.read_register(K_FUNC_IO_MODE, sub_function))

    # ------------------------------------------------------------------
    # CAN Settings
    # ------------------------------------------------------------------

    def set_can_id(self, new_id: int, use_broadcast_id: bool = False) -> None:
        self._check_range("CAN ID", new_id, 1, 255)
        self.write_register(K_FUNC_CAN_ID, 0x01, new_id,
                            0 if use_broadcast_id else None)

    def get_can_id(self, use_broadcast_id: bool = False) -> int:
        return int(self.read_register(K_FUNC_CAN_ID, 0x01,
                                      0 if use_broadcast_id else None))

    def get_firmware_version(self) -> FirmwareVersion:
        raw = self.read_register(K_FUNC_FIRMWARE_VERSION, 0x01) & 0xFFFFFFFF
        return FirmwareVersion(raw, raw & 0xFF, (raw >> 8) & 0xFF,
                               (raw >> 16) & 0xFF, (raw >> 24) & 0xFF)

    def set_can_baudrate(self, index: CanBaudRateIndex | int) -> None:
        v = int(index)
        self._check_range("CAN baudrate index", v, 0, 5)
        self.write_register(K_FUNC_CAN_BAUD_RATE, 0x01, v)

    def get_can_baudrate(self) -> CanBaudRateIndex:
        return CanBaudRateIndex(
            self.read_register(K_FUNC_CAN_BAUD_RATE, 0x01))

    # ------------------------------------------------------------------
    # Drop Detection
    # ------------------------------------------------------------------

    def set_drop_detection_enabled(self, enabled: bool) -> None:
        self.write_register(K_FUNC_OBJECT_DROPPED, K_SUB_DROP_ENABLE,
                            1 if enabled else 0)

    def get_drop_detection_enabled(self) -> bool:
        return self.read_register(K_FUNC_OBJECT_DROPPED,
                                  K_SUB_DROP_ENABLE) != 0

    def acknowledge_drop_feedback(self) -> None:
        self.write_register(K_FUNC_OBJECT_DROPPED, K_SUB_DROP_FEEDBACK, 0)

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def receive_event(self, timeout_s: float = 0.1) -> Optional[Ag95Frame]:
        """Retrieve the next unsolicited frame from the event queue.
        Returns None if no event is available within the timeout."""
        with self._event_lock:
            if self._event_queue:
                return self._event_queue.popleft()
            if timeout_s <= 0:
                return None
            if self._event_cv.wait(timeout=timeout_s):
                if self._event_queue:
                    return self._event_queue.popleft()
        return None
