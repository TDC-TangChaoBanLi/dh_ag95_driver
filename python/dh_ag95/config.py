from dataclasses import dataclass, field
from enum import Enum, IntEnum
from pathlib import Path
from typing import Any, Dict

import yaml


class TransportType(str, Enum):
    OFFICIAL_SERIAL = "official_serial"
    SOCKETCAN = "socketcan"
    PCANBASIC = "pcanbasic"
    MODBUS_RTU = "modbus_rtu"


class CanBaudRateIndex(IntEnum):
    KBPS_500 = 0
    KBPS_400 = 1
    KBPS_250 = 2
    KBPS_200 = 3
    KBPS_125 = 4
    KBPS_100 = 5


@dataclass
class OfficialSerialConfig:
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200


@dataclass
class SocketCanConfig:
    interface_name: str = "can0"
    bitrate: int = 500000


@dataclass
class PcanBasicConfig:
    channel: str = "PCAN_USBBUS1"
    bitrate: int = 500000


@dataclass
class ModbusRtuConfig:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    parity: str = "N"
    data_bits: int = 8
    stop_bits: int = 1


@dataclass
class Ag95Config:
    """Master configuration for the AG-95 gripper driver.

    Attributes:
        gripper_id: CAN ID of the gripper, range 1–255, default 0x01.
        transport_type: Transport layer to use.
        command_interval_ms: Minimum interval between consecutive commands (≥20 ms).
        read_timeout_ms: Per-transaction read timeout.
        max_retries: Maximum retry attempts per transact (0 = no retries).
        wait_write_echo: If True, write_register waits for device echo via transact.
        coalesce_writes: If True, consecutive writes to the same register are coalesced.
        coalesce_window_ms: Coalescing time window in ms (0 = disabled).
        skip_duplicate_writes: If True, writes equal to the last-sent value are skipped.
        recv_thread_timeout_ms: Poll timeout for the background receive thread.
        max_event_queue_size: Maximum unsolicited frames kept in the event queue.
        auto_initialize: If True, connect() calls initialize(wait=True).
        default_force_percent: Default gripping force percentage.
    """
    gripper_id: int = 1
    transport_type: TransportType = TransportType.OFFICIAL_SERIAL
    command_interval_ms: int = 25
    read_timeout_ms: int = 500
    max_retries: int = 2
    wait_write_echo: bool = True
    coalesce_writes: bool = False
    coalesce_window_ms: int = 0
    skip_duplicate_writes: bool = True
    recv_thread_timeout_ms: int = 100
    max_event_queue_size: int = 64
    auto_initialize: bool = False
    default_force_percent: int = 30
    official_serial: OfficialSerialConfig = field(default_factory=OfficialSerialConfig)
    socketcan: SocketCanConfig = field(default_factory=SocketCanConfig)
    pcanbasic: PcanBasicConfig = field(default_factory=PcanBasicConfig)
    modbus_rtu: ModbusRtuConfig = field(default_factory=ModbusRtuConfig)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Ag95Config":
        cfg = cls()
        for key in [
            "gripper_id", "command_interval_ms", "read_timeout_ms",
            "max_retries", "wait_write_echo", "coalesce_writes",
            "coalesce_window_ms", "skip_duplicate_writes",
            "recv_thread_timeout_ms", "max_event_queue_size",
            "auto_initialize", "default_force_percent",
        ]:
            if key in data:
                setattr(cfg, key, data[key])
        if "transport_type" in data:
            cfg.transport_type = TransportType(data["transport_type"])
        for section, cls_ in [
            ("official_serial", OfficialSerialConfig),
            ("socketcan", SocketCanConfig),
            ("pcanbasic", PcanBasicConfig),
            ("modbus_rtu", ModbusRtuConfig),
        ]:
            if section in data and data[section] is not None:
                setattr(cfg, section, cls_(**data[section]))
        return cfg

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Ag95Config":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(yaml.safe_load(f) or {})
