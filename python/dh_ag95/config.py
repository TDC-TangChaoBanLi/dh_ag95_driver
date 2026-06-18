from dataclasses import dataclass, field
from enum import Enum, IntEnum
from pathlib import Path
from typing import Any, Dict

import yaml


class TransportType(str, Enum):
    OFFICIAL_SERIAL = "official_serial"
    SOCKETCAN = "socketcan"
    SLCAN = "slcan"
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
class SlcanConfig:
    port: str = "/dev/ttyUSB0"
    serial_baudrate: int = 115200
    can_bitrate: int = 500000
    configure_on_open: bool = True


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
    gripper_id: int = 1
    transport_type: TransportType = TransportType.OFFICIAL_SERIAL
    command_interval_ms: int = 50
    read_timeout_ms: int = 500
    auto_initialize: bool = False
    default_force_percent: int = 30
    official_serial: OfficialSerialConfig = field(default_factory=OfficialSerialConfig)
    socketcan: SocketCanConfig = field(default_factory=SocketCanConfig)
    slcan: SlcanConfig = field(default_factory=SlcanConfig)
    pcanbasic: PcanBasicConfig = field(default_factory=PcanBasicConfig)
    modbus_rtu: ModbusRtuConfig = field(default_factory=ModbusRtuConfig)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Ag95Config":
        cfg = cls()
        for key in ["gripper_id", "command_interval_ms", "read_timeout_ms", "auto_initialize", "default_force_percent"]:
            if key in data:
                setattr(cfg, key, data[key])
        if "transport_type" in data:
            cfg.transport_type = TransportType(data["transport_type"])
        for section, cls_ in [
            ("official_serial", OfficialSerialConfig),
            ("socketcan", SocketCanConfig),
            ("slcan", SlcanConfig),
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
