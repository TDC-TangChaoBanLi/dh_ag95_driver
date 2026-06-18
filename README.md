# dh_ag95_core_driver

Standalone DH-Robotics AG-95 driver library. It does **not** depend on ROS.

It contains:

- C++17 driver library (`dh_ag95_core`)
- Python driver package (`dh_ag95`)
- Transport backends:
  - `official_serial`: DH official protocol converter USB virtual serial port, 14-byte frame `FF FE FD FC + ID + 8-byte CAN payload + FB`
  - `socketcan`: Linux SocketCAN, standard CAN 2.0A frame, 11-bit CAN ID = gripper ID, DLC=8
  - `slcan`: SLCAN ASCII serial CAN adapter, standard CAN 2.0A frame
  - `pcanbasic`: PEAK PCAN-Basic backend for PEAK PCAN-USB devices. C++ is implemented for Windows through runtime loading of `PCANBasic.dll`; Python uses `python-can`'s `pcan` interface.
  - `modbus_rtu`: reserved placeholder; not implemented in this version

Platform support:

- Linux: `official_serial`, `socketcan`, `slcan`; Python can also use `pcanbasic` if python-can and PEAK runtime are available.
- Windows: `official_serial`, `slcan`, and `pcanbasic`; `socketcan` is Linux-only and throws a clear error on Windows.

Implemented AG-95 CAN protocol functions:

- initialization feedback switch `0x08/0x01`
- initialization command / state `0x08/0x02`
- force `0x05/0x02` and `0x05/0x03`
- position `0x06/0x02`
- current status `0x0F/0x01`
- I/O mode `0x10/0x01..0x0B`
- CAN ID `0x12/0x01`
- firmware version `0x13/0x01`
- CAN baud rate `0x14/0x01`
- object dropped feedback `0x15/0x01` and `0x15/0x02`

## Build C++ on Linux

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --install .
```

## Build C++ on Windows

Use Visual Studio Developer PowerShell or CMake GUI:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

For `pcanbasic`, install the PEAK PCAN driver/PCAN-Basic runtime so `PCANBasic.dll` can be found by Windows.

## C++ examples

Official DH USB converter:

```cpp
#include <dh_ag95/ag95_gripper.hpp>

int main() {
  dh_ag95::Ag95Config cfg;
  cfg.transport_type = dh_ag95::TransportType::OfficialSerial;
  cfg.gripper_id = 1;
  cfg.official_serial.port = "COM5";       // Windows, or /dev/ttyACM0 on Linux
  cfg.official_serial.baudrate = 115200;

  dh_ag95::Ag95Gripper gripper(cfg);
  gripper.connect();
  gripper.initialize(true);
  gripper.set_force(30);
  gripper.set_position(60);
  auto state = gripper.read_state();
  gripper.disconnect();
}
```

PEAK PCAN-USB on Windows:

```cpp
#include <dh_ag95/ag95_gripper.hpp>

int main() {
  dh_ag95::Ag95Config cfg;
  cfg.transport_type = dh_ag95::TransportType::PcanBasic;
  cfg.gripper_id = 1;
  cfg.pcanbasic.channel = "PCAN_USBBUS1";
  cfg.pcanbasic.bitrate = 500000;

  dh_ag95::Ag95Gripper gripper(cfg);
  gripper.connect();
  gripper.initialize(true);
  gripper.set_position(60);
  gripper.disconnect();
}
```

## Python example

```bash
pip install -e python
pip install -e "python[can]"       # needed for socketcan/slcan/pcanbasic via python-can
```

```python
from dh_ag95 import Ag95Config, TransportType, Ag95Gripper

cfg = Ag95Config(transport_type=TransportType.OFFICIAL_SERIAL)
cfg.official_serial.port = "COM5"       # Windows, or /dev/ttyACM0 on Linux

g = Ag95Gripper(cfg)
g.connect()
g.initialize(wait=True)
g.set_force(30)
g.set_position(60)
print(g.read_state())
g.disconnect()
```

PEAK PCAN-USB via Python:

```python
from dh_ag95 import Ag95Config, TransportType, Ag95Gripper

cfg = Ag95Config(transport_type=TransportType.PCANBASIC)
cfg.pcanbasic.channel = "PCAN_USBBUS1"
cfg.pcanbasic.bitrate = 500000

g = Ag95Gripper(cfg)
g.connect()
g.initialize(wait=True)
g.set_position(60)
g.disconnect()
```

## Notes

- Command interval defaults to 30 ms because the official manual recommends an interval greater than 20 ms.
- Force is limited to 20..100.
- Position is limited to 0..100.
- Changing CAN ID or CAN baud rate requires rebooting the gripper.
- If your USB-to-CAN appears as **PCAN**, use `pcanbasic` on Windows. On Linux, prefer `socketcan` if the PEAK driver exposes it as `can0`.
- For raw SocketCAN mode, configure your CAN interface before running, e.g.:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```
