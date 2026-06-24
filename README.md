# dh_ag95_core_driver

Standalone DH-Robotics AG-95 gripper driver library.  **No ROS dependency.**

---

## Features

- C++17 driver library (`dh_ag95_core`) and Python package (`dh_ag95`)
- Three transport backends:
  | Transport | Protocol | Platform |
  |---|---|---|
  | `official_serial` | 14-byte framed (`FF FE FD FC … FB`) over serial | Linux / Windows |
  | `socketcan` | Raw CAN 2.0A frames (8-byte payload, CAN ID = gripper ID) | Linux only |
  | `pcanbasic` | Raw CAN 2.0A frames via PEAK PCAN-Basic API | Windows (C++); cross-platform (Python via python-can) |
  | `modbus_rtu` | Reserved — not yet implemented | — |

- **Background receive thread** — unsolicited frames (drop-detection, etc.) are collected into an internal event queue and never lost.
- **Write-command coalescing** — optional merging of rapid consecutive writes to the same register, reducing bus traffic.
- **Configurable write-echo waiting** — choose between confirmed writes (default) or fire-and-forget for maximum throughput.
- **Doxygen-documented public API** for C++; Google-style docstrings for Python.

### Implemented CAN Protocol Functions

| Function | Register | R/W |
|---|---|---|
| Initialization feedback enable/disable | `0x08 / 0x01` | R/W |
| Initialization command / status poll | `0x08 / 0x02` | R/W |
| Gripping force (internal / external) | `0x05 / 0x02`, `0x05 / 0x03` | R/W |
| Jaw position | `0x06 / 0x02` | R/W |
| Current status | `0x0F / 0x01` | R |
| I/O mode parameters | `0x10 / 0x01 … 0x0B` | R/W |
| CAN ID | `0x12 / 0x01` | R/W |
| Firmware version | `0x13 / 0x01` | R |
| CAN baud rate | `0x14 / 0x01` | R/W |
| Object-dropped detection | `0x15 / 0x01`, `0x15 / 0x02` | R/W |

---

## Architecture

```
 ┌──────────────────────────────────────────────┐
 │                  Your App                     │
 │  set_position()  get_position()  flush()  …  │
 └──────────────────┬───────────────────────────┘
                    │
 ┌──────────────────▼───────────────────────────┐
 │              Ag95Gripper                      │
 │  ┌─────────────┐  ┌────────────┐             │
 │  │  transact() │  │ write      │             │
 │  │  (send+wait)│  │ coalescing │             │
 │  └──────┬──────┘  └────────────┘             │
 │         │                                     │
 │  ┌──────▼──────┐  ┌──────────────────────┐   │
 │  │  receive    │  │    event_queue        │   │
 │  │  thread     │──▶  (unsolicited frames) │   │
 │  └──────┬──────┘  └──────────┬───────────┘   │
 └─────────┼────────────────────┼───────────────┘
           │                    │ receive_event()
 ┌─────────▼────────────────────▼───────────────┐
 │              ITransport                       │
 │  OfficialSerial  SocketCan  PcanBasic  …     │
 └──────────────────────────────────────────────┘
```

**How it works:**

1. `connect()` starts a background receive thread that continuously reads from the transport.
2. `transact()` sends a command, then waits on a condition variable.  The receive thread matches the response and wakes `transact()`.
3. Frames that do **not** match any pending transaction (e.g. drop-detection `0x15 0x02`) go into an internal event queue.  Call `receive_event()` to retrieve them.
4. `write_register()` (when `wait_write_echo = true`) uses `transact()` internally, so every write is confirmed by the device echo.
5. When `coalesce_writes` is enabled, rapid writes to the same register are deferred and sent only when the coalescing window expires or a read forces a flush.

---

## Build C++

### Linux

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --install .        # optional
```

### Windows

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

For `pcanbasic`, install the [PEAK PCAN driver / PCAN-Basic runtime](https://www.peak-system.com/).

---

## C++ Quick Start

```cpp
#include <dh_ag95/ag95_gripper.hpp>

int main() {
  dh_ag95::Ag95Config cfg;
  cfg.transport_type = dh_ag95::TransportType::OfficialSerial;
  cfg.gripper_id     = 1;
  cfg.official_serial.port = "/dev/ttyACM0";

  dh_ag95::Ag95Gripper gripper(cfg);
  gripper.connect();
  gripper.initialize(true);               // block until init done

  gripper.set_force(30);
  gripper.set_position(60);

  auto state = gripper.get_all_state();
  // state.position_percent, state.force_internal_percent, state.status …

  // Check for unsolicited events (e.g. drop detection)
  if (auto ev = gripper.receive_event(std::chrono::milliseconds(0))) {
    // handle event frame
  }

  gripper.disconnect();
}
```

SocketCAN on Linux:

```cpp
cfg.transport_type = dh_ag95::TransportType::SocketCan;
cfg.socketcan.interface_name = "can0";
```

PCAN-Basic on Windows:

```cpp
cfg.transport_type = dh_ag95::TransportType::PcanBasic;
cfg.pcanbasic.channel = "PCAN_USBBUS1";
cfg.pcanbasic.bitrate = 500000;
```

### Command-line example

```bash
./build/ag95_cpp_example [gripper_id] [transport] [interface]
# e.g.  ./build/ag95_cpp_example 1 socketcan can0
```

---

## Python Quick Start

```bash
pip install -e python
pip install -e "python[can]"      # for socketcan / pcanbasic backends
```

```python
from dh_ag95 import Ag95Config, TransportType, Ag95Gripper

cfg = Ag95Config(transport_type=TransportType.SOCKETCAN)
cfg.gripper_id = 1
cfg.socketcan.interface_name = "can0"

g = Ag95Gripper(cfg)
g.connect()
g.initialize(wait=True)

g.set_force(30)
g.set_position(60)
print(g.get_all_state())

# Receive unsolicited events
ev = g.receive_event(timeout_s=0.0)
if ev:
    print(f"Event: {ev}")

g.disconnect()
```

Official serial:

```python
cfg = Ag95Config(transport_type=TransportType.OFFICIAL_SERIAL)
cfg.official_serial.port = "/dev/ttyACM0"
```

---

## Configuration Reference

### `Ag95Config` (C++) / `Ag95Config` (Python)

| Field | Type | Default | Description |
|---|---|---|---|
| `gripper_id` | `uint8_t` / `int` | `1` | CAN ID of the gripper (1–255, 0x01 default) |
| `transport_type` | enum | `OfficialSerial` | `OfficialSerial`, `SocketCan`, `PcanBasic`, or `ModbusRtu` |
| `command_interval` / `command_interval_ms` | ms | `20` | Minimum gap between consecutive commands (≥20 per protocol spec) |
| `read_timeout` / `read_timeout_ms` | ms | `500` | Per-attempt timeout waiting for a response |
| `max_retries` | `int` | `2` | Retry attempts per `transact()` call (0 = no retry, 2 = 3 total attempts) |
| `wait_write_echo` | `bool` | `true` | If true, `write_register()` waits for device echo via `transact()` |
| `coalesce_writes` | `bool` | `false` | Enable write-command coalescing (merge rapid writes to same register) |
| `coalesce_window` / `coalesce_window_ms` | ms | `0` | Coalescing time window; `0` disables coalescing |
| `skip_duplicate_writes` | `bool` | `true` | Skip a write if its value equals the last-sent value for the same register |
| `recv_thread_timeout` / `recv_thread_timeout_ms` | ms | `100` | Poll timeout for the background receive thread |
| `max_event_queue_size` | `size_t` / `int` | `64` | Max unsolicited frames in the event queue (older frames dropped when full) |
| `auto_initialize` | `bool` | `false` | If true, `connect()` calls `initialize(wait=true)` automatically |
| `default_force_percent` | `int` | `30` | Default gripping force |

### Transport-specific configs

- **`OfficialSerialConfig`**: `port` (device path), `baudrate` (default 115200)
- **`SocketCanConfig`**: `interface_name` (e.g. `"can0"`), `bitrate` (reference only — configure interface externally)
- **`PcanBasicConfig`**: `channel` (e.g. `"PCAN_USBBUS1"`), `bitrate`

---

## Write Coalescing

When `coalesce_writes = true` and `coalesce_window > 0`, rapid consecutive writes to the **same register** are merged:

```
Time ──────────────────────────────────────────▶
      set_pos(50)  set_pos(51)  set_pos(52)
           │            │            │
           ▼            ▼            ▼
       [defer]     [replace]    [replace]
                                        │
                              coalesce_window expires
                                        │
                                        ▼
                              send set_pos(52)  ← only one CAN frame!
```

- Different registers (e.g. position vs. force) are coalesced independently.
- A `read_register()` call **automatically flushes** all pending writes before reading, so reads always reflect the latest state.
- Call `flush()` to force-send all pending writes immediately.
- `skip_duplicate_writes = true` (default) always skips writes equal to the last-sent value, even when coalescing is disabled.

---

## Event Handling

The background receive thread pushes any frame that does **not** match a pending `transact()` request into an internal FIFO queue.

```cpp
// C++
auto ev = gripper.receive_event(std::chrono::milliseconds(100));
if (ev) { /* ev->function, ev->sub_function, ev->value … */ }
```

```python
# Python
ev = g.receive_event(timeout_s=0.1)
if ev: print(ev.function, ev.sub_function, ev.value)
```

Common unsolicited frames:
- `0x15 0x02 00 …` — object dropped (when drop detection is enabled)
- `0x08 0x02 00 …` — initialization-complete auto-feedback (disabled by default; we poll instead)

---

## Notes

- **Command interval**: defaults to 20 ms (protocol minimum).  Increase if bus reliability is a concern.
- **Force range**: 20–100 %.
- **Position range**: 0–100 % (0 = fully closed, 100 = fully open).
- **CAN ID / baud rate changes** require a gripper **power-cycle** to take effect.
- **SocketCAN setup** (Linux):
  ```bash
  sudo ip link set can0 down
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
  ```
- On Linux, prefer `socketcan` if your USB-to-CAN adapter appears as a `canX` interface.
- On Windows, use `pcanbasic` for PEAK PCAN-USB adapters.
- The Python `pcanbasic` backend requires `python-can` with the `pcan` interface.
