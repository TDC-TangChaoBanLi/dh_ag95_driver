#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace dh_ag95 {

enum class TransportType {
  OfficialSerial,  ///< 14-byte framed protocol over a serial port (USB converter).
  SocketCan,       ///< Raw CAN 2.0A frames via Linux SocketCAN interface.
  PcanBasic,       ///< Raw CAN 2.0A frames via PEAK PCAN-Basic API (Windows).
  ModbusRtu        ///< Reserved – not yet implemented.
};

enum class CanBaudRateIndex : int32_t {
  Kbps500 = 0,
  Kbps400 = 1,
  Kbps250 = 2,
  Kbps200 = 3,
  Kbps125 = 4,
  Kbps100 = 5
};

/// Configuration for the official DH protocol converter (14-byte framed serial).
struct OfficialSerialConfig {
  std::string port = "/dev/ttyACM0";  ///< Serial port device path.
  int baudrate = 115200;              ///< Serial baud rate (typically 115200).
};

/// Configuration for the Linux SocketCAN interface.
struct SocketCanConfig {
  std::string interface_name = "can0"; ///< CAN network interface name (e.g. can0, vcan0).
  int bitrate = 500000;  ///< For user reference; interface must be configured externally.
};

/// Configuration for PEAK PCAN-Basic (Windows).
struct PcanBasicConfig {
  /// Channel name, e.g. "PCAN_USBBUS1" or a numeric handle string like "0x51".
  std::string channel = "PCAN_USBBUS1";
  int bitrate = 500000;  ///< CAN bitrate in bits/sec (500000, 250000, etc.).
};

/// Reserved configuration for Modbus RTU – not yet implemented.
struct ModbusRtuConfig {
  std::string port = "/dev/ttyUSB0";
  int baudrate = 115200;
  char parity = 'N';
  int data_bits = 8;
  int stop_bits = 1;
};

struct Ag95Config {
  /// @brief CAN ID of the gripper, range 1–255, default 0x01.
  uint8_t gripper_id = 1;

  /// @brief Transport layer to use for communication.
  TransportType transport_type = TransportType::OfficialSerial;

  /// @brief Minimum interval between consecutive commands (≥ 20 ms per protocol spec).
  std::chrono::milliseconds command_interval{25};

  /// @brief Per-transaction read timeout.  A single transact attempt gives up after this duration.
  std::chrono::milliseconds read_timeout{500};

  /// @brief Maximum retry attempts per transact call (0 = no retries, 2 = 3 total attempts).
  int max_retries = 2;

  /// @brief If true, write_register() waits for the device echo via transact().
  ///        If false, the write is fire-and-forget (the receive thread discards the echo).
  bool wait_write_echo = true;

  /// @brief If true, consecutive writes to the same register are coalesced.
  ///        Only the last value within coalesce_window is actually sent.
  bool coalesce_writes = false;

  /// @brief Coalescing time window.  A pending write is held for this long in case
  ///        a newer value arrives.  0 ms disables coalescing.
  std::chrono::milliseconds coalesce_window{0};

  /// @brief If true, a write whose value equals the last-sent value for the same
  ///        register is silently skipped (always-on optimization).
  bool skip_duplicate_writes = true;

  /// @brief Poll timeout for the background receive thread.  Shorter values reduce
  ///        shutdown latency; longer values reduce CPU wake-ups.
  std::chrono::milliseconds recv_thread_timeout{100};

  /// @brief Maximum number of unsolicited frames kept in the event queue.
  ///        Frames beyond this limit are silently dropped.
  size_t max_event_queue_size = 64;

  /// @brief If true, connect() automatically calls initialize(true) after opening the transport.
  bool auto_initialize = false;

  /// @brief Default gripping force percentage used when no explicit force is set.
  int default_force_percent = 30;

  OfficialSerialConfig official_serial;
  SocketCanConfig socketcan;
  PcanBasicConfig pcanbasic;
  ModbusRtuConfig modbus_rtu;
};

}  // namespace dh_ag95
