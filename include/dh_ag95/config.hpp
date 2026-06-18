#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace dh_ag95 {

enum class TransportType {
  OfficialSerial,
  SocketCan,
  Slcan,
  PcanBasic,
  ModbusRtu
};

enum class CanBaudRateIndex : int32_t {
  Kbps500 = 0,
  Kbps400 = 1,
  Kbps250 = 2,
  Kbps200 = 3,
  Kbps125 = 4,
  Kbps100 = 5
};

struct OfficialSerialConfig {
  std::string port = "/dev/ttyACM0";
  int baudrate = 115200;
};

struct SocketCanConfig {
  std::string interface_name = "can0";
  int bitrate = 500000;  // for user reference; interface must be configured externally
};

struct SlcanConfig {
  std::string port = "/dev/ttyUSB0";
  int serial_baudrate = 115200;
  int can_bitrate = 500000;
  bool configure_on_open = true;
};

struct PcanBasicConfig {
  // PEAK PCAN-Basic channel name, for example PCAN_USBBUS1, PCAN_USBBUS2, or a numeric handle string such as 0x51.
  std::string channel = "PCAN_USBBUS1";
  int bitrate = 500000;
};

struct ModbusRtuConfig {
  std::string port = "/dev/ttyUSB0";
  int baudrate = 115200;
  char parity = 'N';
  int data_bits = 8;
  int stop_bits = 1;
};

struct Ag95Config {
  uint8_t gripper_id = 1;
  TransportType transport_type = TransportType::OfficialSerial;
  std::chrono::milliseconds command_interval{50};
  std::chrono::milliseconds read_timeout{500};
  bool auto_initialize = false;
  int default_force_percent = 30;

  OfficialSerialConfig official_serial;
  SocketCanConfig socketcan;
  SlcanConfig slcan;
  PcanBasicConfig pcanbasic;
  ModbusRtuConfig modbus_rtu;
};

}  // namespace dh_ag95
