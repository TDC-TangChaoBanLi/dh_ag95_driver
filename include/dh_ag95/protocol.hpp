#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace dh_ag95 {

constexpr uint8_t kRead = 0x00;
constexpr uint8_t kWrite = 0x01;
constexpr uint8_t kReserve = 0x00;

constexpr uint8_t kFuncForce = 0x05;
constexpr uint8_t kFuncPosition = 0x06;
constexpr uint8_t kFuncInitialization = 0x08;
constexpr uint8_t kFuncFeedback = 0x0F;
constexpr uint8_t kFuncIoMode = 0x10;
constexpr uint8_t kFuncCanId = 0x12;
constexpr uint8_t kFuncFirmwareVersion = 0x13;
constexpr uint8_t kFuncCanBaudRate = 0x14;
constexpr uint8_t kFuncObjectDropped = 0x15;

constexpr uint8_t kSubInitFeedback = 0x01;
constexpr uint8_t kSubInitialize = 0x02;
constexpr uint8_t kSubForceInternal = 0x02;
constexpr uint8_t kSubForceExternal = 0x03;
constexpr uint8_t kSubPosition = 0x02;
constexpr uint8_t kSubStatus = 0x01;
constexpr uint8_t kSubCanId = 0x01;
constexpr uint8_t kSubFirmwareVersion = 0x01;
constexpr uint8_t kSubCanBaudRate = 0x01;
constexpr uint8_t kSubDropEnable = 0x01;
constexpr uint8_t kSubDropFeedback = 0x02;

struct Ag95Frame {
  uint8_t id = 1;
  uint8_t function = 0;
  uint8_t sub_function = 0;
  uint8_t rw = kRead;
  uint8_t reserve = kReserve;
  int32_t value = 0;
};

struct RawCanFrame {
  uint32_t can_id = 0;
  std::array<uint8_t, 8> data{};
  uint8_t dlc = 8;
};

class ProtocolError : public std::runtime_error {
 public:
  explicit ProtocolError(const std::string& what) : std::runtime_error(what) {}
};

class Ag95Protocol {
 public:
  static RawCanFrame to_can(const Ag95Frame& frame);
  static Ag95Frame from_can(uint32_t can_id, const std::array<uint8_t, 8>& data);

  static std::array<uint8_t, 14> to_official_serial(const Ag95Frame& frame);
  static Ag95Frame from_official_serial(const std::array<uint8_t, 14>& bytes);

  static Ag95Frame make(uint8_t id, uint8_t function, uint8_t sub_function, uint8_t rw, int32_t value);
  static std::string frame_to_string(const Ag95Frame& frame);
  static std::vector<uint8_t> value_to_bytes(int32_t value);
  static int32_t bytes_to_value(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
};

}  // namespace dh_ag95
