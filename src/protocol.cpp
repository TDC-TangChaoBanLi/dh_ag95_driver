#include "dh_ag95/protocol.hpp"

#include <iomanip>
#include <sstream>

namespace dh_ag95 {

int32_t Ag95Protocol::bytes_to_value(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  uint32_t u = static_cast<uint32_t>(b0) |
               (static_cast<uint32_t>(b1) << 8) |
               (static_cast<uint32_t>(b2) << 16) |
               (static_cast<uint32_t>(b3) << 24);
  return static_cast<int32_t>(u);
}

std::vector<uint8_t> Ag95Protocol::value_to_bytes(int32_t value) {
  uint32_t u = static_cast<uint32_t>(value);
  return {static_cast<uint8_t>(u & 0xFF),
          static_cast<uint8_t>((u >> 8) & 0xFF),
          static_cast<uint8_t>((u >> 16) & 0xFF),
          static_cast<uint8_t>((u >> 24) & 0xFF)};
}

Ag95Frame Ag95Protocol::make(uint8_t id, uint8_t function, uint8_t sub_function, uint8_t rw, int32_t value) {
  if (rw != kRead && rw != kWrite) {
    throw ProtocolError("rw must be 0x00(read) or 0x01(write)");
  }
  Ag95Frame f;
  f.id = id;
  f.function = function;
  f.sub_function = sub_function;
  f.rw = rw;
  f.reserve = kReserve;
  f.value = value;
  return f;
}

RawCanFrame Ag95Protocol::to_can(const Ag95Frame& frame) {
  RawCanFrame out;
  out.can_id = frame.id;
  auto v = value_to_bytes(frame.value);
  out.data = {frame.function, frame.sub_function, frame.rw, frame.reserve, v[0], v[1], v[2], v[3]};
  out.dlc = 8;
  return out;
}

Ag95Frame Ag95Protocol::from_can(uint32_t can_id, const std::array<uint8_t, 8>& data) {
  Ag95Frame f;
  f.id = static_cast<uint8_t>(can_id & 0xFF);
  f.function = data[0];
  f.sub_function = data[1];
  f.rw = data[2];
  f.reserve = data[3];
  f.value = bytes_to_value(data[4], data[5], data[6], data[7]);
  return f;
}

std::array<uint8_t, 14> Ag95Protocol::to_official_serial(const Ag95Frame& frame) {
  auto can = to_can(frame);
  std::array<uint8_t, 14> out{};
  out[0] = 0xFF; out[1] = 0xFE; out[2] = 0xFD; out[3] = 0xFC;
  out[4] = frame.id;
  for (size_t i = 0; i < 8; ++i) out[5 + i] = can.data[i];
  out[13] = 0xFB;
  return out;
}

Ag95Frame Ag95Protocol::from_official_serial(const std::array<uint8_t, 14>& bytes) {
  if (!(bytes[0] == 0xFF && bytes[1] == 0xFE && bytes[2] == 0xFD && bytes[3] == 0xFC && bytes[13] == 0xFB)) {
    throw ProtocolError("invalid official serial frame header/footer");
  }
  std::array<uint8_t, 8> data{};
  for (size_t i = 0; i < 8; ++i) data[i] = bytes[5 + i];
  return from_can(bytes[4], data);
}

std::string Ag95Protocol::frame_to_string(const Ag95Frame& frame) {
  std::ostringstream oss;
  oss << "id=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.id)
      << " function=0x" << std::setw(2) << static_cast<int>(frame.function)
      << " sub=0x" << std::setw(2) << static_cast<int>(frame.sub_function)
      << " rw=0x" << std::setw(2) << static_cast<int>(frame.rw)
      << " value=" << std::dec << frame.value;
  return oss.str();
}

}  // namespace dh_ag95
