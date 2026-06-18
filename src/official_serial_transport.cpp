#include "dh_ag95/official_serial_transport.hpp"

namespace dh_ag95 {

OfficialSerialTransport::OfficialSerialTransport(OfficialSerialConfig config) : config_(std::move(config)) {}
OfficialSerialTransport::~OfficialSerialTransport() { close(); }

void OfficialSerialTransport::open() {
  if (is_open()) return;
  serial_.open(config_.port, config_.baudrate);
}

void OfficialSerialTransport::close() { serial_.close(); }

bool OfficialSerialTransport::is_open() const { return serial_.is_open(); }

void OfficialSerialTransport::send(const Ag95Frame& frame) {
  auto bytes = Ag95Protocol::to_official_serial(frame);
  serial_.write_all(bytes.data(), bytes.size());
  serial_.drain_write();
}

Ag95Frame OfficialSerialTransport::receive(std::chrono::milliseconds timeout) {
  if (!is_open()) throw TransportError("serial port is not open");
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::array<uint8_t, 14> buf{};
  size_t pos = 0;
  const std::array<uint8_t,4> header{0xFF,0xFE,0xFD,0xFC};

  while (std::chrono::steady_clock::now() < deadline) {
    auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    auto chunk = serial_.read_some(1, remain);
    if (chunk.empty()) continue;
    uint8_t b = chunk.front();
    if (pos < 4) {
      if (b == header[pos]) {
        buf[pos++] = b;
      } else {
        pos = (b == header[0]) ? 1 : 0;
        if (pos == 1) buf[0] = b;
      }
    } else {
      buf[pos++] = b;
      if (pos == buf.size()) {
        return Ag95Protocol::from_official_serial(buf);
      }
    }
  }
  throw TimeoutError("timeout waiting for official serial frame");
}

}  // namespace dh_ag95
