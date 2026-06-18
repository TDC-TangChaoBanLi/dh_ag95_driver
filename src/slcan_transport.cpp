#include "dh_ag95/slcan_transport.hpp"

#include <iomanip>
#include <sstream>

namespace dh_ag95 {
namespace {
char bitrate_to_slcan(int bitrate) {
  switch (bitrate) {
    case 10000: return '0';
    case 20000: return '1';
    case 50000: return '2';
    case 100000: return '3';
    case 125000: return '4';
    case 250000: return '5';
    case 500000: return '6';
    case 800000: return '7';
    case 1000000: return '8';
    default: throw TransportError("unsupported SLCAN bitrate");
  }
}
uint8_t hexbyte(const std::string& s, size_t pos) {
  return static_cast<uint8_t>(std::stoul(s.substr(pos, 2), nullptr, 16));
}
}

SlcanTransport::SlcanTransport(SlcanConfig config) : config_(std::move(config)) {}
SlcanTransport::~SlcanTransport() { close(); }

void SlcanTransport::open() {
  if (is_open()) return;
  serial_.open(config_.port, config_.serial_baudrate);
  serial_.flush_io();
  if (config_.configure_on_open) {
    write_line("C");
    write_line(std::string("S") + bitrate_to_slcan(config_.can_bitrate));
    write_line("O");
  }
}

void SlcanTransport::close() {
  if (serial_.is_open() && config_.configure_on_open) {
    try { write_line("C"); } catch (...) {}
  }
  serial_.close();
}

bool SlcanTransport::is_open() const { return serial_.is_open(); }

void SlcanTransport::write_line(const std::string& line) {
  std::string out = line + "\r";
  serial_.write_all(reinterpret_cast<const uint8_t*>(out.data()), out.size());
  serial_.drain_write();
}

std::string SlcanTransport::read_line(std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string line;
  while (std::chrono::steady_clock::now() < deadline) {
    auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    auto chunk = serial_.read_some(1, remain);
    if (chunk.empty()) continue;
    char c = static_cast<char>(chunk.front());
    if (c == '\r' || c == '\n') {
      if (!line.empty()) return line;
    } else {
      line.push_back(c);
    }
  }
  throw TimeoutError("timeout waiting for SLCAN line");
}

void SlcanTransport::send(const Ag95Frame& frame) {
  if (!is_open()) throw TransportError("SLCAN port is not open");
  auto raw = Ag95Protocol::to_can(frame);
  std::ostringstream oss;
  oss << 't' << std::uppercase << std::hex << std::setw(3) << std::setfill('0') << (raw.can_id & 0x7FF) << '8';
  for (auto b : raw.data) oss << std::setw(2) << static_cast<int>(b);
  write_line(oss.str());
}

Ag95Frame SlcanTransport::receive(std::chrono::milliseconds timeout) {
  while (true) {
    auto line = read_line(timeout);
    if (line.size() < 5 || line[0] != 't') continue;
    uint32_t can_id = static_cast<uint32_t>(std::stoul(line.substr(1, 3), nullptr, 16));
    int dlc = std::stoi(line.substr(4, 1), nullptr, 16);
    if (dlc != 8 || line.size() < 5 + 16) throw ProtocolError("invalid SLCAN CAN frame");
    std::array<uint8_t,8> data{};
    for (int i = 0; i < 8; ++i) data[i] = hexbyte(line, 5 + i * 2);
    return Ag95Protocol::from_can(can_id, data);
  }
}

}  // namespace dh_ag95
