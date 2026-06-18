#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dh_ag95 {

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  void open(const std::string& port, int baudrate);
  void close();
  bool is_open() const;

  void write_all(const uint8_t* data, std::size_t size);
  std::vector<uint8_t> read_some(std::size_t max_size, std::chrono::milliseconds timeout);
  uint8_t read_byte(std::chrono::milliseconds timeout);
  void flush_io();
  void drain_write();

 private:
#ifdef _WIN32
  void* handle_{nullptr};
#else
  int fd_{-1};
#endif
};

}  // namespace dh_ag95
