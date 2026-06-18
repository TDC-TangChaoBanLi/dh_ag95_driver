#pragma once

#include <string>

#include "dh_ag95/config.hpp"
#include "dh_ag95/serial_port.hpp"
#include "dh_ag95/transport.hpp"

namespace dh_ag95 {

class SlcanTransport : public ITransport {
 public:
  explicit SlcanTransport(SlcanConfig config);
  ~SlcanTransport() override;

  void open() override;
  void close() override;
  bool is_open() const override;
  void send(const Ag95Frame& frame) override;
  Ag95Frame receive(std::chrono::milliseconds timeout) override;
  std::string name() const override { return "slcan"; }

 private:
  SlcanConfig config_;
  SerialPort serial_;

  void write_line(const std::string& line);
  std::string read_line(std::chrono::milliseconds timeout);
};

}  // namespace dh_ag95
