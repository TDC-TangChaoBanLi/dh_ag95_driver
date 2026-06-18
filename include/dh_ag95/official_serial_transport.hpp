#pragma once

#include <string>

#include "dh_ag95/config.hpp"
#include "dh_ag95/serial_port.hpp"
#include "dh_ag95/transport.hpp"

namespace dh_ag95 {

class OfficialSerialTransport : public ITransport {
 public:
  explicit OfficialSerialTransport(OfficialSerialConfig config);
  ~OfficialSerialTransport() override;

  void open() override;
  void close() override;
  bool is_open() const override;
  void send(const Ag95Frame& frame) override;
  Ag95Frame receive(std::chrono::milliseconds timeout) override;
  std::string name() const override { return "official_serial"; }

 private:
  OfficialSerialConfig config_;
  SerialPort serial_;
};

}  // namespace dh_ag95
