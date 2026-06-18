#pragma once

#include "dh_ag95/config.hpp"
#include "dh_ag95/transport.hpp"

namespace dh_ag95 {

class ModbusRtuTransport : public ITransport {
 public:
  explicit ModbusRtuTransport(ModbusRtuConfig config) : config_(std::move(config)) {}
  void open() override;
  void close() override {}
  bool is_open() const override { return false; }
  void send(const Ag95Frame& frame) override;
  Ag95Frame receive(std::chrono::milliseconds timeout) override;
  std::string name() const override { return "modbus_rtu_unimplemented"; }

 private:
  ModbusRtuConfig config_;
};

}  // namespace dh_ag95
