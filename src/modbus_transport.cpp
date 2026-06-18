#include "dh_ag95/modbus_transport.hpp"

namespace dh_ag95 {

void ModbusRtuTransport::open() {
  throw TransportError("Modbus RTU transport is reserved but not implemented in this version");
}

void ModbusRtuTransport::send(const Ag95Frame&) {
  throw TransportError("Modbus RTU transport is reserved but not implemented in this version");
}

Ag95Frame ModbusRtuTransport::receive(std::chrono::milliseconds) {
  throw TransportError("Modbus RTU transport is reserved but not implemented in this version");
}

}  // namespace dh_ag95
