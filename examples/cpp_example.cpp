#include <iostream>

#include "dh_ag95/ag95_gripper.hpp"

int main(int argc, char** argv) {
  dh_ag95::Ag95Config cfg;
  // Default to SocketCAN on Linux; override via command line.
  cfg.transport_type = dh_ag95::TransportType::SocketCan;
  cfg.gripper_id = 1;
  cfg.socketcan.interface_name = "can0";
  
  // Allow override via command line: <program> [gripper_id] [transport_type] [port/channel]
  if (argc > 1) cfg.gripper_id = static_cast<uint8_t>(std::stoi(argv[1]));
  if (argc > 2) {
    std::string t = argv[2];
    if (t == "official_serial") cfg.transport_type = dh_ag95::TransportType::OfficialSerial;
    else if (t == "socketcan") cfg.transport_type = dh_ag95::TransportType::SocketCan;
    else if (t == "slcan") cfg.transport_type = dh_ag95::TransportType::Slcan;
    else if (t == "pcanbasic") cfg.transport_type = dh_ag95::TransportType::PcanBasic;
    else if (t == "modbus_rtu") cfg.transport_type = dh_ag95::TransportType::ModbusRtu;
  }
  if (argc > 3) {
    switch (cfg.transport_type) {
      case dh_ag95::TransportType::OfficialSerial: cfg.official_serial.port = argv[3]; break;
      case dh_ag95::TransportType::SocketCan: cfg.socketcan.interface_name = argv[3]; break;
      case dh_ag95::TransportType::Slcan: cfg.slcan.port = argv[3]; break;
      case dh_ag95::TransportType::PcanBasic: cfg.pcanbasic.channel = argv[3]; break;
      case dh_ag95::TransportType::ModbusRtu: cfg.modbus_rtu.port = argv[3]; break;
      default: break;
    }
  }

  try {
    dh_ag95::Ag95Gripper gripper(cfg);
    gripper.connect();
    std::cout << "Connected (ID=" << static_cast<int>(cfg.gripper_id) << ", transport=";
    switch (cfg.transport_type) {
      case dh_ag95::TransportType::OfficialSerial: std::cout << "official_serial"; break;
      case dh_ag95::TransportType::SocketCan: std::cout << "socketcan"; break;
      case dh_ag95::TransportType::Slcan: std::cout << "slcan"; break;
      case dh_ag95::TransportType::PcanBasic: std::cout << "pcanbasic"; break;
      case dh_ag95::TransportType::ModbusRtu: std::cout << "modbus_rtu"; break;
    }
    std::cout << ")\n";
    gripper.initialize(true);
    std::cout << "Initialized\n";
    gripper.set_force(30);
    gripper.set_position(60);
    auto state = gripper.read_state();
    std::cout << "position=" << state.position_percent 
              << " force_internal=" << state.force_internal_percent
              << " status=" << dh_ag95::status_to_string(state.status) << "\n";
    auto fw = gripper.get_firmware_version();
    std::cout << "firmware=" << fw.to_string() << "\n";
    gripper.disconnect();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
