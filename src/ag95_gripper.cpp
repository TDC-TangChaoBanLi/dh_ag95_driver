#include "dh_ag95/ag95_gripper.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

#include "dh_ag95/modbus_transport.hpp"
#include "dh_ag95/official_serial_transport.hpp"
#include "dh_ag95/slcan_transport.hpp"
#include "dh_ag95/pcanbasic_transport.hpp"
#ifndef _WIN32
#include "dh_ag95/socketcan_transport.hpp"
#endif

namespace dh_ag95 {

std::string FirmwareVersion::to_string() const {
  std::ostringstream oss;
  oss << static_cast<int>(b3) << "." << static_cast<int>(b2) << "." << static_cast<int>(b1) << "." << static_cast<int>(b0);
  return oss.str();
}

std::string status_to_string(GripperStatus status) {
  switch (status) {
    case GripperStatus::MovingOrDefault: return "default_or_moving";
    case GripperStatus::ReachedPosition: return "reached_position_not_grasped";
    case GripperStatus::GraspedObject: return "grasped_object_not_reached_position";
    default: return "unknown";
  }
}

Ag95Gripper::Ag95Gripper(Ag95Config config) : config_(std::move(config)), transport_(make_transport(config_)) {}
Ag95Gripper::Ag95Gripper(Ag95Config config, TransportPtr transport) : config_(std::move(config)), transport_(std::move(transport)) {}
Ag95Gripper::~Ag95Gripper() { try { disconnect(); } catch (...) {} }

TransportPtr Ag95Gripper::make_transport(const Ag95Config& config) {
  switch (config.transport_type) {
    case TransportType::OfficialSerial: return std::make_shared<OfficialSerialTransport>(config.official_serial);
    case TransportType::SocketCan:
#ifndef _WIN32
      return std::make_shared<SocketCanTransport>(config.socketcan);
#else
      throw TransportError("SocketCAN is Linux-only; on Windows use transport_type=pcanbasic for PEAK PCAN devices or official_serial/slcan when appropriate");
#endif
    case TransportType::Slcan: return std::make_shared<SlcanTransport>(config.slcan);
    case TransportType::PcanBasic: return std::make_shared<PcanBasicTransport>(config.pcanbasic);
    case TransportType::ModbusRtu: return std::make_shared<ModbusRtuTransport>(config.modbus_rtu);
  }
  throw std::invalid_argument("unknown transport type");
}

void Ag95Gripper::connect() {
  transport_->open();
  // Drain stale frames and let the bus settle before sending commands.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  {
    auto drain_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < drain_end) {
      try { transport_->receive(std::chrono::milliseconds(10)); }
      catch (const TimeoutError&) { break; }
    }
  }
  last_command_time_ = std::chrono::steady_clock::now() - config_.command_interval;
  if (config_.auto_initialize) initialize(true);
}

void Ag95Gripper::disconnect() { if (transport_) transport_->close(); }
bool Ag95Gripper::is_connected() const { return transport_ && transport_->is_open(); }

void Ag95Gripper::enforce_interval() {
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = now - last_command_time_;
  if (elapsed < config_.command_interval) std::this_thread::sleep_for(config_.command_interval - elapsed);
  last_command_time_ = std::chrono::steady_clock::now();
}

Ag95Frame Ag95Gripper::transact(const Ag95Frame& frame) {
  if (!is_connected()) throw TransportError("gripper transport is not connected");
  constexpr int kMaxRetries = 4;
  for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
    try {
      enforce_interval();
      transport_->send(frame);
      const auto deadline = std::chrono::steady_clock::now() + config_.read_timeout;
      // Read frames until we find a matching response or time out.
      // Non-matching frames (unsolicited status/event frames from the gripper)
      // are discarded so the correct response is not missed.
      while (std::chrono::steady_clock::now() < deadline) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0) break;
        try {
          auto resp = transport_->receive(remaining);
          if (resp.id == frame.id && resp.function == frame.function &&
              resp.sub_function == frame.sub_function && resp.rw == frame.rw) {
            return resp;
          }
          // non-matching → discard, re-enter receive with updated remaining
        } catch (const TimeoutError&) {
          break;  // deadline reached
        }
      }
      throw TimeoutError("timeout waiting for matching CAN frame");
    } catch (const TimeoutError&) {
      if (attempt == kMaxRetries) throw;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (const ProtocolError&) {
      if (attempt == kMaxRetries) throw;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  throw TransportError("unreachable");
}

int32_t Ag95Gripper::read_register(uint8_t function, uint8_t sub_function, uint8_t id_override) {
  uint8_t id = (id_override == 0xFF) ? config_.gripper_id : id_override;
  return transact(Ag95Protocol::make(id, function, sub_function, kRead, 0)).value;
}

void Ag95Gripper::send_only(const Ag95Frame& frame) {
  if (!is_connected()) throw TransportError("gripper transport is not connected");
  enforce_interval();
  transport_->send(frame);
}

int32_t Ag95Gripper::write_register(uint8_t function, uint8_t sub_function, int32_t value, uint8_t id_override) {
  uint8_t id = (id_override == 0xFF) ? config_.gripper_id : id_override;
  send_only(Ag95Protocol::make(id, function, sub_function, kWrite, value));
  return value;
}

void Ag95Gripper::check_percent(const std::string& name, int value, int min_v, int max_v) {
  if (value < min_v || value > max_v) throw std::out_of_range(name + " must be in [" + std::to_string(min_v) + ", " + std::to_string(max_v) + "]");
}

void Ag95Gripper::set_initialization_feedback_enabled(bool enabled) { write_register(kFuncInitialization, kSubInitFeedback, enabled ? 1 : 0); }
bool Ag95Gripper::get_initialization_feedback_enabled() { return read_register(kFuncInitialization, kSubInitFeedback) != 0; }

void Ag95Gripper::initialize(bool wait, std::chrono::milliseconds timeout) {
  write_register(kFuncInitialization, kSubInitialize, 0);
  if (!wait) return;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      if (is_initialized()) return;
    } catch (const TimeoutError&) {
      // gripper may be busy calibrating, retry after sleep
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  throw TimeoutError("timeout waiting for initialization");
}

bool Ag95Gripper::is_initialized() { return read_register(kFuncInitialization, kSubInitialize) == 1; }

void Ag95Gripper::set_force(int percent, uint8_t sub_function) {
  check_percent("force", percent, 20, 100);
  if (sub_function != kSubForceInternal && sub_function != kSubForceExternal) throw std::invalid_argument("force sub_function must be 0x02 or 0x03");
  write_register(kFuncForce, sub_function, percent);
}

int Ag95Gripper::get_force(uint8_t sub_function) {
  if (sub_function != kSubForceInternal && sub_function != kSubForceExternal) throw std::invalid_argument("force sub_function must be 0x02 or 0x03");
  return static_cast<int>(read_register(kFuncForce, sub_function));
}

void Ag95Gripper::set_position(int percent) {
  check_percent("position", percent, 0, 100);
  write_register(kFuncPosition, kSubPosition, percent);
}

int Ag95Gripper::get_position() { return static_cast<int>(read_register(kFuncPosition, kSubPosition)); }

GripperStatus Ag95Gripper::get_status() {
  int32_t raw = read_register(kFuncFeedback, kSubStatus);
  if (raw == 0) return GripperStatus::MovingOrDefault;
  if (raw == 2) return GripperStatus::ReachedPosition;
  if (raw == 3) return GripperStatus::GraspedObject;
  return GripperStatus::Unknown;
}

Ag95State Ag95Gripper::read_state() {
  Ag95State s;
  try { s.position_percent = get_position(); } catch (...) {}
  try { s.force_internal_percent = get_force(kSubForceInternal); } catch (...) {}
  try { s.force_external_percent = get_force(kSubForceExternal); } catch (...) {}
  try { s.initialized = is_initialized(); } catch (...) {}
  try {
    s.raw_status = read_register(kFuncFeedback, kSubStatus);
    if (s.raw_status == 0) s.status = GripperStatus::MovingOrDefault;
    else if (s.raw_status == 2) s.status = GripperStatus::ReachedPosition;
    else if (s.raw_status == 3) s.status = GripperStatus::GraspedObject;
    else s.status = GripperStatus::Unknown;
  } catch (...) {}
  s.reached = s.status == GripperStatus::ReachedPosition;
  s.grasped = s.status == GripperStatus::GraspedObject;
  s.moving = s.status == GripperStatus::MovingOrDefault;
  return s;
}

void Ag95Gripper::set_io_mode_enabled(bool enabled) { write_register(kFuncIoMode, 0x09, enabled ? 1 : 0); }
bool Ag95Gripper::get_io_mode_enabled() { return read_register(kFuncIoMode, 0x09) != 0; }

void Ag95Gripper::set_io_parameter(uint8_t sub_function, int value) {
  switch (sub_function) {
    case 0x01: case 0x02: case 0x05: case 0x06: check_percent("I/O position", value, 0, 100); break;
    case 0x03: case 0x07: case 0x0A: case 0x0B: check_percent("I/O force", value, 20, 100); break;
    case 0x04: case 0x08: check_percent("I/O control index", value, 0, 1); break;
    case 0x09: check_percent("I/O enable", value, 0, 1); break;
    default: throw std::invalid_argument("unsupported I/O sub_function");
  }
  write_register(kFuncIoMode, sub_function, value);
}

int Ag95Gripper::get_io_parameter(uint8_t sub_function) {
  if (sub_function < 0x01 || sub_function > 0x0B) throw std::invalid_argument("unsupported I/O sub_function");
  return static_cast<int>(read_register(kFuncIoMode, sub_function));
}

void Ag95Gripper::io_control_group1(uint8_t position_index) { set_io_parameter(0x04, position_index ? 1 : 0); }
void Ag95Gripper::io_control_group2(uint8_t position_index) { set_io_parameter(0x08, position_index ? 1 : 0); }

void Ag95Gripper::set_can_id(uint8_t new_id, bool use_broadcast_id) {
  if (new_id < 1) throw std::out_of_range("CAN ID must be 1..255");
  write_register(kFuncCanId, kSubCanId, new_id, use_broadcast_id ? 0 : 0xFF);
}

uint8_t Ag95Gripper::get_can_id(bool use_broadcast_id) {
  return static_cast<uint8_t>(read_register(kFuncCanId, kSubCanId, use_broadcast_id ? 0 : 0xFF));
}

FirmwareVersion Ag95Gripper::get_firmware_version() {
  int32_t raw_signed = read_register(kFuncFirmwareVersion, kSubFirmwareVersion);
  uint32_t raw = static_cast<uint32_t>(raw_signed);
  FirmwareVersion v;
  v.raw = raw;
  v.b0 = raw & 0xFF;
  v.b1 = (raw >> 8) & 0xFF;
  v.b2 = (raw >> 16) & 0xFF;
  v.b3 = (raw >> 24) & 0xFF;
  return v;
}

void Ag95Gripper::set_can_baudrate(CanBaudRateIndex index) {
  int32_t v = static_cast<int32_t>(index);
  if (v < 0 || v > 5) throw std::out_of_range("CAN baudrate index must be 0..5");
  write_register(kFuncCanBaudRate, kSubCanBaudRate, v);
}

CanBaudRateIndex Ag95Gripper::get_can_baudrate() {
  int32_t v = read_register(kFuncCanBaudRate, kSubCanBaudRate);
  if (v < 0 || v > 5) throw ProtocolError("invalid CAN baudrate index returned by gripper");
  return static_cast<CanBaudRateIndex>(v);
}

void Ag95Gripper::set_drop_detection_enabled(bool enabled) { write_register(kFuncObjectDropped, kSubDropEnable, enabled ? 1 : 0); }
bool Ag95Gripper::get_drop_detection_enabled() { return read_register(kFuncObjectDropped, kSubDropEnable) != 0; }
void Ag95Gripper::acknowledge_drop_feedback() { write_register(kFuncObjectDropped, kSubDropFeedback, 0); }

std::optional<Ag95Frame> Ag95Gripper::receive_event(std::chrono::milliseconds timeout) {
  if (!is_connected()) throw TransportError("gripper transport is not connected");
  try { return transport_->receive(timeout); }
  catch (const TimeoutError&) { return std::nullopt; }
}

}  // namespace dh_ag95
