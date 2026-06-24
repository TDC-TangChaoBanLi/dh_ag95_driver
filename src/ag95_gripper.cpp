#include "dh_ag95/ag95_gripper.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>
#include <vector>

#include "dh_ag95/modbus_transport.hpp"
#include "dh_ag95/official_serial_transport.hpp"
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
      throw TransportError("SocketCAN is Linux-only; on Windows use transport_type=pcanbasic for PEAK PCAN devices or official_serial");
#endif
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
      catch (const TimeoutError&) { continue; }
    }
  }
  // Start background receive thread.
  receive_running_.store(true, std::memory_order_relaxed);
  receive_thread_ = std::thread(&Ag95Gripper::receive_loop, this);

  last_command_time_ = std::chrono::steady_clock::now() - config_.command_interval;
  if (config_.auto_initialize) initialize(true);
}

void Ag95Gripper::disconnect() {
  // Flush any pending coalesced writes before shutting down.
  flush_pending_writes();

  // Stop receive thread first.
  receive_running_.store(false, std::memory_order_relaxed);
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  // Abort any pending transaction.
  {
    std::lock_guard<std::mutex> lock(transact_mutex_);
    pending_transaction_.reset();
  }
  // Clear event queue.
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    std::queue<Ag95Frame>().swap(event_queue_);
  }
  if (transport_) transport_->close();
}
bool Ag95Gripper::is_connected() const { return transport_ && transport_->is_open(); }

void Ag95Gripper::enforce_interval() {
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = now - last_command_time_;
  if (elapsed < config_.command_interval) std::this_thread::sleep_for(config_.command_interval - elapsed);
  last_command_time_ = std::chrono::steady_clock::now();
}

bool Ag95Gripper::frame_matches(const Ag95Frame& expected, const Ag95Frame& received) {
  return received.id == expected.id &&
         received.function == expected.function &&
         received.sub_function == expected.sub_function &&
         received.rw == expected.rw;
}

void Ag95Gripper::receive_loop() {
  while (receive_running_.load(std::memory_order_relaxed)) {
    try {
      auto frame = transport_->receive(config_.recv_thread_timeout);

      // Check if this frame matches a pending transaction.
      std::shared_ptr<PendingTransaction> pt;
      {
        std::lock_guard<std::mutex> lock(transact_mutex_);
        if (pending_transaction_ && frame_matches(pending_transaction_->expected, frame)) {
          pt = pending_transaction_;
        }
      }

      if (pt) {
        // Deliver to the waiting transact caller.
        std::lock_guard<std::mutex> pt_lock(pt->mtx);
        pt->response = frame;
        pt->completed = true;
        pt->cv.notify_one();
        continue;
      }

      // Unsolicited frame — push to event queue.
      {
        std::lock_guard<std::mutex> lock(event_mutex_);
        if (event_queue_.size() < max_event_queue_size_) {
          event_queue_.push(frame);
        }
        event_cv_.notify_one();
      }
    } catch (const TimeoutError&) {
      // Normal poll timeout, loop continues.
    } catch (const std::exception&) {
      // Transport error; if still running, sleep briefly to avoid busy spin.
      if (receive_running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
}

Ag95Frame Ag95Gripper::transact(const Ag95Frame& frame) {
  if (!is_connected()) throw TransportError("gripper transport is not connected");

  auto pt = std::make_shared<PendingTransaction>();
  pt->expected = frame;

  {
    std::lock_guard<std::mutex> lock(transact_mutex_);
    pending_transaction_ = pt;
  }

  for (int attempt = 0; attempt <= config_.max_retries; ++attempt) {
    try {
      enforce_interval();
      transport_->send(frame);

      std::unique_lock<std::mutex> pt_lock(pt->mtx);
      const auto deadline = std::chrono::steady_clock::now() + config_.read_timeout;

      while (!pt->completed) {
        if (pt->cv.wait_until(pt_lock, deadline) == std::cv_status::timeout) {
          break;
        }
      }

      if (pt->completed) {
        std::lock_guard<std::mutex> lock(transact_mutex_);
        pending_transaction_.reset();
        return pt->response;
      }
    } catch (const ProtocolError&) {
      // Will retry below.
    }

    if (attempt < config_.max_retries) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  {
    std::lock_guard<std::mutex> lock(transact_mutex_);
    pending_transaction_.reset();
  }
  throw TimeoutError("timeout waiting for matching CAN frame after " +
                     std::to_string(config_.max_retries + 1) + " attempts");
}

int32_t Ag95Gripper::read_register(uint8_t function, uint8_t sub_function, uint8_t id_override) {
  // Flush any pending coalesced writes before reading so the read reflects
  // the latest written state.
  flush_pending_writes();
  uint8_t id = (id_override == 0xFF) ? config_.gripper_id : id_override;
  return transact(Ag95Protocol::make(id, function, sub_function, kRead, 0)).value;
}

int32_t Ag95Gripper::send_write_immediate(uint8_t function, uint8_t sub_function, int32_t value, uint8_t id_override) {
  uint8_t id = (id_override == 0xFF) ? config_.gripper_id : id_override;
  auto frame = Ag95Protocol::make(id, function, sub_function, kWrite, value);
  if (config_.wait_write_echo) {
    return transact(frame).value;
  }
  // Fire-and-forget: send without waiting.  The receive thread discards the echo.
  enforce_interval();
  transport_->send(frame);
  return value;
}

int32_t Ag95Gripper::write_register(uint8_t function, uint8_t sub_function, int32_t value, uint8_t id_override) {
  uint8_t id = (id_override == 0xFF) ? config_.gripper_id : id_override;

  // If coalescing is disabled, send immediately.
  if (!config_.coalesce_writes || config_.coalesce_window.count() == 0) {
    return send_write_immediate(function, sub_function, value, id_override);
  }

  uint16_t key = static_cast<uint16_t>((function << 8) | sub_function);
  auto& cache = write_cache_[key];
  auto now = std::chrono::steady_clock::now();

  // Initialize cache entry on first use.
  if (cache.last_send_time.time_since_epoch().count() == 0) {
    cache.id = id;
    cache.function = function;
    cache.sub_function = sub_function;
  }

  // --- Skip duplicate value (always-on, safe optimization) ---
  if (config_.skip_duplicate_writes && !cache.pending &&
      cache.last_sent_value == value) {
    return value;
  }

  // --- Already have a pending write for this register ---
  if (cache.pending) {
    if (cache.pending_value == value) {
      // Same value, just refresh the timestamp.
      cache.pending_since = now;
      return value;
    }
    if (now - cache.pending_since < config_.coalesce_window) {
      // Different value within window → replace the pending value.
      cache.pending_value = value;
      cache.pending_since = now;
      return value;
    }
    // Window expired — flush the old pending first, then set new.
    flush_pending_writes();
  }

  // --- Check if recently sent the same value ---
  if (cache.last_sent_value == value &&
      now - cache.last_send_time < config_.coalesce_window) {
    return value;
  }

  // --- Defer this write ---
  cache.pending = true;
  cache.pending_value = value;
  cache.pending_since = now;
  return value;
}

void Ag95Gripper::flush_pending_writes() {
  // Collect keys with pending writes first (send may modify map).
  std::vector<uint16_t> keys;
  for (const auto& [key, entry] : write_cache_) {
    if (entry.pending) keys.push_back(key);
  }
  for (auto key : keys) {
    auto it = write_cache_.find(key);
    if (it == write_cache_.end() || !it->second.pending) continue;
    auto& cache = it->second;
    send_write_immediate(cache.function, cache.sub_function, cache.pending_value, cache.id);
    cache.last_sent_value = cache.pending_value;
    cache.last_send_time = std::chrono::steady_clock::now();
    cache.pending = false;
  }
}

void Ag95Gripper::flush() {
  flush_pending_writes();
}

void Ag95Gripper::check_range(const std::string& name, int value, int min_v, int max_v) {
  if (value < min_v || value > max_v) throw std::out_of_range(name + " must be in [" + std::to_string(min_v) + ", " + std::to_string(max_v) + "]");
}

void Ag95Gripper::set_initialization_feedback_enabled(bool enabled) { write_register(kFuncInitialization, kSubInitFeedback, enabled ? 1 : 0); }
bool Ag95Gripper::get_initialization_feedback_enabled() { return read_register(kFuncInitialization, kSubInitFeedback) != 0; }

void Ag95Gripper::initialize(bool wait, std::chrono::milliseconds timeout) {
  // Disable initialization-complete auto-feedback; we will poll status instead.
  write_register(kFuncInitialization, kSubInitFeedback, 0);
  // Send the initialization command.
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
  check_range("force", percent, 20, 100);
  if (sub_function != kSubForceInternal && sub_function != kSubForceExternal) throw std::invalid_argument("force sub_function must be 0x02 or 0x03");
  write_register(kFuncForce, sub_function, percent);
}

int Ag95Gripper::get_force(uint8_t sub_function) {
  if (sub_function != kSubForceInternal && sub_function != kSubForceExternal) throw std::invalid_argument("force sub_function must be 0x02 or 0x03");
  return static_cast<int>(read_register(kFuncForce, sub_function));
}

void Ag95Gripper::set_position(int percent) {
  check_range("position", percent, 0, 100);
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

Ag95State Ag95Gripper::get_all_state() {
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
    case 0x01: case 0x02: case 0x05: case 0x06: check_range("I/O position", value, 0, 100); break;
    case 0x03: case 0x07: case 0x0A: case 0x0B: check_range("I/O force", value, 20, 100); break;
    case 0x04: case 0x08: check_range("I/O control index", value, 0, 1); break;
    case 0x09: check_range("I/O enable", value, 0, 1); break;
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
  std::unique_lock<std::mutex> lock(event_mutex_);
  if (!event_queue_.empty()) {
    auto frame = event_queue_.front();
    event_queue_.pop();
    return frame;
  }
  if (timeout.count() == 0) return std::nullopt;
  if (event_cv_.wait_for(lock, timeout) == std::cv_status::timeout) {
    return std::nullopt;
  }
  if (!event_queue_.empty()) {
    auto frame = event_queue_.front();
    event_queue_.pop();
    return frame;
  }
  return std::nullopt;
}

}  // namespace dh_ag95
