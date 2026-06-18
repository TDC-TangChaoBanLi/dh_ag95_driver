#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "dh_ag95/config.hpp"
#include "dh_ag95/protocol.hpp"
#include "dh_ag95/transport.hpp"

namespace dh_ag95 {

enum class GripperStatus : int32_t {
  MovingOrDefault = 0,
  ReachedPosition = 2,
  GraspedObject = 3,
  Unknown = -1
};

struct Ag95State {
  int position_percent = -1;
  int force_internal_percent = -1;
  int force_external_percent = -1;
  GripperStatus status = GripperStatus::Unknown;
  int32_t raw_status = -1;
  bool initialized = false;
  bool reached = false;
  bool grasped = false;
  bool moving = false;
};

struct FirmwareVersion {
  uint32_t raw = 0;
  uint8_t b0 = 0;
  uint8_t b1 = 0;
  uint8_t b2 = 0;
  uint8_t b3 = 0;
  std::string to_string() const;
};

class Ag95Gripper {
 public:
  explicit Ag95Gripper(Ag95Config config);
  Ag95Gripper(Ag95Config config, TransportPtr transport);
  ~Ag95Gripper();

  void connect();
  void disconnect();
  bool is_connected() const;

  Ag95Frame transact(const Ag95Frame& frame);
  int32_t read_register(uint8_t function, uint8_t sub_function, uint8_t id_override = 0xFF);
  int32_t write_register(uint8_t function, uint8_t sub_function, int32_t value, uint8_t id_override = 0xFF);

  void set_initialization_feedback_enabled(bool enabled);
  bool get_initialization_feedback_enabled();
  void initialize(bool wait = false, std::chrono::milliseconds timeout = std::chrono::seconds(10));
  bool is_initialized();

  void set_force(int percent, uint8_t sub_function = kSubForceInternal);
  int get_force(uint8_t sub_function = kSubForceInternal);
  void set_position(int percent);
  int get_position();
  GripperStatus get_status();
  Ag95State read_state();

  void set_io_mode_enabled(bool enabled);
  bool get_io_mode_enabled();
  void set_io_parameter(uint8_t sub_function, int value);
  int get_io_parameter(uint8_t sub_function);
  void io_control_group1(uint8_t position_index);
  void io_control_group2(uint8_t position_index);

  void set_can_id(uint8_t new_id, bool use_broadcast_id = false);
  uint8_t get_can_id(bool use_broadcast_id = false);
  FirmwareVersion get_firmware_version();
  void set_can_baudrate(CanBaudRateIndex index);
  CanBaudRateIndex get_can_baudrate();

  void set_drop_detection_enabled(bool enabled);
  bool get_drop_detection_enabled();
  void acknowledge_drop_feedback();
  std::optional<Ag95Frame> receive_event(std::chrono::milliseconds timeout);

  const Ag95Config& config() const { return config_; }
  uint8_t gripper_id() const { return config_.gripper_id; }
  void set_local_gripper_id(uint8_t id) { config_.gripper_id = id; }

 private:
  Ag95Config config_;
  TransportPtr transport_;
  std::chrono::steady_clock::time_point last_command_time_{};

  TransportPtr make_transport(const Ag95Config& config);
  void enforce_interval();
  void send_only(const Ag95Frame& frame);
  static void check_percent(const std::string& name, int value, int min_v, int max_v);
};

std::string status_to_string(GripperStatus status);

}  // namespace dh_ag95
