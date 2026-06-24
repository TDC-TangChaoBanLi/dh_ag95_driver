#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

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

/**
 * @brief High-level AG-95 gripper controller.
 *
 * Manages communication lifecycle (connect / disconnect), command
 * transactions with automatic retry, and a background receive thread that
 * collects unsolicited frames (e.g. drop-detection events) into an internal
 * event queue.
 *
 * All public methods are safe to call from a single thread.  The internal
 * receive thread is started by connect() and joined by disconnect().
 */
class Ag95Gripper {
 public:
  /// Construct a gripper with the given configuration.  The transport is
  /// created automatically from config.transport_type.
  explicit Ag95Gripper(Ag95Config config);

  /// Construct a gripper with an externally provided transport (for testing /
  /// injection).
  Ag95Gripper(Ag95Config config, TransportPtr transport);

  ~Ag95Gripper();

  // --- Lifecycle -----------------------------------------------------------

  /// Open the transport, drain stale frames, start the background receive
  /// thread, and optionally auto-initialize (see Ag95Config::auto_initialize).
  void connect();

  /// Flush pending writes, stop the receive thread, and close the transport.
  void disconnect();

  /// @return true if the underlying transport is open.
  bool is_connected() const;

  // --- Low-level register access -------------------------------------------

  /// Send a frame and wait for the matching response.  Applies command
  /// interval and retries according to Ag95Config.
  /// @throws TimeoutError if no matching response is received within the
  ///         configured number of retries.
  Ag95Frame transact(const Ag95Frame& frame);

  /// Read a register.  Flushes pending coalesced writes first.
  /// @param id_override 0xFF to use the default gripper ID, otherwise the
  ///                    specified ID is used for this single transaction.
  int32_t read_register(uint8_t function, uint8_t sub_function,
                        uint8_t id_override = 0xFF);

  /// Write a register.  Behaviour depends on Ag95Config:
  /// - wait_write_echo = true  → waits for device echo via transact()
  /// - wait_write_echo = false → fire-and-forget (receive thread discards echo)
  /// - coalesce_writes enabled → value may be deferred (see flush())
  int32_t write_register(uint8_t function, uint8_t sub_function,
                         int32_t value, uint8_t id_override = 0xFF);

  // --- Initialization ------------------------------------------------------

  /// Enable or disable automatic initialization-complete feedback from the
  /// gripper (register 0x08 / sub 0x01).
  void set_initialization_feedback_enabled(bool enabled);

  /// @return true if the initialization-complete feedback is currently enabled.
  bool get_initialization_feedback_enabled();

  /// Send the initialization command.  If `wait` is true, block until the
  /// gripper reports initialization complete (polling register 0x08-0x02).
  /// @note This method first disables the auto-feedback so that polling is
  ///       used exclusively.
  void initialize(bool wait = false,
                  std::chrono::milliseconds timeout = std::chrono::seconds(10));

  /// @return true if the gripper reports that initialization has finished
  ///         (register 0x08-0x02 value == 1).
  bool is_initialized();

  // --- Force & Position ----------------------------------------------------

  /// Set the gripping force.
  /// @param percent  Force percentage [20, 100].
  /// @param sub_function  kSubForceInternal (0x02) or kSubForceExternal (0x03).
  void set_force(int percent, uint8_t sub_function = kSubForceInternal);

  /// Read the current gripping force from the device.
  /// @return Force percentage (actual value read from the gripper).
  int get_force(uint8_t sub_function = kSubForceInternal);

  /// Set the target jaw position.
  /// @param percent  Position percentage [0, 100].  0 = fully closed, 100 = fully open.
  /// @note Sending a new position while the gripper is moving interrupts the
  ///       current motion.
  void set_position(int percent);

  /// Read the current jaw position from the device.
  /// @return Position percentage (actual value read from the gripper).
  int get_position();

  // --- Status --------------------------------------------------------------

  /// Read the current gripper status register (0x0F-0x01).
  GripperStatus get_status();

  /// Convenience method: read position, internal/external force, init state,
  /// and status in a single call.  Performs 5 independent CAN transactions.
  Ag95State get_all_state();

  // --- I/O Mode ------------------------------------------------------------

  /// Enable or disable I/O control mode (register 0x10-0x09).
  void set_io_mode_enabled(bool enabled);

  /// @return true if I/O mode is currently enabled.
  bool get_io_mode_enabled();

  /// Write a generic I/O mode parameter.  Sub-function determines meaning:
  /// 0x01-0x02 / 0x05-0x06 → position [0, 100]
  /// 0x03 / 0x07 / 0x0A-0x0B → force [20, 100]
  /// 0x04 / 0x08           → control index (0 or 1)
  void set_io_parameter(uint8_t sub_function, int value);

  /// Read an I/O mode parameter.
  int get_io_parameter(uint8_t sub_function);

  /// Trigger Group 1 grip action via I/O mode (register 0x10-0x04).
  void io_control_group1(uint8_t position_index);

  /// Trigger Group 2 grip action via I/O mode (register 0x10-0x08).
  void io_control_group2(uint8_t position_index);

  // --- CAN Settings --------------------------------------------------------

  /// Set the gripper CAN ID.  **Requires a gripper restart to take effect.**
  /// @param use_broadcast_id  If true, send with CAN ID 0 (works even if the
  ///                          current ID is unknown).
  void set_can_id(uint8_t new_id, bool use_broadcast_id = false);

  /// Read the current CAN ID from the gripper.
  uint8_t get_can_id(bool use_broadcast_id = false);

  /// Read the firmware version from the gripper (register 0x13-0x01).
  FirmwareVersion get_firmware_version();

  /// Set the CAN baud rate index.  **Requires a gripper restart.**
  /// @param index  0=500K, 1=400K, 2=250K, 3=200K, 4=125K, 5=100K.
  void set_can_baudrate(CanBaudRateIndex index);

  /// Read the current CAN baud rate index from the gripper.
  CanBaudRateIndex get_can_baudrate();

  // --- Drop Detection ------------------------------------------------------

  /// Enable or disable automatic object-dropped feedback (register 0x15-0x01).
  void set_drop_detection_enabled(bool enabled);

  /// @return true if drop-detection feedback is enabled.
  bool get_drop_detection_enabled();

  /// Acknowledge / stop a drop-detection feedback notification (register
  /// 0x15-0x02 write).
  void acknowledge_drop_feedback();

  // --- Events --------------------------------------------------------------

  /// Retrieve the next unsolicited frame from the internal event queue.
  /// Unsolicited frames include drop-detection notifications and any frame
  /// that did not match a pending `transact` request.
  /// @return std::nullopt if no event is available within the timeout.
  std::optional<Ag95Frame> receive_event(std::chrono::milliseconds timeout);

  /// Flush all pending coalesced writes immediately.
  void flush();

  // --- Accessors -----------------------------------------------------------

  /// @return Read-only reference to the current configuration.
  const Ag95Config& config() const { return config_; }

  /// @return The configured gripper CAN ID.
  uint8_t gripper_id() const { return config_.gripper_id; }

  /// Override the gripper CAN ID at runtime (does not write to device).
  void set_local_gripper_id(uint8_t id) { config_.gripper_id = id; }

 private:
  struct PendingTransaction {
    Ag95Frame expected;
    Ag95Frame response;
    bool completed = false;
    std::mutex mtx;
    std::condition_variable cv;
  };

  Ag95Config config_;
  TransportPtr transport_;
  std::chrono::steady_clock::time_point last_command_time_{};

  // Background receive thread
  std::thread receive_thread_;
  std::atomic<bool> receive_running_{false};

  // Event queue for unsolicited frames (e.g. drop detection)
  std::queue<Ag95Frame> event_queue_;
  mutable std::mutex event_mutex_;
  std::condition_variable event_cv_;
  size_t max_event_queue_size_ = 64;

  // Pending transaction — set by transact(), matched by receive thread
  std::shared_ptr<PendingTransaction> pending_transaction_;
  std::mutex transact_mutex_;

  // Write coalescing cache (key = (function << 8) | sub_function)
  struct WriteCacheEntry {
    uint8_t id = 1;
    uint8_t function = 0;
    uint8_t sub_function = 0;
    int32_t last_sent_value = 0;
    std::chrono::steady_clock::time_point last_send_time;
    bool pending = false;
    int32_t pending_value = 0;
    std::chrono::steady_clock::time_point pending_since;
  };
  std::unordered_map<uint16_t, WriteCacheEntry> write_cache_;

  TransportPtr make_transport(const Ag95Config& config);
  void enforce_interval();
  void receive_loop();
  void flush_pending_writes();
  int32_t send_write_immediate(uint8_t function, uint8_t sub_function, int32_t value, uint8_t id_override);
  static bool frame_matches(const Ag95Frame& expected, const Ag95Frame& received);
  static void check_range(const std::string& name, int value, int min_v, int max_v);
};

std::string status_to_string(GripperStatus status);

}  // namespace dh_ag95
