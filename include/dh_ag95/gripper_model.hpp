#pragma once

#include <algorithm>
#include <string>
#include <stdexcept>

namespace dh_ag95 {

/// Physical parameters for a specific gripper model.
struct GripperModelParams {
  double stroke_m = 0.095;       ///< Full jaw stroke in meters (0 = closed, stroke_m = fully open).
  double max_force_n = 160.0;    ///< Maximum gripping force in Newtons (corresponds to 100 %).
  double joint_angle_rad = 0.65; ///< Joint rotation range in radians (URDF revolute joint upper limit).
};

/// Look up the physical parameters for a given gripper model string.
/// Currently supported: "ag-160-95".
/// @throws std::invalid_argument if the model is unknown.
GripperModelParams get_gripper_params(const std::string& model);

// ---- Position conversion ---------------------------------------------------

/// Convert position percentage [0, 100] to meters (jaw opening).
inline double position_percent_to_meters(double percent, const GripperModelParams& p) {
  return std::clamp(percent, 0.0, 100.0) / 100.0 * p.stroke_m;
}

/// Convert position in meters (jaw opening) to percentage [0, 100].
inline double position_meters_to_percent(double meters, const GripperModelParams& p) {
  if (p.stroke_m <= 0.0) return 0.0;
  return std::clamp(meters / p.stroke_m * 100.0, 0.0, 100.0);
}

// ---- Force conversion ------------------------------------------------------

/// Convert force percentage [0, 100] to Newtons (linear mapping).
inline double force_percent_to_newtons(double percent, const GripperModelParams& p) {
  return std::clamp(percent, 0.0, 100.0) / 100.0 * p.max_force_n;
}

/// Convert force in Newtons to percentage [0, 100] (linear mapping).
inline double force_newtons_to_percent(double newtons, const GripperModelParams& p) {
  if (p.max_force_n <= 0.0) return 0.0;
  return std::clamp(newtons / p.max_force_n * 100.0, 0.0, 100.0);
}

// ---- Joint angle (radian) conversion ---------------------------------------
//
// Convention (matches URDF visual):  0 rad = fully OPEN,  joint_angle_rad = fully CLOSED.
// Hardware convention:               0 % = fully closed,  100 % = fully open.
//
// All four functions below are mutual inverses (round-trip consistent).

/// Convert joint angle in radians to position percentage [0, 100].
///   0 rad (open)  → 100 %,   joint_angle_rad (closed) → 0 %.
inline double position_radians_to_percent(double radians, const GripperModelParams& p) {
  if (p.joint_angle_rad <= 0.0) return 0.0;
  return 100.0 - std::clamp(radians / p.joint_angle_rad * 100.0, 0.0, 100.0);
}

/// Convert position percentage [0, 100] to joint angle in radians.
///   0 % (closed) → joint_angle_rad,   100 % (open) → 0 rad.
inline double position_percent_to_radians(double percent, const GripperModelParams& p) {
  if (p.joint_angle_rad <= 0.0) return 0.0;
  return (100.0 - std::clamp(percent, 0.0, 100.0)) / 100.0 * p.joint_angle_rad;
}

/// Convert position in meters to joint angle in radians.
///   0 m (closed) → joint_angle_rad,   stroke_m (open) → 0 rad.
inline double position_meters_to_radians(double meters, const GripperModelParams& p) {
  if (p.stroke_m <= 0.0) return 0.0;
  return (1.0 - std::clamp(meters / p.stroke_m, 0.0, 1.0)) * p.joint_angle_rad;
}

/// Convert joint angle in radians to position in meters.
///   0 rad (open) → stroke_m,   joint_angle_rad (closed) → 0 m.
inline double position_radians_to_meters(double radians, const GripperModelParams& p) {
  if (p.joint_angle_rad <= 0.0) return 0.0;
  return (1.0 - std::clamp(radians / p.joint_angle_rad, 0.0, 1.0)) * p.stroke_m;
}

}  // namespace dh_ag95
