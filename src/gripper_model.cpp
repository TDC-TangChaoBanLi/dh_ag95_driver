#include "dh_ag95/gripper_model.hpp"

#include <unordered_map>

namespace dh_ag95 {

GripperModelParams get_gripper_params(const std::string& model) {
  // Currently only one model is supported; extend this map for future models.
  static const std::unordered_map<std::string, GripperModelParams> kModels = {
    {"ag-160-95", {0.095, 160.0, 0.65}},
  };

  auto it = kModels.find(model);
  if (it != kModels.end()) return it->second;

  throw std::invalid_argument("Unknown gripper model: " + model +
      ". Supported: ag-160-95");
}

}  // namespace dh_ag95
