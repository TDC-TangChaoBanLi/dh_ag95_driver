#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "dh_ag95/config.hpp"
#include "dh_ag95/transport.hpp"

namespace dh_ag95 {

class PcanBasicTransport : public ITransport {
 public:
  explicit PcanBasicTransport(PcanBasicConfig config);
  ~PcanBasicTransport() override;

  void open() override;
  void close() override;
  bool is_open() const override;
  void send(const Ag95Frame& frame) override;
  Ag95Frame receive(std::chrono::milliseconds timeout) override;
  std::string name() const override { return "pcanbasic"; }

 private:
  PcanBasicConfig config_;
  bool open_{false};
#ifdef _WIN32
  void* library_{nullptr};
  uint16_t channel_{0};
  uint16_t baudrate_{0};
#endif
};

}  // namespace dh_ag95
