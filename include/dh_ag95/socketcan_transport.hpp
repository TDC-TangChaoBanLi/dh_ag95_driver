#pragma once

#include <string>

#include "dh_ag95/config.hpp"
#include "dh_ag95/transport.hpp"

namespace dh_ag95 {

class SocketCanTransport : public ITransport {
 public:
  explicit SocketCanTransport(SocketCanConfig config);
  ~SocketCanTransport() override;

  void open() override;
  void close() override;
  bool is_open() const override;
  void send(const Ag95Frame& frame) override;
  Ag95Frame receive(std::chrono::milliseconds timeout) override;
  std::string name() const override { return "socketcan"; }

 private:
  SocketCanConfig config_;
  int fd_{-1};
};

}  // namespace dh_ag95
