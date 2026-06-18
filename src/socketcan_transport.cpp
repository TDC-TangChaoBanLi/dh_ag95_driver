#include "dh_ag95/socketcan_transport.hpp"

#include <cerrno>
#include <cstring>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace dh_ag95 {

SocketCanTransport::SocketCanTransport(SocketCanConfig config) : config_(std::move(config)) {}
SocketCanTransport::~SocketCanTransport() { close(); }

void SocketCanTransport::open() {
  if (is_open()) return;
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) throw TransportError("failed to create CAN socket: " + std::string(std::strerror(errno)));

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, config_.interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd_); fd_ = -1;
    throw TransportError("failed to find CAN interface " + config_.interface_name + ": " + std::strerror(errno));
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd_); fd_ = -1;
    throw TransportError("failed to bind CAN socket: " + std::string(std::strerror(errno)));
  }
}

void SocketCanTransport::close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SocketCanTransport::is_open() const { return fd_ >= 0; }

void SocketCanTransport::send(const Ag95Frame& frame) {
  if (!is_open()) throw TransportError("CAN socket is not open");
  auto raw = Ag95Protocol::to_can(frame);
  can_frame cf{};
  cf.can_id = raw.can_id & CAN_SFF_MASK;
  cf.can_dlc = 8;
  for (int i = 0; i < 8; ++i) cf.data[i] = raw.data[i];
  ssize_t n = ::write(fd_, &cf, sizeof(cf));
  if (n != static_cast<ssize_t>(sizeof(cf))) throw TransportError("CAN write failed: " + std::string(std::strerror(errno)));
}

Ag95Frame SocketCanTransport::receive(std::chrono::milliseconds timeout) {
  if (!is_open()) throw TransportError("CAN socket is not open");
  pollfd pfd{fd_, POLLIN, 0};
  int pr = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (pr < 0) throw TransportError("CAN poll failed");
  if (pr == 0) throw TimeoutError("timeout waiting for CAN frame");
  can_frame cf{};
  ssize_t n = ::read(fd_, &cf, sizeof(cf));
  if (n < 0) throw TransportError("CAN read failed");
  if (cf.can_dlc != 8) throw ProtocolError("received CAN frame with DLC != 8");
  std::array<uint8_t, 8> data{};
  for (int i = 0; i < 8; ++i) data[i] = cf.data[i];
  return Ag95Protocol::from_can(cf.can_id & CAN_SFF_MASK, data);
}

}  // namespace dh_ag95
