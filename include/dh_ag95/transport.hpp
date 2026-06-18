#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "dh_ag95/protocol.hpp"

namespace dh_ag95 {

class TransportError : public std::runtime_error {
 public:
  explicit TransportError(const std::string& what) : std::runtime_error(what) {}
};

class TimeoutError : public TransportError {
 public:
  explicit TimeoutError(const std::string& what) : TransportError(what) {}
};

class ITransport {
 public:
  virtual ~ITransport() = default;
  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual void send(const Ag95Frame& frame) = 0;
  virtual Ag95Frame receive(std::chrono::milliseconds timeout) = 0;
  virtual std::string name() const = 0;
};

using TransportPtr = std::shared_ptr<ITransport>;

}  // namespace dh_ag95
