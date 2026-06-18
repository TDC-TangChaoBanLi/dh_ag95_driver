#include "dh_ag95/serial_port.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "dh_ag95/transport.hpp"

#ifdef _WIN32
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
#else
  #include <cerrno>
  #include <fcntl.h>
  #include <poll.h>
  #include <termios.h>
  #include <unistd.h>
#endif

namespace dh_ag95 {

#ifndef _WIN32
namespace {
speed_t baud_to_termios(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: throw TransportError("unsupported serial baudrate: " + std::to_string(baud));
  }
}
}
#endif

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string& port, int baudrate) {
  if (is_open()) return;
#ifdef _WIN32
  std::string full_port = port;
  if (full_port.rfind("\\\\.\\", 0) != 0) {
    full_port = "\\\\.\\" + full_port;
  }
  HANDLE h = ::CreateFileA(full_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (h == INVALID_HANDLE_VALUE) {
    throw TransportError("failed to open serial port " + port + ": Windows error " + std::to_string(::GetLastError()));
  }

  DCB dcb{};
  dcb.DCBlength = sizeof(DCB);
  if (!::GetCommState(h, &dcb)) {
    ::CloseHandle(h);
    throw TransportError("GetCommState failed for " + port);
  }
  dcb.BaudRate = static_cast<DWORD>(baudrate);
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fInX = FALSE;
  dcb.fOutX = FALSE;
  if (!::SetCommState(h, &dcb)) {
    ::CloseHandle(h);
    throw TransportError("SetCommState failed for " + port);
  }

  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = 1;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 1;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 1000;
  if (!::SetCommTimeouts(h, &timeouts)) {
    ::CloseHandle(h);
    throw TransportError("SetCommTimeouts failed for " + port);
  }
  handle_ = h;
  flush_io();
#else
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw TransportError("failed to open serial port " + port + ": " + std::strerror(errno));
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_); fd_ = -1;
    throw TransportError("tcgetattr failed for " + port);
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, baud_to_termios(baudrate));
  cfsetospeed(&tty, baud_to_termios(baudrate));
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_); fd_ = -1;
    throw TransportError("tcsetattr failed for " + port);
  }
  flush_io();
#endif
}

void SerialPort::close() {
#ifdef _WIN32
  if (handle_) {
    ::CloseHandle(static_cast<HANDLE>(handle_));
    handle_ = nullptr;
  }
#else
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

bool SerialPort::is_open() const {
#ifdef _WIN32
  return handle_ != nullptr;
#else
  return fd_ >= 0;
#endif
}

void SerialPort::write_all(const uint8_t* data, std::size_t size) {
  if (!is_open()) throw TransportError("serial port is not open");
#ifdef _WIN32
  DWORD written = 0;
  if (!::WriteFile(static_cast<HANDLE>(handle_), data, static_cast<DWORD>(size), &written, nullptr) || written != size) {
    throw TransportError("serial write failed: Windows error " + std::to_string(::GetLastError()));
  }
#else
  std::size_t total = 0;
  while (total < size) {
    ssize_t n = ::write(fd_, data + total, size - total);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
      throw TransportError("serial write failed: " + std::string(std::strerror(errno)));
    }
    total += static_cast<std::size_t>(n);
  }
#endif
}

std::vector<uint8_t> SerialPort::read_some(std::size_t max_size, std::chrono::milliseconds timeout) {
  if (!is_open()) throw TransportError("serial port is not open");
  if (max_size == 0) return {};
#ifdef _WIN32
  auto deadline = std::chrono::steady_clock::now() + timeout;
  std::vector<uint8_t> buf(max_size);
  while (std::chrono::steady_clock::now() < deadline) {
    DWORD errors = 0;
    COMSTAT stat{};
    ::ClearCommError(static_cast<HANDLE>(handle_), &errors, &stat);
    DWORD to_read = std::min<DWORD>(static_cast<DWORD>(max_size), stat.cbInQue > 0 ? stat.cbInQue : 1);
    DWORD read = 0;
    if (!::ReadFile(static_cast<HANDLE>(handle_), buf.data(), to_read, &read, nullptr)) {
      throw TransportError("serial read failed: Windows error " + std::to_string(::GetLastError()));
    }
    if (read > 0) {
      buf.resize(read);
      return buf;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return {};
#else
  pollfd pfd{fd_, POLLIN, 0};
  int pr = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (pr < 0) throw TransportError("serial poll failed: " + std::string(std::strerror(errno)));
  if (pr == 0) return {};
  std::vector<uint8_t> buf(max_size);
  ssize_t n = ::read(fd_, buf.data(), buf.size());
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return {};
    throw TransportError("serial read failed: " + std::string(std::strerror(errno)));
  }
  buf.resize(static_cast<std::size_t>(n));
  return buf;
#endif
}

uint8_t SerialPort::read_byte(std::chrono::milliseconds timeout) {
  auto v = read_some(1, timeout);
  if (v.empty()) throw TimeoutError("timeout waiting for serial byte");
  return v.front();
}

void SerialPort::flush_io() {
  if (!is_open()) return;
#ifdef _WIN32
  ::PurgeComm(static_cast<HANDLE>(handle_), PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
  tcflush(fd_, TCIOFLUSH);
#endif
}

void SerialPort::drain_write() {
  if (!is_open()) return;
#ifdef _WIN32
  ::FlushFileBuffers(static_cast<HANDLE>(handle_));
#else
  tcdrain(fd_);
#endif
}

}  // namespace dh_ag95
