#include "dh_ag95/pcanbasic_transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <string>
#include <thread>

#ifdef _WIN32
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
#endif

namespace dh_ag95 {

#ifdef _WIN32
namespace {
using TPCANHandle = uint16_t;
using TPCANStatus = uint32_t;
using TPCANBaudrate = uint16_t;
using TPCANMessageType = uint8_t;

constexpr TPCANStatus PCAN_ERROR_OK = 0x00000U;
constexpr TPCANStatus PCAN_ERROR_QRCVEMPTY = 0x00020U;
constexpr TPCANMessageType PCAN_MESSAGE_STANDARD = 0x00U;

constexpr TPCANHandle PCAN_USBBUS1 = 0x51;
constexpr TPCANHandle PCAN_USBBUS2 = 0x52;
constexpr TPCANHandle PCAN_USBBUS3 = 0x53;
constexpr TPCANHandle PCAN_USBBUS4 = 0x54;
constexpr TPCANHandle PCAN_USBBUS5 = 0x55;
constexpr TPCANHandle PCAN_USBBUS6 = 0x56;
constexpr TPCANHandle PCAN_USBBUS7 = 0x57;
constexpr TPCANHandle PCAN_USBBUS8 = 0x58;
constexpr TPCANHandle PCAN_USBBUS9 = 0x509;
constexpr TPCANHandle PCAN_USBBUS10 = 0x50A;
constexpr TPCANHandle PCAN_USBBUS11 = 0x50B;
constexpr TPCANHandle PCAN_USBBUS12 = 0x50C;
constexpr TPCANHandle PCAN_USBBUS13 = 0x50D;
constexpr TPCANHandle PCAN_USBBUS14 = 0x50E;
constexpr TPCANHandle PCAN_USBBUS15 = 0x50F;
constexpr TPCANHandle PCAN_USBBUS16 = 0x510;

constexpr TPCANBaudrate PCAN_BAUD_1M = 0x0014;
constexpr TPCANBaudrate PCAN_BAUD_800K = 0x0016;
constexpr TPCANBaudrate PCAN_BAUD_500K = 0x001C;
constexpr TPCANBaudrate PCAN_BAUD_250K = 0x011C;
constexpr TPCANBaudrate PCAN_BAUD_125K = 0x031C;
constexpr TPCANBaudrate PCAN_BAUD_100K = 0x432F;

struct TPCANMsg {
  uint32_t ID;
  TPCANMessageType MSGTYPE;
  uint8_t LEN;
  uint8_t DATA[8];
};

using CAN_Initialize_t = TPCANStatus (__stdcall *)(TPCANHandle, TPCANBaudrate, uint8_t, uint32_t, uint16_t);
using CAN_Uninitialize_t = TPCANStatus (__stdcall *)(TPCANHandle);
using CAN_Write_t = TPCANStatus (__stdcall *)(TPCANHandle, TPCANMsg*);
using CAN_Read_t = TPCANStatus (__stdcall *)(TPCANHandle, TPCANMsg*, void*);

CAN_Initialize_t p_CAN_Initialize = nullptr;
CAN_Uninitialize_t p_CAN_Uninitialize = nullptr;
CAN_Write_t p_CAN_Write = nullptr;
CAN_Read_t p_CAN_Read = nullptr;

std::string uppercase(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
  return s;
}

TPCANHandle parse_channel(const std::string& channel) {
  auto s = uppercase(channel);
  if (s == "PCAN_USBBUS1") return PCAN_USBBUS1;
  if (s == "PCAN_USBBUS2") return PCAN_USBBUS2;
  if (s == "PCAN_USBBUS3") return PCAN_USBBUS3;
  if (s == "PCAN_USBBUS4") return PCAN_USBBUS4;
  if (s == "PCAN_USBBUS5") return PCAN_USBBUS5;
  if (s == "PCAN_USBBUS6") return PCAN_USBBUS6;
  if (s == "PCAN_USBBUS7") return PCAN_USBBUS7;
  if (s == "PCAN_USBBUS8") return PCAN_USBBUS8;
  if (s == "PCAN_USBBUS9") return PCAN_USBBUS9;
  if (s == "PCAN_USBBUS10") return PCAN_USBBUS10;
  if (s == "PCAN_USBBUS11") return PCAN_USBBUS11;
  if (s == "PCAN_USBBUS12") return PCAN_USBBUS12;
  if (s == "PCAN_USBBUS13") return PCAN_USBBUS13;
  if (s == "PCAN_USBBUS14") return PCAN_USBBUS14;
  if (s == "PCAN_USBBUS15") return PCAN_USBBUS15;
  if (s == "PCAN_USBBUS16") return PCAN_USBBUS16;
  return static_cast<TPCANHandle>(std::stoul(channel, nullptr, 0));
}

TPCANBaudrate parse_bitrate(int bitrate) {
  switch (bitrate) {
    case 1000000: return PCAN_BAUD_1M;
    case 800000: return PCAN_BAUD_800K;
    case 500000: return PCAN_BAUD_500K;
    case 250000: return PCAN_BAUD_250K;
    case 125000: return PCAN_BAUD_125K;
    case 100000: return PCAN_BAUD_100K;
    default: throw TransportError("unsupported PCANBasic bitrate: " + std::to_string(bitrate));
  }
}

void* symbol(void* library, const char* name) {
  auto p = ::GetProcAddress(static_cast<HMODULE>(library), name);
  if (!p) throw TransportError(std::string("PCANBasic symbol not found: ") + name);
  return reinterpret_cast<void*>(p);
}
}
#endif

PcanBasicTransport::PcanBasicTransport(PcanBasicConfig config) : config_(std::move(config)) {}
PcanBasicTransport::~PcanBasicTransport() { close(); }

void PcanBasicTransport::open() {
#ifdef _WIN32
  if (open_) return;
  library_ = ::LoadLibraryA("PCANBasic.dll");
  if (!library_) throw TransportError("failed to load PCANBasic.dll; install PEAK PCAN driver/PCAN-Basic runtime");
  p_CAN_Initialize = reinterpret_cast<CAN_Initialize_t>(symbol(library_, "CAN_Initialize"));
  p_CAN_Uninitialize = reinterpret_cast<CAN_Uninitialize_t>(symbol(library_, "CAN_Uninitialize"));
  p_CAN_Write = reinterpret_cast<CAN_Write_t>(symbol(library_, "CAN_Write"));
  p_CAN_Read = reinterpret_cast<CAN_Read_t>(symbol(library_, "CAN_Read"));
  channel_ = parse_channel(config_.channel);
  baudrate_ = parse_bitrate(config_.bitrate);
  auto status = p_CAN_Initialize(channel_, baudrate_, 0, 0, 0);
  if (status != PCAN_ERROR_OK) {
    ::FreeLibrary(static_cast<HMODULE>(library_)); library_ = nullptr;
    throw TransportError("CAN_Initialize failed, PCAN status=0x" + std::to_string(status));
  }
  open_ = true;
#else
  throw TransportError("PCANBasic C++ transport is only implemented on Windows in this package; on Linux use transport_type=socketcan with the peak_usb/SocketCAN driver, or use Python python-can pcan backend.");
#endif
}

void PcanBasicTransport::close() {
#ifdef _WIN32
  if (open_ && p_CAN_Uninitialize) p_CAN_Uninitialize(channel_);
  open_ = false;
  if (library_) { ::FreeLibrary(static_cast<HMODULE>(library_)); library_ = nullptr; }
#else
  open_ = false;
#endif
}

bool PcanBasicTransport::is_open() const { return open_; }

void PcanBasicTransport::send(const Ag95Frame& frame) {
#ifdef _WIN32
  if (!open_) throw TransportError("PCANBasic transport is not open");
  auto raw = Ag95Protocol::to_can(frame);
  TPCANMsg msg{};
  msg.ID = raw.can_id & 0x7FF;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 8;
  for (int i = 0; i < 8; ++i) msg.DATA[i] = raw.data[i];
  auto status = p_CAN_Write(channel_, &msg);
  if (status != PCAN_ERROR_OK) throw TransportError("CAN_Write failed, PCAN status=0x" + std::to_string(status));
#else
  (void)frame;
  throw TransportError("PCANBasic C++ transport is only implemented on Windows");
#endif
}

Ag95Frame PcanBasicTransport::receive(std::chrono::milliseconds timeout) {
#ifdef _WIN32
  if (!open_) throw TransportError("PCANBasic transport is not open");
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    TPCANMsg msg{};
    auto status = p_CAN_Read(channel_, &msg, nullptr);
    if (status == PCAN_ERROR_OK) {
      if (msg.LEN != 8) throw ProtocolError("received PCAN frame with DLC != 8");
      std::array<uint8_t, 8> data{};
      for (int i = 0; i < 8; ++i) data[i] = msg.DATA[i];
      return Ag95Protocol::from_can(msg.ID & 0x7FF, data);
    }
    if (status != PCAN_ERROR_QRCVEMPTY) throw TransportError("CAN_Read failed, PCAN status=0x" + std::to_string(status));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw TimeoutError("timeout waiting for PCANBasic CAN frame");
#else
  (void)timeout;
  throw TransportError("PCANBasic C++ transport is only implemented on Windows");
#endif
}

}  // namespace dh_ag95
