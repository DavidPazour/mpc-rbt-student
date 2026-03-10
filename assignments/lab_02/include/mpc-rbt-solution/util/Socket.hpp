#ifndef COMMUNICATION_EXAMPLE_SOCKET_H
#define COMMUNICATION_EXAMPLE_SOCKET_H

// clang-format off
#include <array>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
// clang-format on

namespace Socket
{
static constexpr uint16_t MAX_FRAME_MTU = 1500;

struct IPFrame
{
  uint16_t port = 0;
  std::string address{};
  std::array<uint8_t, MAX_FRAME_MTU> serializedData{};
  ssize_t dataSize = 0;
};

class UDP
{
public:
  explicit UDP(const uint16_t localPort) : port(localPort) {}
  virtual ~UDP();

  void create();
  void configure() const;
  void bind() const;

  [[nodiscard]] bool send(const IPFrame & frame) const;
  [[nodiscard]] bool receive(IPFrame & frame) const;

private:
  uint16_t port;
  int fileDescriptor = -1;
  rclcpp::Logger logger{rclcpp::get_logger("socket")};
};
}  // namespace Socket

#endif  // COMMUNICATION_EXAMPLE_SOCKET_H
