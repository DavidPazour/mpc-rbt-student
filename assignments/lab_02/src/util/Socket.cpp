#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

// clang-format off
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <mpc-rbt-solution/util/Socket.hpp>
// clang-format on

namespace Socket
{
UDP::~UDP()
{
  if (fileDescriptor != -1) {
    close(fileDescriptor);
    fileDescriptor = -1;
  }
}

void UDP::create()
{
  if ((fileDescriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    throw std::runtime_error{
      "Failed to create a new socket with port: " + std::to_string(port) +
      ", error: " + strerror(errno) + "(" + std::to_string(errno) + ")."};
  }
}

void UDP::configure() const
{
  const int optval = 1;
  if (
    setsockopt(
      fileDescriptor, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const void *>(&optval),
      sizeof(optval)) != 0) {
    throw std::runtime_error{
      "Failed to set options for the new socket with port: " + std::to_string(port) +
      ", error: " + strerror(errno) + "(" + std::to_string(errno) + ")."};
  }
}

void UDP::bind() const
{
  sockaddr_in addr{};
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;
  if (::bind(fileDescriptor, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
    throw std::runtime_error{
      "Failed to bind the new socket with port: " + std::to_string(port) +
      ", error: " + strerror(errno) + "(" + std::to_string(errno) + ")."};
  }
}

bool UDP::send(const IPFrame & frame) const
{
  sockaddr_in addr{};
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(frame.port);
  if (!inet_aton(frame.address.c_str(), &addr.sin_addr)) {
    RCLCPP_WARN(logger, "Failed to send frame, invalid address: '%s'", frame.address.c_str());
    return false;
  }

  const ssize_t sentBytes = sendto(
    fileDescriptor, frame.serializedData.data(), frame.dataSize, 0,
    reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));

  return sentBytes >= 0 && (sentBytes == frame.dataSize);
}

bool UDP::receive(IPFrame & frame) const
{
  memset(frame.serializedData.data(), 0, frame.serializedData.size());
  sockaddr_in addr{};
  memset(&addr, 0, sizeof(addr));
  socklen_t addrSize = sizeof(addr);

  const ssize_t receivedBytes = recvfrom(
    fileDescriptor, frame.serializedData.data(), frame.serializedData.size(), 0,
    reinterpret_cast<sockaddr *>(&addr), &addrSize);

  frame.port = ntohs(addr.sin_port);
  frame.address = inet_ntoa(addr.sin_addr);
  frame.dataSize = receivedBytes;

  return receivedBytes >= 0;
}
}  // namespace Socket
