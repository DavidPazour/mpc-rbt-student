#ifndef COMMUNICATION_EXAMPLE_RECEIVER_H
#define COMMUNICATION_EXAMPLE_RECEIVER_H

#include <mpc-rbt-solution/util/Socket.hpp>
#include <mpc-rbt-solution/util/Utils.hpp>

namespace Receiver
{
class Node : public Socket::UDP
{
public:
  explicit Node(const Utils::Config::Receiver & receiverConfig)
  : Socket::UDP(receiverConfig.localPort), config(receiverConfig)
  {
    this->create();
    this->configure();
    this->bind();
    callback = [this](const Socket::IPFrame & frame) { onDataReceived(frame); };
  }

  void run();

private:
  void onDataReceived(const Socket::IPFrame & frame);

  Utils::Message data{};

  Utils::Config::Receiver config;
  std::function<void(const Socket::IPFrame &)> callback;
  rclcpp::Logger logger{rclcpp::get_logger("receiver")};

  friend class TestMock;
};
}  // namespace Receiver

namespace Receiver
{
class TestMock : public Node
{
public:
  TestMock(const Utils::Config::Receiver & receiverConfig) : Receiver::Node(receiverConfig) {}

  [[nodiscard]] const Utils::Message & getData() const { return data; }
};
}  // namespace Receiver

#endif  // COMMUNICATION_EXAMPLE_RECEIVER_H
