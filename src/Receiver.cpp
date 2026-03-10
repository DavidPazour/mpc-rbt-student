#include <mpc-rbt-solution/Receiver.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};
    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  if (Utils::Message::deserialize(frame, data)) {  // Bod 3.a
    RCLCPP_INFO(logger, "Přijato: x=%.2f, y=%.2f, z=%.2f", data.x, data.y, data.z);
  }
  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
}
