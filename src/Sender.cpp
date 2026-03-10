#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };

  data.x++;
  data.y++;
  data.z++;

  Utils::Message::serialize(frame, data);

if (this->send(frame))
  {
    RCLCPP_INFO(logger, "Sending: x=%.2f, y=%.2f, z=%.2f", data.x, data.y, data.z);
  }
}
