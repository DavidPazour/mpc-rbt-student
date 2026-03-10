#include <mpc-rbt-solution/Sender.hpp>

int main(int argc, char ** argv)
{
  Utils::configureLogging();
  const auto config = Utils::Config::fromArgs(argc, argv);
  if (!config.has_value()) throw std::runtime_error{"Failed to load config file."};

  RCLCPP_DEBUG(rclcpp::get_logger("example"), "example debug message");
  RCLCPP_INFO(rclcpp::get_logger("example"), "example info message");
  RCLCPP_WARN(rclcpp::get_logger("example"), "example warning message");
  RCLCPP_ERROR(rclcpp::get_logger("example"), "example error message");
  RCLCPP_FATAL(rclcpp::get_logger("example"), "example fatal error message");

  Sender::Node(config->sender).run();

  return 0;
}
