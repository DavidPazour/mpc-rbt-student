#ifndef COMMUNICATION_EXAMPLE_UTILS_H
#define COMMUNICATION_EXAMPLE_UTILS_H

// clang-format off
#include <cstdint>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
// clang-format on

#define UNIMPLEMENTED(fn)                                                                       \
  (throw std::runtime_error{                                                                    \
    "[" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "] [" + \
    fn + "]: UNIMPLEMENTED !"})
#define UNFINISHED(fn) RCLCPP_FATAL(rclcpp::get_logger(fn), "UNFINISHED !")

namespace Utils
{
struct Config
{
  struct Sender
  {
    uint16_t localPort = 0;
    uint16_t remotePort = 0;
    std::string remoteAddress{};
    uint16_t sendingPeriodMillis = 0;
  };
  struct Receiver
  {
    uint16_t localPort = 0;
  };

  Sender sender{};
  Receiver receiver{};

  [[nodiscard]] static std::optional<Config> fromArgs(int argc, char ** argv);
};

void from_json(const nlohmann::json & j, Config::Sender & s);
void from_json(const nlohmann::json & j, Config::Receiver & r);
void from_json(const nlohmann::json & j, Config & c);

struct Message
{
  uint64_t timestamp = 0;
  std::string frame = "";
  double x = 0;
  double y = 0;
  double z = 0;

  static void to_json(nlohmann::json & j, const Message & m);
  static void from_json(const nlohmann::json & j, Message & m);
  static bool serialize(Socket::IPFrame & f, const Message & m);
  static bool deserialize(const Socket::IPFrame & f, Message & m);
};

static inline void configureLogging()
{
  const auto envLogLevel = std::getenv("LOG_LEVEL");
  if (
    rcutils_logging_set_logger_level("", !envLogLevel ? 51 : std::stoi(envLogLevel) * 10) !=
    RCUTILS_RET_OK) {
    throw std::runtime_error{
      "Failed to set logging level, error: " + std::string(rcutils_get_error_string().str)};
  }
}
}  // namespace Utils

#endif  // COMMUNICATION_EXAMPLE_UTILS_H
