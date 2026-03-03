// clang-format off
#include <fstream>

#include <mpc-rbt-solution/util/Socket.hpp>
#include <mpc-rbt-solution/util/Utils.hpp>
// clang-format on

namespace Utils
{
std::optional<Config> Config::fromArgs(int argc, char ** argv)
{
  rclcpp::Logger logger{rclcpp::get_logger("config")};
  if (argc < 2) {
    RCLCPP_ERROR(logger, "Config file not provided.");
    return std::nullopt;
  }
  if (!argv) {
    RCLCPP_ERROR(logger, "Invalid arguments.");
    return std::nullopt;
  }

  std::ifstream configFile(argv[1]);
  nlohmann::json configJson;
  try {
    configJson = nlohmann::json::parse(configFile);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to parse the config file: '%s' (error: %s).", argv[1], e.what());
    return std::nullopt;
  }

  Config config{};
  from_json(configJson, config);
  return config;
}

void from_json(const nlohmann::json & j, Config::Sender & c)
{
  j.at("localPort").get_to(c.localPort);
  j.at("remotePort").get_to(c.remotePort);
  j.at("remoteAddress").get_to(c.remoteAddress);
  j.at("sendingPeriodMillis").get_to(c.sendingPeriodMillis);
}

void from_json(const nlohmann::json & j, Config::Receiver & c)
{
  j.at("localPort").get_to(c.localPort);
}

void from_json(const nlohmann::json & j, Config & c)
{
  j.at("sender").get_to(c.sender);
  j.at("receiver").get_to(c.receiver);
}

void Message::to_json(nlohmann::json & j, const Message & m)
{
  j = nlohmann::json{
    {"timestamp", m.timestamp},
    {"frame", m.frame},
    {"x", m.x},
    {"y", m.y}, 
    {"z", m.z}
  };
}

void Message::from_json(const nlohmann::json & j, Message & m)
{
  j.at("timestamp").get_to(m.timestamp);
  j.at("frame").get_to(m.frame);
  j.at("x").get_to(m.x);
  j.at("y").get_to(m.y);
  j.at("z").get_to(m.z);
}

bool Message::serialize(Socket::IPFrame & f, const Message & m)
{
  nlohmann::json dataJson;
  to_json(dataJson, m);
  std::string dataStr = to_string(dataJson);
  if (dataStr.size() > f.serializedData.size()) return false;

  memset(f.serializedData.data(), 0, f.serializedData.size());
  std::move(dataStr.begin(), dataStr.end(), f.serializedData.begin());
  f.dataSize = static_cast<ssize_t>(dataStr.size());

  return true;
}

bool Message::deserialize(const Socket::IPFrame & f, Message & m)
{
  const nlohmann::json dataJson = nlohmann::json::parse(f.serializedData, nullptr, false);
  if (dataJson.is_discarded()) return false;

  from_json(dataJson, m);

  return true;
}
}  // namespace Utils
