// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("MojeNodePub"), count_(0)
  {
  this->declare_parameter("voltage_max", 42.0);
  this->declare_parameter("voltage_min", 32.0);

    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);

    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10,
      std::bind(&MinimalPublisher::battery_callback, this, std::placeholders::_1));

  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "MojeNodePub";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float v_max = this->get_parameter("voltage_max").as_double();
    float v_min = this->get_parameter("voltage_min").as_double();

    float percentage = (msg->data - v_min) / (v_max - v_min) * 100.0;

    auto out = std_msgs::msg::Float32();
    out.data = percentage;
    RCLCPP_INFO(this->get_logger(), "Battery: %.2fV = %.2f%%", msg->data, percentage);
    battery_pub_->publish(out);
  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
