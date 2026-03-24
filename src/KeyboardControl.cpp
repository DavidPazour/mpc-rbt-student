#include <chrono>
#include <functional>

#include "KeyboardControl.hpp"
#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode() : rclcpp::Node("keyboard_control_node") {

    // Vytvoření publisheru na topic /cmd_vel
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Vytvoření timeru - volá timerCallback každých 100ms
    timer_ = this->create_wall_timer(
        100ms, std::bind(&KeyboardControlNode::timerCallback, this));

    // Set terminal settings to non-blocking
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");
}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    geometry_msgs::msg::Twist twist{};
    char c;
    bool received_any_input = false;

    // Vyprázdnění bufferu klávesnice
    while (read(STDIN_FILENO, &c, 1) > 0) {
        received_any_input = true;
        if (c == '\033') { // ESC sekvence pro šipky
            char seq[2];
            if (read(STDIN_FILENO, &seq, 2) == 2) {
                if (seq[0] == '[') {
                    // Resetujeme twist pro nejnovější příkaz v bufferu
                    twist = geometry_msgs::msg::Twist(); 
                    switch (seq[1]) {
                        case 'A': twist.linear.x = 0.5;  break;  // nahoru
                        case 'B': twist.linear.x = -0.5; break;  // dolů
                        case 'C': twist.angular.z = -1.0; break; // doprava
                        case 'D': twist.angular.z = 1.0;  break; // doleva
                    }
                }
            }
        }
    }

    // Publikujeme zprávu (pokud nebyl stisk, pošlou se nuly = robot zastaví)
    twist_publisher_->publish(twist);

    // Volitelné: Použití proměnné pro debug, aby zmizelo varování
    if (received_any_input) {
        RCLCPP_DEBUG(this->get_logger(), "Input processed.");
    }
}
