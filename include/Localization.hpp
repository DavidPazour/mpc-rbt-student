#ifndef LOCALIZATION_HPP_
#define LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode();

private:
    /**
     * @brief Callback pro příjem dat z enkodérů (rychlosti kol)
     */
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Výpočet přímé kinematiky a integrace pozice
     * @param left_vel Rychlost levého kola [rad/s]
     * @param right_vel Rychlost pravého kola [rad/s]
     * @param dt Časový krok [s]
     */
    void update_odometry(double left_vel, double right_vel, double dt);

    /**
     * @brief Publikuje zprávu typu nav_msgs/Odometry se zadaným časovým razítkem
 */
void publish_odometry(const rclcpp::Time & stamp);

/**
 * @brief Publikuje transformace map -> odom a odom -> base_link se zadaným časovým razítkem
 */
void publish_transform(const rclcpp::Time & stamp);

    // ROS 2 komunikace
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Interní stav
    nav_msgs::msg::Odometry odometry_;
    rclcpp::Time last_time_;
};

#endif // LOCALIZATION_HPP_
