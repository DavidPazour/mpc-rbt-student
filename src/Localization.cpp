#include "Localization.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Inicializace odometrické zprávy
    odometry_.header.frame_id = "odom";
    odometry_.child_frame_id = "base_link";
    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.orientation.w = 1.0;

    // Subscriber - opravený název metody na joint_callback
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&LocalizationNode::joint_callback, this, std::placeholders::_1));

    // Publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->velocity.size() < 2) return;
    
    // MÍSTO this->get_clock()->now() použij čas ze zprávy!
    auto current_time = rclcpp::Time(msg->header.stamp);
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0) return;

    update_odometry(msg->velocity[1], msg->velocity[0], dt);
    
    // Předávej tento čas dál do publikace
    publish_odometry(msg->header.stamp);
    publish_transform(msg->header.stamp);
}

void LocalizationNode::update_odometry(double left_vel, double right_vel, double dt) {
    // 1. Ochrana proti NaN z enkodérů
    if (std::isnan(left_vel) || std::isnan(right_vel)) return;

    // 2. Výpočet rychlostí
    double v = (right_vel + left_vel) / 2.0 * robot_config::WHEEL_RADIUS;
    
    // Ochrana proti dělení nulou u rozchodu kol
    double wheel_dist = 2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS;
    double w = (wheel_dist > 0.0001) ? ((right_vel - left_vel) / wheel_dist * robot_config::WHEEL_RADIUS) : 0.0;

    // 3. Získání aktuální orientace
    tf2::Quaternion q_old;
    try {
        tf2::fromMsg(odometry_.pose.pose.orientation, q_old);
        // Pokud je kvaternion neplatný (všechno nuly), zresetuj ho
        if (q_old.length2() < 0.1) q_old.setRPY(0, 0, 0);
    } catch (...) {
        q_old.setRPY(0, 0, 0);
    }

    double roll, pitch, theta_old;
    tf2::Matrix3x3(q_old).getRPY(roll, pitch, theta_old);

    // 4. Integrace
    double delta_theta = w * dt;
    double theta_new = theta_old + delta_theta;
    double avg_theta = theta_old + (delta_theta / 2.0);

    odometry_.pose.pose.position.x += v * std::cos(avg_theta) * dt;
    odometry_.pose.pose.position.y += v * std::sin(avg_theta) * dt;

    // 5. Uložení - VŽDY normalizuj kvaternion
    tf2::Quaternion q_new;
    q_new.setRPY(0, 0, theta_new);
    q_new.normalize(); // Pojistka proti TF_DENORMALIZED_QUATERNION
    odometry_.pose.pose.orientation = tf2::toMsg(q_new);
}

void LocalizationNode::publish_odometry(const rclcpp::Time & stamp) {
    odometry_.header.stamp = stamp; // Použij čas z enkodérů
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publish_transform(const rclcpp::Time & stamp) {
    // 1. Transformace ODOM -> BASE_LINK
    geometry_msgs::msg::TransformStamped t_odom;
    t_odom.header.stamp = stamp; // Používáme parametr stamp!
    t_odom.header.frame_id = "odom";
    t_odom.child_frame_id = "base_link";
    t_odom.transform.translation.x = odometry_.pose.pose.position.x;
    t_odom.transform.translation.y = odometry_.pose.pose.position.y;
    t_odom.transform.translation.z = 0.0;
    t_odom.transform.rotation = odometry_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t_odom);

    // 2. Transformace MAP -> ODOM
    geometry_msgs::msg::TransformStamped t_map;
    t_map.header.stamp = stamp; // Používáme parametr stamp!
    t_map.header.frame_id = "map";
    t_map.child_frame_id = "odom";
    t_map.transform.translation.x = 0.0;
    t_map.transform.translation.y = 0.0;
    t_map.transform.translation.z = 0.0;
    t_map.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t_map);
}
