#include <cmath>
#include <functional>

#include "joy_driver/joy_diver_node.hpp"

JoyDriverNode::JoyDriverNode() : Node("joy_driver_node") {
    declare_parameters();
    get_parameters();


    // Subscriber
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyDriverNode::joy_callback, this, std::placeholders::_1));

    // Publishers
    driver_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "/driver_rpm", 10);

    RCLCPP_INFO(this->get_logger(), "JoyDriverNode has started.");
}

void JoyDriverNode::declare_parameters() {
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<float>("deadzone_linear", 0.05);
    this->declare_parameter<int>("max_rpm", 330);
}

void JoyDriverNode::get_parameters() {
    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("deadzone_linear", deadzone_linear_);
    this->get_parameter("max_rpm", max_rpm_);
}

void JoyDriverNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (axis_linear_ < 0 || axis_linear_ >= static_cast<int>(msg->axes.size())) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "axis_linear (%d) is out of range for axes size %zu", axis_linear_, msg->axes.size());
        return;
    }

    double axis_value = msg->axes[axis_linear_];
    if (std::abs(axis_value) < deadzone_linear_) {
        axis_value = 0.0;
    }

    auto driver_msg_ = std_msgs::msg::Float64();
    driver_msg_.data = axis_value * max_rpm_;

    driver_vel_publisher_->publish(driver_msg_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
