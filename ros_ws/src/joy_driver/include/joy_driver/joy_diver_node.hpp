#pragma once

// ros2 libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

class JoyDriverNode : public rclcpp::Node {
    public:
        JoyDriverNode();

    private:
        void declare_parameters();
        void get_parameters();

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_vel_publisher_;

        int axis_linear_{1};
        int max_rpm_{330};
        double deadzone_linear_{0.05};
};