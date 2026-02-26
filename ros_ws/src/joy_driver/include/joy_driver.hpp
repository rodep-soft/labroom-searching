#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include <chrono>

// include ros2
#include <rclcpp/rclcpp.hpp>

// ros2 message
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

class JoyDriverNode : public rclcpp::Node {
    public:
        JoyDriverNode();

    private:
        void declare_parameters();
        void get_parameters();

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        double set_angular_z(double rotate_right, double rotate_left);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        // Parameters (loaded from config / ros parameters)
        int linear_axis_x_ = 1;
        int linear_axis_y_ = 0;
        int rotate_axis_right_ = 3;
        int rotate_axis_left_ = 2;

        int max_rpm_ = 330;

        double deadzone_linear_ = 0.05;
        double deadzone_angular_ = 0.03;

};
