#include <cmath>

// ROS2 libraries
#include "joy_driver/joy_driver.hpp"

JoyDriverNode::JoyDriverNode() : Node("joy_driver_node") {
    declare_parameters();
    get_parameters();

    // Subscriber
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyDriverNode::joy_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "JoyDriverNode has started.");
}

void JoyDriverNode::declare_parameters() {
    this->declare_parameter<int>("linear_axis_x", 1);
    this->declare_parameter<int>("linear_axis_y", 0);
    this->declare_parameter<int>("rotate_axis_right", 3);
    this->declare_parameter<int>("rotate_axis_left", 2);
    this->declare_parameter<float>("deadzone_linear", 0.05);
    this->declare_parameter<float>("deadzone_angular", 0.03);
    this->declare_parameter<int>("max_rpm", 330);
}

void JoyDriverNode::get_parameters() {
    this->get_parameter("linear_axis_x", linear_axis_x_);
    this->get_parameter("linear_axis_y", linear_axis_y_);
    this->get_parameter("rotate_axis_right", rotate_axis_right_);
    this->get_parameter("rotate_axis_left", rotate_axis_left_);
    this->get_parameter("deadzone_linear", deadzone_linear_);
    this->get_parameter("deadzone_angular", deadzone_angular_);
    this->get_parameter("max_rpm", max_rpm_);
}

void JoyDriverNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    // ジョイスティックの軸から値を取得
    // x, yの軸を確認する必要あり
    double linear_x = msg->axes[linear_axis_x_];
    double linear_y = msg->axes[linear_axis_y_];
    double rotate_right = msg->axes[rotate_axis_right_];
    double rotate_left = msg->axes[rotate_axis_left_];

    // Deadzoneの適用
    if (std::abs(linear_x) < deadzone_linear_) linear_x = 0.0;
    if (std::abs(linear_y) < deadzone_linear_) linear_y = 0.0;

    double angular_z = set_angular_z(rotate_right, rotate_left);

    // cmd_velメッセージの作成
    auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();
    cmd_vel_msg->linear.x = linear_x * max_rpm_;
    cmd_vel_msg->linear.y = linear_y * max_rpm_;
    cmd_vel_msg->angular.z = angular_z;

    // cmd_velをパブリッシュ
    cmd_vel_publisher_->publish(*cmd_vel_msg);
}


double JoyDriverNode::set_angular_z(double rotate_right, double rotate_left) {
    double angular_z = 0.0;
   
    if(rotate_right < rotate_left){
        // 左回転の方が大きい場合は左回転を優先
        angular_z = -(rotate_left + 1.0) * 0.5;
    } else if(rotate_right > rotate_left){
        // 右回転の方が大きい場合は右回転を優先
        angular_z = (rotate_right + 1.0) * 0.5;
    } else {
        angular_z = 0.0;
    }

    if(std::abs(angular_z) < deadzone_angular_) angular_z = 0.0;
    
    return angular_z;
}












