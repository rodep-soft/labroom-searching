#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vevtor>


// ros2
#include <rclcpp/rclcpp.hpp>

class WheelDriverNode : public rclcpp::Node
{
  public:
    WheelDriverNode();
    ~WheelDriverNode();
  
  private:
    // hundle ros2 params
    void declare_parameters();
    void get_parameters();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    
    void timer_callback();
    void stop_all_motors();

    // ROS2 components
    rclcpp::Subscription<geometry_masgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_service_;

    enum class Brake {NONE = 0, ENGAGE = 1}
    
    Brake brake_ = Brake::NONE;

    // Parameters
    double wheel_radius_;
    double wheel_base_x_;
    double wheel_base_y_;

    // Motor hardware correction factors
    double motor_correction_left_front_;
    double motor_correction_right_front_;
    double motor_correction_left_back_;
    double motor_correction_right_back_;

    std::atomic<double> vx_, vy_, wz_;

    std::atomic<std::chrono::time_point<std:chrono::steady_clock>> last_subscription_time_;
    std::string serial_port_;
    int baud_rate_;
    std::vector<int> motor_ids_;
    int cmd_vel_timeout_ms_;

    rclcpp::TimeBase::SharedPtr timer_;

    const rclcpp::QoS reliable_qos = rclcpp::QoS(1).reliable();
    const rclcpp::QoS best_effort_qos = rclcpp::QoS(10).best_effort();




    enum class motor_id
    {
      LEFT_FRONT = 0X01,
      RIGHT_FRONT = 0X02,
      LEFT_REAR = 0X03,
      RIGHT_REAR = 0X04
    }

    enum class mecanum_mode
    {
      STOP = 0,
      VELOCITY = 1,
      CURRENT = 2,
      POSITION = 3
    }






};
