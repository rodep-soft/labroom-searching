#pragma once

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MecanumController : public rclcpp::Node {
    public:
        MecanumController();

    private:
        // Protocol constants
        static constexpr uint8_t CMD_VELOCITY_ = 0x02;
        static constexpr uint8_t CMD_GET_STATUS_ = 0x74;

        // Motor IDs for mecanum wheels
        // Front-left, front-right, rear-left, rear-right
        static constexpr uint8_t MOTOR_FL_ID_ = 0x01;
        static constexpr uint8_t MOTOR_FR_ID_ = 0x02;
        static constexpr uint8_t MOTOR_RL_ID_ = 0x03;
        static constexpr uint8_t MOTOR_RR_ID_ = 0x04;

        // IO
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_{io_context_};

        // ROS interfaces
        // Subscription for cmd_vel (Twist)
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        
        // Subscriptions for individual motor velocities (optional)
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_fl_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_fr_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_rl_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_rr_subscription_;

        // Publishers for velocity feedback
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_fl_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_fr_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_rl_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_rr_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        // Parameters
        std::string port_name_;
        int baud_rate_;
        int max_rpm_;
        double wheel_base_;  // Distance between front and rear wheels (Lx)
        double track_width_; // Distance between left and right wheels (Ly)

        // Current velocity targets for each motor
        double target_vel_fl_ = 0.0;
        double target_vel_fr_ = 0.0;
        double target_vel_rl_ = 0.0;
        double target_vel_rr_ = 0.0;

        // Param helpers
        void declare_parameters();
        void get_parameters();

        // Serial helpers
        bool setup_serial_port(const std::string &port_name, unsigned int baud_rate);

        // Protocol helpers
        uint8_t calc_crc8(const std::vector<uint8_t> &data);
        std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double target_velocity);
        int32_t decode_velocity_feedback(const std::vector<uint8_t> &response);
        void send_velocity_commands();
        void request_and_receive_feedback();

        // Callbacks
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void velocity_fl_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void velocity_fr_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void velocity_rl_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void velocity_rr_callback(const std_msgs::msg::Float64::SharedPtr msg);
        
        // Mecanum kinematics
        void compute_wheel_velocities(double linear_x, double linear_y, double angular_z);
};
