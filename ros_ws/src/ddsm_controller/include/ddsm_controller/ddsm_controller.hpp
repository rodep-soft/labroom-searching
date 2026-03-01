#pragma once

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DDSMController : public rclcpp::Node {
    public:
        DDSMController();

    private:
        static constexpr uint8_t CMD_SPEED_LOOP_ = 0x64;

        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_{io_context_};

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

        std::string port_name_;
        int baud_rate_;
        int motor_max_rpm_;
        int user_max_rpm_;
        double stop_eps_;
        double wheel_radius_;
        double wheel_base_x_;
        double wheel_base_y_;

        void declare_parameters();
        void get_parameters();

        bool setup_serial_port(const std::string &port_name, unsigned int baud_rate);

        uint8_t calc_crc8(const std::vector<uint8_t> &data);
        std::vector<uint8_t> create_velocity_command(uint8_t motor_id, int target_rpm, bool is_braking) const;
        std::array<int, 4> calculate_target_rpms_from_twist(const geometry_msgs::msg::Twist& twist) const;
        bool is_brake_command(const geometry_msgs::msg::Twist& twist) const;

        void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
