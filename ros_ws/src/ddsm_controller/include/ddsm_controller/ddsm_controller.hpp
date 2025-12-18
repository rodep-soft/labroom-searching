#pragma once

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class DDSMController : public rclcpp::Node {
    public:
        DDSMController();

    private:
        // Protocol constants
        static constexpr uint8_t MOTOR_ID_ = 0x01;
        static constexpr uint8_t CMD_VELOCITY_ = 0x02;
        static constexpr uint8_t CMD_GET_STATUS_ = 0x74;

        // IO
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_{io_context_};

        // ROS interfaces
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Parameters
        std::string serial_port_name_;
        int baud_rate_{};
        int motor_id_{};

        // Param helpers
        void declare_parameters();
        void get_parameters();

        // Serial helpers
        bool setup_serial_port(const std::string &port_name, unsigned int baud_rate);

        // Protocol helpers
        uint8_t calc_crc8(const std::vector<uint8_t> &data);
        std::vector<uint8_t> create_velocity_command(double target_velocity);
        int32_t decode_velocity_feedback(const std::vector<uint8_t> &response);
        void request_and_receive_feedback();

        // Callbacks
        void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);
};