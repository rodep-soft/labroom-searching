#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>

#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SimpleMotor : public rclcpp::Node {
    public:
        SimpleMotor();
        ~SimpleMotor();

    private:

        constexpr uint8_t MOTOR_ID_ = 0x01;
        constexpr uint8_t CMD_VELOCITY_ = 0x02;
        constexpr uint8_t GET_STATUS_ = 0x74;

        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_;

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void declare_parameters();
        void get_parameters();

        uint16_t calculate_crc16(const std::vector<uint8_t>& data);
        std::vector<uint8_t> create_velocity_command(double target_vel);

        int32_t decode_velocity_feedback(const std::vector<uint8_t>& response);
        void set_protocol_mode();

        bool open_serial_port(const std::string& port_name, unsigned int baud_rate);
        void setup_serial_port(const std::string& port_name, unsigned int baud_rate);

        void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void request_velocity_feedback();       

}