#pragma once

#include <cstddef>
#include <termios.h>
#include <thread>

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

class KeyboardTeleopNode : public rclcpp::Node 
{
    public:
        KeyboardTeleopNode();
        ~KeyboardTeleopNode();

    private:
        void start_keyboard_read();
        void handle_read_callback(const boost::system::error_code& error, std::size_t length);
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

        boost::asio::io_context io_context_;
        boost::asio::posix::stream_descriptor input_;
        std::thread io_thread_;

        struct termios old_terminal_settings_ {};
        char input_buffer_ = 0;

        const double linear_speed_ = 1.0;
        const double angular_speed_ = 0.4;
};
