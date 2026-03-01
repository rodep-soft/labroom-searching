#include "keyboard_teleop/keyboard_teleop_node.hpp"

#include <functional>
#include <iostream>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

KeyboardTeleopNode::KeyboardTeleopNode()
    : rclcpp::Node("keyboard_teleop_node"), input_(io_context_, STDIN_FILENO) {

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);


    tcgetattr(STDIN_FILENO, &old_terminal_settings_);
    struct termios new_terminal_settings = old_terminal_settings_;
    new_terminal_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal_settings);

    // 非同期読み取り開始
    start_keyboard_read();

    // boostのイベントループを別スレッドで回す
    io_thread_ = std::thread([this]() { io_context_.run(); });

    // RCLCPP_INFO出力
    RCLCPP_INFO(this->get_logger(), "Keyboard Teleop (C++) Started");
    RCLCPP_INFO(this->get_logger(), "---------------------------");
    RCLCPP_INFO(this->get_logger(), "h: right");
    RCLCPP_INFO(this->get_logger(), "j: forward");
    RCLCPP_INFO(this->get_logger(), "k: backward");
    RCLCPP_INFO(this->get_logger(), "l: left");
    RCLCPP_INFO(this->get_logger(), "a: rotate left");
    RCLCPP_INFO(this->get_logger(), "s: rotate right");
    RCLCPP_INFO(this->get_logger(), "Quit:     CTRL-C");
    RCLCPP_INFO(this->get_logger(), "---------------------------");
}

KeyboardTeleopNode::~KeyboardTeleopNode() {
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }

    // 終了時に端末設定を元に戻す
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_settings_);
}

void KeyboardTeleopNode::start_keyboard_read() {
    boost::asio::async_read(input_, boost::asio::buffer(&input_buffer_, 1),
        std::bind(&KeyboardTeleopNode::handle_read_callback, this, 
            std::placeholders::_1, std::placeholders::_2));
}

void KeyboardTeleopNode::handle_read_callback(const boost::system::error_code& error, std::size_t length) {
    if(!error && length > 0) {
        geometry_msgs::msg::Twist msg;
        bool should_publish = true;

        switch(input_buffer_) {
            case 'h':
                msg.linear.y = -linear_speed_;
                break;
            case 'j':
                msg.linear.x = linear_speed_;
                break;
            case 'k':
                msg.linear.x = -linear_speed_;
                break;
            case 'l':
                msg.linear.y = linear_speed_;
                break;
            case 'a':
                msg.angular.z = angular_speed_;
                break;
            case 's':
                msg.angular.z = -angular_speed_;
                break;
            default: should_publish = false; break;
        }

        if (should_publish) {
            RCLCPP_INFO(
                this->get_logger(),
                "Published Twist vx=%.2f vy=%.2f wz=%.2f",
                msg.linear.x,
                msg.linear.y,
                msg.angular.z);
            twist_publisher_->publish(msg);
        }

        // 再度非同期読み取りを開始
        start_keyboard_read();
    } else if (error != boost::asio::error::operation_aborted) {
        start_keyboard_read();
    }
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
    rclcpp::shutdown();
    return 0;
}
