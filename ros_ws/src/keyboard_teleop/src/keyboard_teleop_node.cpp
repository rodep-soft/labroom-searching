#include "keyboard_teleop/keyboard_teleop_node.hpp"
#include <sys/select.h>
#include <sys/time.h>

KeyboardTeleopNode::KeyboardTeleopNode()
    : rclcpp::Node("keyboard_teleop_node"), input_(io_context_, STDIN_FILENO) {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/cmd_vel", 10);

    tcgetatter(STDIN_FILENO, &old_terminal_settings_);
    struct termios new_terminal_settings = old_terminal_settings_;
    new_terminal_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal_settings);

    // 非同期読み取り開始
    start_keyboard_read();

    // boostのイベントループを別スレッドで回す
    std::thread([this]() { io_context_.run(); }).detach();

    // RCLCPP_INFO出力
    RCLCPP_INFO(this->get_logger(), "Keyboard Teleop (C++) Started");
    RCLCPP_INFO(this->get_logger(), "---------------------------");
    RCLCPP_INFO(this->get_logger(), "Forward[m/s]:  a (1.0), s (1.5), d (2.0)");
    RCLCPP_INFO(this->get_logger(), "Backward[m/s]: f (-1.0), g (-1.5), h (-2.0)");
    RCLCPP_INFO(this->get_logger(), "Stop:     Space or other keys");
    RCLCPP_INFO(this->get_logger(), "Quit:     CTRL-C");
    RCLCPP_INFO(this->get_logger(), "---------------------------");
}

KeyboardTeleopNode::~KeyboardTeleopNode() {
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
        std_msgs::msg::Float64 msg;

        switch(input_buffer_) {
            case 'a': msg.data = 1.0; break;
            case 's': msg.data = 1.5; break;
            case 'd': msg.data = 2.0; break;
            case 'f': msg.data = -1.0; break;
            case 'g': msg.data = -1.5; break;
            case 'h': msg.data = -2.0; break;
            case ' ':
            default: msg.data = 0.0; break;
        }

        RCLCPP_INFO(this->get_logger(), "Published cmd_vel: %.2f m/s", msg.data);

        publisher_->publish(msg);

        // 再度非同期読み取りを開始
        start_keyboard_read();
    }
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
    rclcpp::shutdown();
    return 0;
}
