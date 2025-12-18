#include "keyboard_teleop/keyboard_teleop_node.hpp"
#include <sys/select.h>
#include <sys/time.h>

KeyboardTeleopNode::KeyboardTeleopNode() : rclcpp::Node("keyboard_teleop_node") {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/cmd_vel", 10);

    std::cout << "Keyboard Teleop (C++) Started\n";
    std::cout << "---------------------------\n";
    std::cout << "Forward[m/s]:  a (1.0), s (1.5), d (2.0)\n";
    std::cout << "Backward[m/s]: f (-1.0), g (-1.5), h (-2.0)\n";
    std::cout << "Stop:     Space or other keys\n";
    std::cout << "Quit:     CTRL-C\n";
    std::cout << "---------------------------\n";
}

int KeyboardTeleopNode::getchar_nonblock(int timeout_ms) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    struct timeval tv;
    struct timeval* ptv = nullptr;
    if (timeout_ms >= 0) {
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        ptv = &tv;
    }

    int retval = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, ptv);
    if (retval > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
        unsigned char c;
        ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n == 1) return static_cast<int>(c);
    }
    return -1;
}

void KeyboardTeleopNode::run_node() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int ch;
    while (rclcpp::ok()) {
        ch = getchar_nonblock(100); // 100msでポーリング
        if (ch == -1) {
            continue;
        }
        char c = static_cast<char>(ch);
        std_msgs::msg::Float64 msg;
        switch (c) {
            case 'a':
                msg.data = 1.0;
                break;
            case 's':
                msg.data = 1.5;
                break;
            case 'd':
                msg.data = 2.0;
                break;
            case 'f':
                msg.data = -1.0;
                break;
            case 'g':
                msg.data = -1.5;
                break;
            case 'h':
                msg.data = -2.0;
                break;
            case ' ':
            default:
                msg.data = 0.0;
                break;
        }
        publisher_->publish(msg);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleopNode>();

    // Nodeを別スレッドで回し、メインスレッドでキー入力を待つ
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });
    
    node->run_node();
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}