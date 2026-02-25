#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class KeyboardTeleopNode : public rclcpp::Node 
{
    public:
        KeyboardTeleopNode();

        void run_node();
        // 非ブロッキングで1文字読み取る。未入力時は-1を返す。
        int getchar_nonblock(int timeout_ms = 100);

    private:
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};
