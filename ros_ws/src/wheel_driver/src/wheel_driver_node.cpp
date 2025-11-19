#include <crclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// header file
#include "wheel_driver/wheel_driver_node.hpp"

WheelDriverNode::WheelDriverNode(): Node("wheel_driver_node")
{
    declare_parameters();
    get_parameters();
    if(moo)
}


int main(int argc, char **argv){
    rclcpp::inti(argc, argv);
    rclcpp::spin(std::make_shared<WheelDriverNode>);
    rclcpp::shutdown();
    return 0;
}