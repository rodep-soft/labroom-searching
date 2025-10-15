#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyDisplayNode : public rclcpp::Node
{
public:
  JoyDisplayNode() : Node("joy_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyDisplayNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Axes: ");
    for (size_t i = 0; i < msg->axes.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  [%zu]: %f", i, msg->axes[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Buttons: ");
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  [%zu]: %d", i, msg->buttons[i]);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDisplayNode>());
  rclcpp::shutdown();
  return 0;
}