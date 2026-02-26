#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <vector>

#include <booost/asio.hpp>
#include <boost/asio/serial_port.hpp>

// ヘッダー
#include "mecanum_controller/mecanum_controller.hpp"

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

MecanumController::MecanumController() : rclcpp::Node("mecanum_controller_node") {

    public:
        MecanumController();
        ~MecanumController();

    private:
        void declare_parameters();
        void get_parameters();

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void send_velocity_commands();

        uint8_t calc_crc8(const std::vector<uint8_t> &data);
        std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double target_velocity);
        int32_t decode_velocity_feedback(const std::vector<uint8_t> &response);

        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_{io_context_};

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        rclcpp::TimerBase::SharedPtr timer_; 
        
        // Parameters
        std::string port_name_;
        int baud_rate_;
        int max_rpm_;
        double wheel_base_x_;  // 機体の縦幅 (Lx)
        double wheel_base_y_; // 機体の横幅 (Ly)
        std::vector<int64_t> motor_ids_int64_; // paramで受け取るため一時的なmotor_id配列
        std::vector<uint8_t> motor_ids_; // 使用するmotor_id配列
    
}
