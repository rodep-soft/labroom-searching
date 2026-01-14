#include "mecanum_controller/mecanum_controller.hpp"
#include <chrono>
#include <thread>
#include <unistd.h>

MecanumController::MecanumController() : rclcpp::Node("mecanum_controller_node") {
    declare_parameters();
    get_parameters();

    // Create subscription for cmd_vel (main control input)
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MecanumController::cmd_vel_callback, this, std::placeholders::_1));

    // Create subscriptions for each motor (optional, for manual control)
    vel_fl_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mecanum/vel_fl", 10, std::bind(&MecanumController::velocity_fl_callback, this, std::placeholders::_1));
    
    vel_fr_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mecanum/vel_fr", 10, std::bind(&MecanumController::velocity_fr_callback, this, std::placeholders::_1));
    
    vel_rl_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mecanum/vel_rl", 10, std::bind(&MecanumController::velocity_rl_callback, this, std::placeholders::_1));
    
    vel_rr_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mecanum/vel_rr", 10, std::bind(&MecanumController::velocity_rr_callback, this, std::placeholders::_1));

    // Create publishers for feedback
    vel_fl_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/mecanum/vel_fl_feedback", 10);
    vel_fr_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/mecanum/vel_fr_feedback", 10);
    vel_rl_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/mecanum/vel_rl_feedback", 10);
    vel_rr_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/mecanum/vel_rr_feedback", 10);

    setup_serial_port(port_name_, static_cast<unsigned int>(baud_rate_));

    // Timer for sending periodic velocity commands and receiving feedback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MecanumController::send_velocity_commands, this));
}

void MecanumController::declare_parameters() {
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("max_rpm", 200);
    this->declare_parameter<double>("wheel_base", 0.3);   // 前後輪間距離 (m)
    this->declare_parameter<double>("track_width", 0.3); // 左右輪間距離 (m)
}

void MecanumController::get_parameters() {
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    max_rpm_ = this->get_parameter("max_rpm").as_int();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    track_width_ = this->get_parameter("track_width").as_double();
}

bool MecanumController::setup_serial_port(const std::string& port_name, unsigned int baud_rate) {
    try {
        serial_port_.open(port_name);
        if(!serial_port_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port : %s", port_name.c_str());
            return false;
        }

        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none)); 
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));
    
        RCLCPP_INFO(this->get_logger(), "Serial port %s initialized successfully.", port_name.c_str());
        return true;
    } catch(const boost::system::system_error& e){
        RCLCPP_ERROR(this->get_logger(), "Serial port setup failed : %s", e.what());
        return false;
    }
}

void MecanumController::velocity_fl_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    target_vel_fl_ = msg->data;
}

void MecanumController::velocity_fr_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    target_vel_fr_ = msg->data;
}

void MecanumController::velocity_rl_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    target_vel_rl_ = msg->data;
}

void MecanumController::velocity_rr_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    target_vel_rr_ = msg->data;
}

void MecanumController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Twistメッセージから4輪の速度を計算
    compute_wheel_velocities(msg->linear.x, msg->linear.y, msg->angular.z);
}

void MecanumController::compute_wheel_velocities(double linear_x, double linear_y, double angular_z) {
    // メカナム逆運動学
    // 参考: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
    // vFL = vx - vy - ω*(Lx+Ly)
    // vFR = vx + vy + ω*(Lx+Ly)
    // vRL = vx + vy - ω*(Lx+Ly)
    // vRR = vx - vy + ω*(Lx+Ly)
    
    double half_base = wheel_base_ / 2.0;
    double half_track = track_width_ / 2.0;
    double wheel_radius = half_base + half_track; // 回転半径
    
    target_vel_fl_ = linear_x - linear_y - angular_z * wheel_radius;
    target_vel_fr_ = linear_x + linear_y + angular_z * wheel_radius;
    target_vel_rl_ = linear_x + linear_y - angular_z * wheel_radius;
    target_vel_rr_ = linear_x - linear_y + angular_z * wheel_radius;
    
    RCLCPP_DEBUG(this->get_logger(), "Wheel velocities: FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f",
                target_vel_fl_, target_vel_fr_, target_vel_rl_, target_vel_rr_);
}

void MecanumController::send_velocity_commands() {
    if(!serial_port_.is_open()) return;

    // Send commands for all four motors
    std::vector<uint8_t> motors = {MOTOR_FL_ID_, MOTOR_FR_ID_, MOTOR_RL_ID_, MOTOR_RR_ID_};
    std::vector<double> velocities = {target_vel_fl_, target_vel_fr_, target_vel_rl_, target_vel_rr_};
    std::vector<const char*> motor_names = {"FL", "FR", "RL", "RR"};

    for(size_t i = 0; i < motors.size(); ++i) {
        auto command = create_velocity_command(motors[i], velocities[i]);
        try {
            boost::asio::write(serial_port_, boost::asio::buffer(command));
            RCLCPP_DEBUG(this->get_logger(), "Sent velocity to motor %s (ID 0x%02X): %.2f", 
                        motor_names[i], motors[i], velocities[i]);
        } catch(const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Boost Asio write failed: %s", e.what());
        }
    }

    // Request feedback from all motors
    request_and_receive_feedback();
}

int32_t MecanumController::decode_velocity_feedback(const std::vector<uint8_t>& response) {
    if(response.size() < 6) {
        RCLCPP_ERROR(this->get_logger(), "Response too short to decode velocity feedback.");
        return 0;
    }
    int32_t velocity = static_cast<int32_t>(response[4]) |
                       (static_cast<int32_t>(response[5]) << 8);
    return velocity;
}

void MecanumController::request_and_receive_feedback() {
    if(!serial_port_.is_open()) return;

    std::vector<uint8_t> motors = {MOTOR_FL_ID_, MOTOR_FR_ID_, MOTOR_RL_ID_, MOTOR_RR_ID_};
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers = {
        vel_fl_publisher_, vel_fr_publisher_, vel_rl_publisher_, vel_rr_publisher_
    };
    std::vector<const char*> motor_names = {"FL", "FR", "RL", "RR"};

    for(size_t i = 0; i < motors.size(); ++i) {
        std::vector<uint8_t> request = {motors[i], CMD_GET_STATUS_, 0x00, 0x00, 0x00, 0x00};
        uint8_t crc_req = calc_crc8(request);
        request.push_back(crc_req);

        // Send request
        try {
            boost::asio::write(serial_port_, boost::asio::buffer(request));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Boost Asio Write failed: %s", e.what());
            continue;
        }

        // Receive response
        try {
            std::vector<uint8_t> response_buffer(32);
            size_t bytes_transferred = serial_port_.read_some(boost::asio::buffer(response_buffer));

            if(bytes_transferred > 0) {
                response_buffer.resize(bytes_transferred);
                int32_t velocity_feedback = decode_velocity_feedback(response_buffer);

                auto feedback_msg = std::make_shared<std_msgs::msg::Float64>();
                feedback_msg->data = static_cast<double>(velocity_feedback) / 1000.0;
                publishers[i]->publish(*feedback_msg);

                RCLCPP_DEBUG(this->get_logger(), "Received velocity feedback from motor %s (ID 0x%02X): %.2f", 
                            motor_names[i], motors[i], feedback_msg->data);
            }
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Boost Asio Read failed: %s", e.what());
        }
    }
}

uint8_t MecanumController::calc_crc8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0x00;
    const uint8_t poly = 0x8C; // reflected polynomial for CRC-8/MAXIM
    for (auto b : data) {
        crc ^= b;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ poly;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> MecanumController::create_velocity_command(uint8_t motor_id, double target_velocity) {
    int16_t vel = static_cast<int16_t>(target_velocity * max_rpm_);
    std::vector<uint8_t> frame;
    frame.reserve(7);
    frame.push_back(motor_id);
    frame.push_back(CMD_VELOCITY_);
    frame.push_back(static_cast<uint8_t>(vel & 0xFF));
    frame.push_back(static_cast<uint8_t>((vel >> 8) & 0xFF));
    frame.push_back(0x00);
    frame.push_back(0x00);
    uint8_t crc = calc_crc8(frame);
    frame.push_back(crc);
    return frame;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MecanumController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
