#include "ddsm_controller/ddsm_controller.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <thread>

DDSMController::DDSMController() : rclcpp::Node("ddsm_controller_node") {
    declare_parameters();
    get_parameters();

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DDSMController::velocity_callback, this, std::placeholders::_1));

    setup_serial_port(port_name_, static_cast<unsigned int>(baud_rate_));
}

// calc_crc8 is implemented at the bottom in the helpers section

void DDSMController::declare_parameters() {
    this->declare_parameter<std::string>("port_name", "/dev/ddsm");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("motor_max_rpm", 330);
    this->declare_parameter<int>("user_max_rpm", 60);
    this->declare_parameter<double>("stop_eps", 0.02);
    this->declare_parameter<double>("wheel_radius", 0.08);
    this->declare_parameter<double>("wheel_base_x", 0.42);
    this->declare_parameter<double>("wheel_base_y", 0.34);
}

void DDSMController::get_parameters() {
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    motor_max_rpm_ = this->get_parameter("motor_max_rpm").as_int();
    user_max_rpm_ = this->get_parameter("user_max_rpm").as_int();
    stop_eps_ = this->get_parameter("stop_eps").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_x_ = this->get_parameter("wheel_base_x").as_double();
    wheel_base_y_ = this->get_parameter("wheel_base_y").as_double();
}

bool DDSMController::setup_serial_port(const std::string& port_name, unsigned int baud_rate) {
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

void DDSMController::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if(!serial_port_.is_open()) return;
    const bool braking = is_brake_command(*msg);
    const auto target_rpms = calculate_target_rpms_from_twist(*msg);

    try {
        for (int id = 1; id <= 4; ++id) {
            const int target_rpm = target_rpms[static_cast<std::size_t>(id - 1)];
            const auto command = create_velocity_command(static_cast<uint8_t>(id), target_rpm, braking);
            boost::asio::write(serial_port_, boost::asio::buffer(command));
            std::this_thread::sleep_for(std::chrono::microseconds(2500));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Sent cmd_vel: vx=%.2f vy=%.2f wz=%.2f | brake=%s | rpm[FL,FR,RL,RR]=[%d,%d,%d,%d]",
            msg->linear.x,
            msg->linear.y,
            msg->angular.z,
            braking ? "true" : "false",
            target_rpms[0],
            target_rpms[1],
            target_rpms[2],
            target_rpms[3]);

    } catch(const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Boost Asio write failed: %s", e.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DDSMController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// --- helpers implementation ---
uint8_t DDSMController::calc_crc8(const std::vector<uint8_t>& data) const {
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

std::vector<uint8_t> DDSMController::create_velocity_command(uint8_t motor_id, int target_rpm, bool is_braking) const {
    const int rpm_limit = std::min(user_max_rpm_, motor_max_rpm_);
    target_rpm = std::clamp(target_rpm, -rpm_limit, rpm_limit);
    int send_rpm = (motor_id == 1 || motor_id == 3) ? -target_rpm : target_rpm;

    std::vector<uint8_t> frame(10);
    frame[0] = motor_id;
    frame[1] = CMD_SPEED_LOOP_;

    if (is_braking) {
        frame[2] = 0x00;
        frame[3] = 0x00;
    } else {
        frame[2] = static_cast<uint8_t>((send_rpm >> 8) & 0xFF);
        frame[3] = static_cast<uint8_t>(send_rpm & 0xFF);
    }

    frame[4] = 0x00;
    frame[5] = 0x00;
    frame[6] = 0x05;
    frame[7] = 0x00;
    frame[8] = 0x00;

    std::vector<uint8_t> crc_data(frame.begin(), frame.begin() + 9);
    frame[9] = calc_crc8(crc_data);
    return frame;
}

std::array<int, 4> DDSMController::calculate_target_rpms_from_twist(const geometry_msgs::msg::Twist& twist) const {
    const double l = wheel_base_x_ + wheel_base_y_;
    const double vx = twist.linear.x;
    const double vy = twist.linear.y;
    const double wz = twist.angular.z;

    const double w_fl = (vx + vy - l * wz) / wheel_radius_;
    const double w_fr = (vx - vy + l * wz) / wheel_radius_;
    const double w_rl = (vx - vy - l * wz) / wheel_radius_;
    const double w_rr = (vx + vy + l * wz) / wheel_radius_;

    constexpr double pi = 3.14159265358979323846;
    const double to_rpm = 60.0 / (2.0 * pi);

    return {
        static_cast<int>(std::lround(w_fl * to_rpm)),
        static_cast<int>(std::lround(w_fr * to_rpm)),
        static_cast<int>(std::lround(w_rl * to_rpm)),
        static_cast<int>(std::lround(w_rr * to_rpm))
    };
}

bool DDSMController::is_brake_command(const geometry_msgs::msg::Twist& twist) const {
    return std::fabs(twist.linear.x) < stop_eps_ &&
           std::fabs(twist.linear.y) < stop_eps_ &&
           std::fabs(twist.angular.z) < stop_eps_;
}




