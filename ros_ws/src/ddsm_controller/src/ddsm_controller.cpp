#include "ddsm_controller/ddsm_controller.hpp"
#include <chrono>
#include <thread>
#include <unistd.h>

DDSMController::DDSMController() : rclcpp::Node("ddsm_controller_node") {
    declare_parameters();
    get_parameters();

    motor_vel_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/motor_vel_rpm", 10, std::bind(&DDSMController::velocity_callback, this, std::placeholders::_1));
    
    motor_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("motor_vel_feedback", 10);

    setup_serial_port(port_name_, static_cast<unsigned int>(baud_rate_));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&DDSMController::request_and_receive_feedback, this));        
}

// calc_crc8 is implemented at the bottom in the helpers section

void DDSMController::declare_parameters() {
    this->declare_parameter<std::string>("port_name", "/dev/ttyACM0"); //変更するべき
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("motor_id", 1);
    this->declare_parameter<int>("max_rpm", 200);
}

void DDSMController::get_parameters() {
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    motor_id_ = this->get_parameter("motor_id").as_int();
    max_rpm_ = this->get_parameter("max_rpm").as_int();
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

void DDSMController::velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    if(!serial_port_.is_open()) return;

    double target_velocity = msg->data;
    auto command = create_velocity_command(target_velocity);

    try {
        boost::asio::write(serial_port_, boost::asio::buffer(command));
        RCLCPP_INFO(this->get_logger(), "Sent velocity to ID 0x%02X: %.2f", motor_id_, target_velocity);

    } catch(const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Boost Asio write failed: %s", e.what());
    }
}

int32_t DDSMController::decode_velocity_feedback(const std::vector<uint8_t>& response) {
    if(response.size() < 6) {
        RCLCPP_ERROR(this->get_logger(), "Response too short to decode velocity feedback.");
        return 0;
    }
    int32_t velocity = static_cast<int32_t>(response[4]) |
                       (static_cast<int32_t>(response[5]) << 8);
    return velocity;
}

void DDSMController::request_and_receive_feedback() {
    if(!serial_port_.is_open()) return;

    std::vector<uint8_t> request = {static_cast<uint8_t>(motor_id_), CMD_GET_STATUS_, 0x00, 0x00, 0x00, 0x00};
    uint8_t crc_req = calc_crc8(request);
    request.push_back(crc_req);

    // 送信
    try {
        boost::asio::write(serial_port_, boost::asio::buffer(request));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 1ms待機 変更したほうがいい
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Boost Asio Write failed: %s", e.what());
        return;
    }

    // 受信
    try {
        std::vector<uint8_t> response_buffer(32);
        size_t bytes_transferred = serial_port_.read_some(boost::asio::buffer(response_buffer));

        if(bytes_transferred > 0) {
            response_buffer.resize(bytes_transferred);
            int32_t velocity_feedback = decode_velocity_feedback(response_buffer);

            auto feedback_msg = std::make_shared<std_msgs::msg::Float64>();
            feedback_msg->data = static_cast<double>(velocity_feedback) / 1000.0; // 単位変換
            motor_vel_publisher_->publish(*feedback_msg);

            RCLCPP_INFO(this->get_logger(), "Received velocity feedback from ID 0x%02X: %.2f", motor_id_, feedback_msg->data);
        }
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Boost Asio Read failed: %s", e.what());
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
uint8_t DDSMController::calc_crc8(const std::vector<uint8_t>& data) {
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

std::vector<uint8_t> DDSMController::create_velocity_command(double target_velocity) {
    // Example: motor_id, CMD_VELOCITY, vel_l, vel_h, 0x00, 0x00, crc
    // Scale velocity to int16 (e.g., m/s * 1000)
    int16_t vel = static_cast<int16_t>(target_velocity * max_rpm_);
    std::vector<uint8_t> frame;
    frame.reserve(7);
    frame.push_back(static_cast<uint8_t>(motor_id_));
    frame.push_back(CMD_VELOCITY_);
    frame.push_back(static_cast<uint8_t>(vel & 0xFF));
    frame.push_back(static_cast<uint8_t>((vel >> 8) & 0xFF));
    frame.push_back(0x00);
    frame.push_back(0x00);
    uint8_t crc = calc_crc8(frame);
    frame.push_back(crc);
    return frame;
}




