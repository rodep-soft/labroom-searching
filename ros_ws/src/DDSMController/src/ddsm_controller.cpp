
#include "DDSMController/ddsm_controller.hpp"


DDSMController::DDSMController() : 
    Node("ddsm_controller_node"), io_vcontext_(), serial_port_(io_context_) 
{
    declare_parameters();
    get_parameters();

    velocity_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel", 10, std::bind(&DDSMController::velocity_callback, this, std::placeholders::_1));
    
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("motor_vel_feedback", 10);

    setup_serial_port(serial_port_name_, baudrate_);

    timer_ = this->create_wall_timer(
        std::chrono::milliserconds(200),
        std::bind(&DDSMController::request_and_receive_feedback, this));        
}

uint8_t DDSMController::calc_crc8(const std::vector<uint8_t>& data) {
  uint8_t crc = 0x00;  // 初期値  (一般的なMaxim CRCの標準)

  const uint8_t reflected_polynomial = 0x8C;

  // データバイトを一つずつ処理
  for (size_t i = 0; i < data.size(); i++) {  // DATA[0]~DATA[8]まで、合計9バイト
    crc ^= data[i];                           // 現在のバイトとCRCレジスタをXOR

    // 各バイトの8ビットを処理 (LSB First)
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {                           // 最下位ビットが1の場合
        crc = (crc >> 1) ^ reflected_polynomial;  // 右シフトして多項式とXOR
      } else {
        crc >>= 1;  // 最下位ビットが0の場合、単に右シフト
      }
    }
  }
  return crc;
}

void DDSMController::declare_parameters() {
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0"); //変更するべき
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("motor_id", 1);
}

void DDSMController::get_parameters() {
    this->get_parameter("port_name", serial_port_name_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("motor_id", motor_id_);
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

void DDSMController::velocity_callback(const std:msgs::msg::SharedPtr msg) {
    if(!serial_port_.is_open()) return;

    double target_velocity = msg-> data;
    auto command = create_velocity_command(target_velocity);

    try {
        boost::asio::write(serial_port_, boost::asio::buffer(command));
        RCLCPP_INFO(this->get_logger(), "Sent velocity to ID 0x%02X: %.2f", motor_id_, target_velocity);

    } catch(const boost::system::system_error& e) {
        if(e.code() != boost::system_error& e){
            RCLCPP_ERROR(this->get_logger(), "Boost Asio write failed: %s", e.what());
        }
    }
}

void DDSMController::decode_velocity_feedback(const std::vector<uint8_t>& response) {
    if(response.size() < 8) {
        RCLCPP_ERROR(this->get_logger(), "Response too short to decode velocity feedback.");
        return 0;
    }

    int32_t torque = static_cast<int32_t>(response[2]) |
                      (static_cast<int32_t>(response[3]) << 8);
                      
    int32_t velocity = static_cast<int32_t>(response[4]) |
                      (static_cast<int32_t>(response[5]) << 8);
    return velocity;
}

void DDSMController::request_and_receive_feedback() {
    if(!serial_port_.is_open()) return;

    std::vector<uint8_t> request = {
        motor_id_, CMD_GET_STATUS_, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t crc_req = calc_crc8(request);
    request.push_back(crc_req & 0xFF);

    // 送信
    try {
        boost::asio::write(serial_port_, boost::asio::buffer(request));
        usleep(1000);  // 1ms待機 変更したほうがいい
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
            velocity_publisher_->publish(*feedback_msg);

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




