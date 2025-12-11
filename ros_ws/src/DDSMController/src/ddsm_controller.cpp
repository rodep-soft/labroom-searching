
#include "simple_motor_node/simple_motor.hpp"


SimpleMotor::SimpleMotor() : Node("simple_motor_node"),
    io_vcontext_(), serial_port_(io_context_) 
{
    declare_parameters();
    get_parameters();

    velocity_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel", 10, std::bind(&SimpleMotor::velocity_callback, this, std::placeholders::_1));
    
    velocity_publisher_ = this->create_publisher<std::msgs::msg::Float64>("motor_vel_feedback", 10);

    setup_serial_port(serial_port_name_, baudrate_);

    timer_ = this->create_wall_timer(
        std::chrono::milliserconds(200),
        std::bind(&SimpleMotor::request_and_receive_feedback, this));        
}

void declare_parameters() {
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0"); //変更するべき
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("motor_id", 1);
}

void get_parameters() {
    this->get_parameter("port_name", serial_port_name_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("motor_id", motor_id_)
}

bool setup_serial_port(const std::string& port_name, unsigned int baud_rate) {
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

void velocity_callback(const std:msgs::msg::Shared_por)















int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMotor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




