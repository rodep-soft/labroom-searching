#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// boost library
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

class MotorController {
public:
    MotorController(const std::string& port_name, unsigned int baud_rate);
    void reinit_port(const std::string& port_name, unsigned int baud_rate);
    std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double target_velocity);
    void send_velocity_command(uint8_t motor_id, double target_velocity);

private:
    void init_port(const std::string& port_name, unsigned int baud_rate);
    uint8_t calc_crc8(const std::vector<uint8_t>& data);

    // Serial port parameters
    std::string port_name_ = "/dev/ddsm";
    unsigned int baud_rate_ = 115200;

    // Boost Asio components
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;

    std::vector<uint8_t> buffer_;

};