#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

class MecanumController {
public:
    MecanumController(const std::string& port_name = "/dev/ddsm", unsigned int baud_rate = 115200);
    void send_same_rpm_to_all(int rpm);

private:
    void init_port();
    uint8_t calc_crc8_maxim(const std::vector<uint8_t>& data) const;
    std::vector<uint8_t> create_velocity_packet(uint8_t motor_id, int rpm) const;
    void print_packet(const std::vector<uint8_t>& packet, const std::string& prefix, uint8_t motor_id) const;

    std::string port_name_ = "/dev/ddsm";
    unsigned int baud_rate_ = 115200;

    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;

    const int SPEED_LOOP_COMMAND_ = 0x64; // 速度制御コマンド
};
