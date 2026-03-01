#include "motor.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <stdexcept>

MotorController::MotorController()
    : serial_port_(io_context_) {
    init_port(port_name_, baud_rate_);
}

MotorController::MotorController(const std::string& port_name, unsigned int baud_rate)
    : serial_port_(io_context_), port_name_(port_name), baud_rate_(baud_rate) {
    init_port(port_name_, baud_rate_);
}

void MotorController::init_port(const std::string& port_name, unsigned int baud_rate) {
    try {
        serial_port_.open(port_name);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        
        std::cout << "Serial port initialized: " << port_name << " at " << baud_rate << " baud." << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
    }
}

uint8_t MotorController::calc_crc8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0x00;
    for (uint8_t b : data) {
        crc ^= b;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> MotorController::create_velocity_command(uint8_t motor_id, int rpm) {
    // 速度制限
    rpm = std::clamp(rpm, -330, 330);

    // ID 1, 3 の反転処理
    int send_rpm = (motor_id == 1 || motor_id == 3) ? -rpm : rpm;

    std::vector<uint8_t> packet(10);
    packet[0] = motor_id;
    packet[1] = SPEED_LOOP_COMMAND_;
    packet[2] = static_cast<uint8_t>(send_rpm >> 8);
    packet[3] = static_cast<uint8_t>(send_rpm & 0xFF);
    packet[4] = 0x00;
    packet[5] = 0x00;
    packet[6] = 0x0A;                         // 加速時間
    packet[7] = 0x00;                         // ブレーキ時間
    packet[8] = 0x00;
    
    // CRC計算
    std::vector<uint8_t> crc_data(packet.begin(), packet.begin() + 9);
    packet[9] = calc_crc8(crc_data);

    return packet;
}

void MotorController::send_velocity_command(uint8_t motor_id, int rpm) {
    if (!serial_port_.is_open()) return;

    std::vector<uint8_t> packet = create_velocity_command(motor_id, rpm);

    // 送信
    boost::asio::write(serial_port_, boost::asio::buffer(packet));

    // デバッグ出力
    std::cout << "[ID:" << (int)motor_id << "] Sent: ";
    for(auto b : packet) std::printf("%02X ", b);
    std::cout << std::endl;

    // 受信
    // 注意: DDSMの仕様に合わせてサイズ調整が必要。ここでは10バイト待機。
    std::vector<uint8_t> response(10);
    boost::system::error_code ec;
    size_t len = serial_port_.read_some(boost::asio::buffer(response), ec);
    
    if (!ec && len > 0) {
        std::cout << "[ID:" << (int)motor_id << "] Recv: ";
        for(size_t i=0; i<len; ++i) std::printf("%02X ", response[i]);
        std::cout << std::endl;
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <motor_id> <speed_rpm>" << std::endl;
        return 1;
    }

    try {
        int id_value = std::stoi(argv[1]);
        int speed_rpm = std::stoi(argv[2]);

        if (id_value < 0 || id_value > 255) {
            std::cerr << "motor_id must be in range [0, 255]" << std::endl;
            return 1;
        }

        MotorController controller;
        controller.send_velocity_command(static_cast<uint8_t>(id_value), speed_rpm);
    } catch (const std::exception& e) {
        std::cerr << "Invalid argument: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}