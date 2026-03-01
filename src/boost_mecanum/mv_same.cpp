#include "mv_same.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

MecanumController::MecanumController(const std::string& port_name, unsigned int baud_rate)
    : port_name_(port_name), baud_rate_(baud_rate), serial_port_(io_context_) {
    init_port();
}

void MecanumController::init_port() {
    serial_port_.open(port_name_);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial_port_.non_blocking(true);

    std::cout << "Serial port initialized: " << port_name_ << " at " << baud_rate_ << " baud" << std::endl;
}

uint8_t MecanumController::calc_crc8_maxim(const std::vector<uint8_t>& data) const {
    uint8_t crc = 0x00;
    for (uint8_t b : data) {
        crc ^= b;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x01) {
                crc = static_cast<uint8_t>((crc >> 1) ^ 0x8C);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> MecanumController::create_velocity_packet(uint8_t motor_id, int rpm) const {
    rpm = std::clamp(rpm, -330, 330);
    int send_rpm = (motor_id == 1 || motor_id == 3) ? -rpm : rpm;

    std::vector<uint8_t> packet(10);
    packet[0] = motor_id;
    packet[1] = SPEED_LOOP_COMMAND_;
    packet[2] = static_cast<uint8_t>((send_rpm >> 8) & 0xFF);
    packet[3] = static_cast<uint8_t>(send_rpm & 0xFF);
    packet[4] = 0x00;
    packet[5] = 0x00;
    packet[6] = 0x0A;
    packet[7] = 0x00;
    packet[8] = 0x00;

    std::vector<uint8_t> crc_data(packet.begin(), packet.begin() + 9);
    packet[9] = calc_crc8_maxim(crc_data);
    return packet;
}

void MecanumController::print_packet(const std::vector<uint8_t>& packet, const std::string& prefix, uint8_t motor_id) const {
    std::cout << "[ID:" << static_cast<int>(motor_id) << "] " << prefix;
    for (uint8_t b : packet) {
        std::printf("%02X ", b);
    }
    std::cout << std::endl;
}

void MecanumController::send_same_rpm_to_all(int rpm) {
    std::cout << "--- 4輪一括送信 (全輪 " << rpm << " RPM) ---" << std::endl;

    for (uint8_t id = 1; id <= 4; ++id) {
        auto packet = create_velocity_packet(id, rpm);
        print_packet(packet, "送信データ: ", id);

        boost::asio::write(serial_port_, boost::asio::buffer(packet));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::vector<uint8_t> response(10);
        boost::system::error_code ec;
        std::size_t len = serial_port_.read_some(boost::asio::buffer(response), ec);

        if (!ec && len > 0) {
            std::cout << "[ID:" << static_cast<int>(id) << "] 受信データ: ";
            for (std::size_t i = 0; i < len; ++i) {
                std::printf("%02X ", response[i]);
            }
            std::cout << std::endl;
        } else if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again || len == 0) {
            std::cout << "[ID:" << static_cast<int>(id) << "] 受信データなし" << std::endl;
        } else {
            std::cout << "[ID:" << static_cast<int>(id) << "] 受信エラー: " << ec.message() << std::endl;
        }

        std::cout << "---------------------------" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <rpm>" << std::endl;
        std::cerr << "Example: " << argv[0] << " 30" << std::endl;
        return 1;
    }

    try {
        int rpm = std::stoi(argv[1]);
        MecanumController controller;
        controller.send_same_rpm_to_all(rpm);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
