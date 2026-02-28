#include <boost/asio.hpp>
#include <vector>
#include <string>
#include <iostream>


int main(int argc, char* argv[]) {
    if(argc < 2) {
        std::cout<<"Usage: ./set_motor_id <new_id>"<<std::endl;
        std::cout<<"Example: ./set_motor_id 1"<<std::endl;
        return -1;
    }

    uint8_t target_id = static_cast<uint8_t>(std::stoi(argv[1]));
    std::string port_name = "/dev/ttyUSB0";
    uint32_t baud_rate = 115200;

    boost::asio::io_context io;
    boost::asio::serial_port port(io);

    try {
        //シリアルポートのオープン
        port.open(port_name);
        port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        port.set_option(boost::asio::serial_port_base::character_size(8));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        std::vector<uint8_t> command = {
            0xAA,
            0x55,
            0x53,
            target_id,//ID
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };

        boost::asio::write(port, boost::asio::buffer(command));

        std::cout << "------------------------------------------" << std::endl;
        std::cout << "Command sent: Set Motor ID to [" << (int)target_id << "]" << std::endl;
        std::cout << "------------------------------------------" << std::endl;
        std::cout << "NEXT STEP:" << std::endl;
        std::cout << "1. Power OFF the motor." << std::endl;
        std::cout << "2. Power ON the motor again." << std::endl;
        std::cout << "3. The new ID [" << (int)target_id << "] will be active." << std::endl;
        std::cout << "------------------------------------------" << std::endl;
    } catch(std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;

}
