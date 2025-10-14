#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

int main() {
    const char* device = "/dev/ttyACM0"; // LiDARのUSBポート
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open device." << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    tcsetattr(fd, TCSANOW, &options);

    // 初期化コマンド送信
    write(fd, "SCIP2.0\n", 8);
    usleep(100000);

    // データ取得コマンド（距離データ）
    write(fd, "GD0000108000\n", 13);
    usleep(100000);

    char buffer[4096];
    int bytesRead = read(fd, buffer, sizeof(buffer));
    if (bytesRead > 0) {
        std::cout << "Received data:\n" << std::string(buffer, bytesRead) << std::endl;
    } else {
        std::cerr << "No data received." << std::endl;
    }

    close(fd);
    return 0;
}