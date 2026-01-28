#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <math.h>
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

// パラメータ
const double L_SUM = 0.42 + 0.34; // wheel_base_x + y
const double WHEEL_R = 0.08;
const double MAX_RPM = 30.0;

// CRC-8 計算
uint8_t calc_crc8(uint8_t *data, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
            else crc >>= 1;
        }
    }
    return crc;
}

int init_ddsm() {
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) return -1;
    struct termios opt;
    tcgetattr(fd, &opt);
    cfsetispeed(&opt, B115200); cfsetospeed(&opt, B115200);
    opt.c_cflag |= (CLOCAL | CREAD | CS8);
    opt.c_lflag &= ~(ICANON | ECHO | ISIG);
    opt.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &opt);
    return fd;
}

void update_ddsm(int fd) {
    if (fd < 0) return;
    double vx = cmd.vel_x;
    double vy = cmd.vel_y;
    double wz = cmd.angular_z;
    double rad_to_rpm = 60.0 / (2.0 * M_PI);

    double v[4];
    v[0] = (vx - vy - L_SUM * wz) / WHEEL_R; // FL
    v[1] = (vx + vy + L_SUM * wz) / WHEEL_R; // FR
    v[2] = (vx + vy - L_SUM * wz) / WHEEL_R; // RL
    v[3] = (vx - vy + L_SUM * wz) / WHEEL_R; // RR

    for (int id = 1; id <= 4; id++) {
        int rpm = (int)(v[id-1] * rad_to_rpm * MAX_RPM);

        if (rpm > 330) rpm = 330;
        if (rpm < -330) rpm = -330;
        
        uint8_t pkt[10] = {id, 0x64, (uint8_t)(rpm>>8), (uint8_t)(rpm&0xFF), 0,0, 0x0A, 0,0, 0};
        // モーターID1,3が逆向きならここで反転
        if (id == 1 || id == 3) {
             int rev_rpm = -rpm;
             pkt[2] = (uint8_t)(rev_rpm>>8);
             pkt[3] = (uint8_t)(rev_rpm&0xFF);
        }
        pkt[9] = calc_crc8(pkt, 9);
        write(fd, pkt, 10);
        usleep(1000); // モーター間の衝突防止
    }
}

#ifdef __cplusplus
}
#endif