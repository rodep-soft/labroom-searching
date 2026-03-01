#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <math.h>
#include "common.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif  

#ifdef __cplusplus
extern "C" {
#endif

// パラメータ
const double L_SUM = 0.42 + 0.34; // wheel_base_x + y
const double WHEEL_R = 0.08;
const int MOTOR_MAX_RPM = 330;
const double STOP_EPS = 0.02;
const int USER_MAX_RPM = 60;

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
    int fd = open("/dev/ddsm", O_RDWR | O_NOCTTY | O_SYNC);
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

    // 停止判定
    int is_braking = (fabs(vx) < STOP_EPS && fabs(vy) < STOP_EPS && fabs(wz) < STOP_EPS);

    double rad_to_rpm = 60.0 / (2.0 * M_PI);

    double v[4];
    v[0] = (vx + vy - L_SUM * wz) / WHEEL_R; // FL
    v[1] = (vx - vy + L_SUM * wz) / WHEEL_R; // FR
    v[2] = (vx - vy - L_SUM * wz) / WHEEL_R; // RL
    v[3] = (vx + vy + L_SUM * wz) / WHEEL_R; // RR

    for (int id = 1; id <= 4; id++) {
        uint8_t pkt[10] = {0};
        pkt[0] = (uint8_t)id;
        pkt[1] = 0x64; // Speed Loop Mode

        if (is_braking) {
            // --- Active Brake 設定 ---
            pkt[2] = 0x00; // 速度高位 0
            pkt[3] = 0x00; // 速度低位 0
            pkt[4] = 0x00;
            pkt[5] = 0x00;
            pkt[6] = 0x05;
            pkt[7] = 0x00; // ブレーキフラグ (datasheet)
            pkt[8] = 0x00;
        } else {
            // --- 通常走行設定 ---
            int rpm = (int)(v[id-1] * rad_to_rpm);
            int rpm_limit = USER_MAX_RPM;
            if (rpm_limit > MOTOR_MAX_RPM) rpm_limit = MOTOR_MAX_RPM;

            if (rpm > rpm_limit) rpm = rpm_limit;
            if (rpm < -rpm_limit) rpm = -rpm_limit;

            // モーターID1,3が逆向きだから反転
            if (id == 1 || id == 3) {
                rpm = -rpm;
            }

            pkt[2] = (uint8_t)(rpm >> 8);
            pkt[3] = (uint8_t)(rpm & 0xFF);
            pkt[4] = 0x00;
            pkt[5] = 0x00;
            pkt[6] = 0x05; // 加速
            pkt[7] = 0x00;
            pkt[8] = 0x00;
        }

        pkt[9] = calc_crc8(pkt, 9);
        write(fd, pkt, 10);
        usleep(2500); // バス衝突防止
    }
}

#ifdef __cplusplus
}
#endif
