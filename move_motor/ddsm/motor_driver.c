#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <math.h>           // 逆運動学計算用
#include <sys/mman.h>       // 共有メモリ用
#include <sys/stat.h>

// 共通で用いて受け渡しする構造体
#include "common.h"

//yamlファイルに書きたいけどあとからする
// メカナムホイールパラメータ
const double wheel_base_x = 0.42;    // ホイールベース X方向 [m] (仮)
const double wheel_base_y = 0.34;   // ホイールベース Y方向 [m] (仮)
const double wheel_radius = 0.08;   // ホイール半径 [m]
const double gain = 1.0;            // ゲイン()

// モーター補正係数（必要に応じて調整）
const double motor_correction_fl = 1.0;  // Front Left
const double motor_correction_fr = 1.0;  // Front Right
const double motor_correction_rl = 1.0;  // Rear Left
const double motor_correction_rr = 1.0;  // Rear Right

// メカナム逆運動学計算関数
void calculate_mecanum_rpm(double vx, double vy, double wz, int16_t *rpm) {
    const double lxy_sum = wheel_base_x + wheel_base_y;
    const double rad_to_rpm = 60.0 / (2.0 * M_PI);

    // 標準メカナムホイール運動学（物理的配置に合わせる）
    const double wheel_front_left_vel = (vx - vy - lxy_sum * wz) / wheel_radius;
    const double wheel_front_right_vel = (vx + vy + lxy_sum * wz) / wheel_radius;
    const double wheel_rear_left_vel = (vx + vy - lxy_sum * wz) / wheel_radius;
    const double wheel_rear_right_vel = (vx - vy + lxy_sum * wz) / wheel_radius;

    // Convert to RPM with hardware correction factors applied
    rpm[0] = (int16_t)(wheel_front_left_vel * rad_to_rpm * motor_correction_fl);
    rpm[1] = (int16_t)(wheel_front_right_vel * rad_to_rpm * (-1) * motor_correction_fr); // 逆に回るから
    rpm[2] = (int16_t)(wheel_rear_left_vel * rad_to_rpm * motor_correction_rl);
    rpm[3] = (int16_t)(wheel_rear_right_vel * rad_to_rpm * (-1) * motor_correction_rr);
}

// CRC-8/MAXIM 計算 (変更なし)
uint8_t calc_crc8_maxim(uint8_t *data, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
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

// packet作成
void create_vel_packet(uint8_t *packet, uint8_t motor_id, int rpm) {
    if (rpm > 330) rpm = 330;
    if (rpm < -330) rpm = -330;
    int send_rpm = (motor_id == 1 || motor_id == 3) ? -rpm : rpm;

    packet[0] = motor_id;
    packet[1] = 0x64;
    packet[2] = (uint8_t)(send_rpm >> 8);
    packet[3] = (uint8_t)(send_rpm & 0xFF);
    packet[4] = 0x00;
    packet[5] = 0x00;
    packet[6] = 0x0A;
    packet[7] = 0x00;
    packet[8] = 0x00;
    packet[9] = calc_crc8_maxim(packet, 9);
}

// シリアルポート初期化 
int init_serial(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) return -1;
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0; // 非ブロッキング気味に設定
    options.c_cc[VTIME] = 1; 
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int main() {
    const char *device = "/dev/ddsm";
    int serial_fd = init_serial(device);
    if (serial_fd < 0) {
        perror("シリアルポート失敗");
        return 1;
    }

    printf("----DDSM制御ループ開始----\n");

    while (1) {
        // 共有メモリから3軸速度を取得
        const double vx = gain * cmd->vel_x;
        const double vy = gain * cmd->vel_y;
        const double wz = gain * cmd->angular_z;

        // メカナム逆運動学でRPMを計算
        int16_t rpm_values[4];
        calculate_mecanum_rpm(vx, vy, wz, rpm_values);

        // 4つのモーターに個別のRPMを送信
        // 確認必要
        // rpm_values[0]: Front Left (ID=1)
        // rpm_values[1]: Front Right (ID=2)
        // rpm_values[2]: Rear Left (ID=3)
        // rpm_values[3]: Rear Right (ID=4)
        for (uint8_t id = 1; id <= 4; id++) {
            uint8_t packet[10];
            int target_rpm = (int)rpm_values[id - 1];
            create_vel_packet(packet, id, target_rpm);
            write(serial_fd, packet, 10);
            
            // 受信応答を待つ場合は usleep を入れる
            usleep(2000); 
        }

        // 制御周期 (例: 50Hz = 20ms)
        usleep(20000);
    }

    close(serial_fd);
    return 0;
}