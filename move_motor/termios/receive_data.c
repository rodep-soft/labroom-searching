#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

// CRC-8/MAXIMの計算関数 (多項式: x8 + x5 + x4 + 1)
uint8_t calculate_crc8_maxim(uint8_t *data, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C; // 0x8Cは多項式の逆転値
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int main() {
    const char *device = "/dev/ttyACM0";
    int fd;
    struct termios tty;

    // ポートのオープン
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("ポートが開けません");
        return -1;
    }

    // termiosの設定
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr error");
        return -1;
    }

    cfsetospeed(&tty, B115200); // 送信ボーレート
    cfsetispeed(&tty, B115200); // 受信ボーレート

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8bit
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 10; // 10バイト受信するまで待機
    tty.c_cc[VTIME] = 5;  // 0.5秒タイムアウト

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr error");
        return -1;
    }

    // Protocol 2: フィードバック要求データの作成
    uint8_t motor_id = 0x01; // モーターID (環境に合わせて変更)
    uint8_t send_buf[10] = {motor_id, 0x74, 0, 0, 0, 0, 0, 0, 0, 0};
    send_buf[9] = calculate_crc8_maxim(send_buf, 9);

    // データ送信
    printf("Sending command... ");
    ssize_t bytes_written = write(fd, send_buf, 10);
    if (bytes_written < 0) {
        perror("データの送信に失敗しました");
        close(fd);
        return -1;
    } else if (bytes_written < 10) {
        fprintf(stderr, "警告: 予期したバイト数が送信されません（送信: %zd/10）\n", bytes_written);
    }
    printf("Done. (%zd bytes sent)\n", bytes_written);

    // データ受信 (10バイト)
    uint8_t recv_buf[10];
    int n = read(fd, recv_buf, sizeof(recv_buf));

    if (n == 10) {
        // CRCチェック
        uint8_t cal_crc = calculate_crc8_maxim(recv_buf, 9);
        if (cal_crc == recv_buf[9]) {
            // // データ解析
            // int16_t torque_current = (recv_buf[2] << 8) | recv_buf[3];
            // int16_t velocity = (recv_buf[4] << 8) | recv_buf[5];
            // uint8_t temp = recv_buf[6];
            // uint8_t pos_raw = recv_buf[7];
            // float angle = pos_raw * (360.0 / 255.0);

            // printf("\n--- Motor Feedback ---\n");
            // printf("ID: %02X\n", recv_buf[0]);
            // printf("Mode: %d\n", recv_buf[1]);
            // printf("Torque Current: %d\n", torque_current);
            // printf("Velocity: %d\n", velocity);
            // printf("Temperature: %d °C\n", temp);
            // printf("Position: %.2f° (Raw: %d)\n", angle, pos_raw);
            // printf("Error Code: %02X\n", recv_buf[8]);
            // printf("----------------------\n");
            printf("%d %d %d %d %d %d %d %d %d %d\n",
                recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3],
                recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7],
                recv_buf[8], recv_buf[9]);
        }
    } else {
        printf("受信失敗: %d バイトしか受信できませんでした。\n", n);
    }

    close(fd);
    return 0;
}