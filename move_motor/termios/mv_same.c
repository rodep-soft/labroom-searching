#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

// CRC-8/MAXIM 計算
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

// packet作成 input = rpm
void create_vel_packet(uint8_t *packet, uint8_t motor_id, int rpm) {
    printf("  [パケット作成] モータID:%d, 入力RPM:%d\n", motor_id, rpm);
    
    // 速度制限 (-330 ～ 330)
    if (rpm > 330) rpm = 330;
    if (rpm < -330) rpm = -330;

    // ID 1, 3 の場合は逆回転させる処理
    int send_rpm = (motor_id == 1 || motor_id == 3) ? -rpm : rpm;

    // パケット組み立て
    packet[0] = motor_id;
    packet[1] = 0x64;                         // 速度制御コマンド
    packet[2] = (uint8_t)(send_rpm >> 8);     // 速度上位
    packet[3] = (uint8_t)(send_rpm & 0xFF);   // 速度下位
    packet[4] = 0x00;
    packet[5] = 0x00;
    packet[6] = 0x0A;                         // 加速時間 (10: 回らない場合の対策)
    packet[7] = 0x00;                         // ブレーキ時間 (0: なし)
    packet[8] = 0x00;
    
    // CRC計算
    packet[9] = calc_crc8_maxim(packet, 9);
}

// シリアルポート初期化
int init_serial(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("シリアルポートを開けませんでした");
        return -1;
    }
    printf("シリアルポートのオープン成功: fd=%d\n", fd);

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("ターミナル設定の取得に失敗しました");
        close(fd);
        return -1;
    }

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // 受信設定
    options.c_cc[VMIN] = 10;   
    options.c_cc[VTIME] = 5;   // 0.5秒待機

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("ターミナル設定の適用に失敗しました");
        close(fd);
        return -1;
    }
    printf("シリアルポート初期化完了\n");
    return fd;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("使用法: %s <RPM>\n", argv[0]);
        printf("例: %s 30  (全輪 30rpm で回転)\n", argv[0]);
        return 1;
    }

    int target_rpm = atoi(argv[1]);
    const char *device = "/dev/ddsm";
    int fd = init_serial(device);
    if (fd < 0) {
        fprintf(stderr, "ポート初期化に失敗しました\n");
        return 1;
    }

    printf("--- 4輪一括送信 (全輪 %d RPM) ---\n", target_rpm);

    for (uint8_t id = 1; id <= 4; id++) {
        uint8_t command[10];
        create_vel_packet(command, id, target_rpm);

        printf("\n[モータID: %d] パケット作成完了\n", id);
        printf("[ID:%d] 送信データ: ", id);
        for(int i=0; i<10; i++) printf("%02X ", command[i]);
        printf("\n");

        // 送信
        ssize_t bytes_written = write(fd, command, 10);
        if (bytes_written < 0) {
            fprintf(stderr, "[ID:%d] 送信エラー\n", id);
            continue;
        } else if (bytes_written < 10) {
            fprintf(stderr, "[ID:%d] 警告: %zd/10 バイトのみ送信\n", id, bytes_written);
        } else {
            printf("[ID:%d] 送信成功: %zd バイト\n", id, bytes_written);
        }

        // 短い待機を入れてから受信を確認
        usleep(10000); // 10ms
        
        uint8_t res[10];
        int n = read(fd, res, 10);
        if (n > 0) {
            printf("[ID:%d] 受信完了: %d バイト\n", id, n);
            printf("[ID:%d] 受信データ: ", id);
            for(int i=0; i<n; i++) printf("%02X ", res[i]);
            printf("\n");
        } else if (n == 0) {
            printf("[ID:%d] 受信データなし\n", id);
        } else {
            fprintf(stderr, "[ID:%d] 受信エラー\n", id);
        }
        printf("---------------------------\n");
    }

    close(fd);
    printf("全モーターへの送信を完了しました。\n");
    return 0;
}