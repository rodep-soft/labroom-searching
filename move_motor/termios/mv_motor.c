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
        crc ^= data[i]; // データの各バイトと現在のCRC値をXOR
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                // 最下位ビットが1なら、右シフトして多項式 0x8C とXOR
                crc = (crc >> 1) ^ 0x8C;
            } else {
                // 0なら、そのまま右シフト
                crc >>= 1;
            }
        }
    }
    return crc;
}

// packet作成
void create_vel_packet(uint8_t *packet, uint8_t motor_id, int m_rpm) {
    int rpm = m_rpm;
    if (rpm > 330) rpm = 330;
    if (rpm < -330) rpm = -330;

    // ID 1, 3 の場合は逆回転させる処理
    int send_rpm;
    if (motor_id == 1 || motor_id == 3) {
        send_rpm = -rpm;
    } else {
        send_rpm = rpm;
    }

    // packet作成
    packet[0] = motor_id;                  // DATA[0]: モータID
    packet[1] = 0x64;                       // DATA[1]: 速度制御コマンド
    packet[2] = (uint8_t)(send_rpm >> 8);   // DATA[2]: 速度上位8ビット
    packet[3] = (uint8_t)(send_rpm & 0xFF); // DATA[3]: 速度下位8ビット
    packet[4] = 0x00;                       // DATA[4]: 固定
    packet[5] = 0x00;                       // DATA[5]: 固定
    packet[6] = 0x00;                       // DATA[6]: 加速時間(デフォルト0)
    packet[7] = 0x00;                       // DATA[7]: ブレーキ(デフォルト0)
    packet[8] = 0x00;                       // DATA[8]: 固定
    
    // CRC計算と格納
    packet[9] = calc_crc8_maxim(packet, 9);
}



// シリアルポートを初期化する関数
// 成功時はファイルディスクリプタ、失敗時は -1
int init_serial(const char *device) {
    // O_RDWR: 読み書き両用, O_NOCTTY: 制御端末にしない, O_SYNC: 同期モード
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("シリアルポートを開けませんでした");
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("ターミナル設定の取得に失敗しました");
        close(fd);
        return -1;
    }

    // B115200: ボーレート115200に設定
    cfsetispeed(&options, B115200); //input
    cfsetospeed(&options, B115200); //output

    // --- 制御フラグ (c_cflag) の設定 ---
    options.c_cflag |= (CLOCAL | CREAD); // デバイスとして認識して、受信を可能に
    options.c_cflag &= ~PARENB;          // パリティビットなし
    options.c_cflag &= ~CSTOPB;          // ストップビットを1ビットに設定
    options.c_cflag &= ~CSIZE;           // データサイズ指定をマスク
    options.c_cflag |= CS8;              // 8ビットデータに設定
    options.c_cflag &= ~CRTSCTS;         // ハードウェアフロー制御を無効化

    // --- ローカルフラグ (c_lflag) の設定 ---
    // 非カノカルモード（入力をそのまま処理する）に設定
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // --- 入力フラグ (c_iflag) の設定 ---
    // ソフトウェアフロー制御(XON/XOFF)を無効化
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // --- 出力フラグ (c_oflag) の設定 ---
    // 生のデータを出力（加工なし）
    options.c_oflag &= ~OPOST;

    // --- 制御文字 (c_cc) の設定 ---
    // ブロック読み取り: 10バイト受信するか5秒でタイムアウト
    options.c_cc[VMIN] = 10;   // 10バイト受信するまで待機
    options.c_cc[VTIME] = 50;  // タイムアウト5秒（0.1s×50）

    // 設定を即座に反映
    if (tcsetattr(fd, TCSANOW, &options) != 0) {  // エラーチェック追加
        perror("ターミナル設定の適用に失敗しました");
        close(fd);
        return -1;
    }

    return fd;
}
// データを送信する関数
// 引数: ファイルディスクリプタ, 送信データ配列
// 戻り値: 送信バイト数、失敗時は -1
int send_ddsm_data(int fd, uint8_t data[10]) {
    if (fd < 0) return -1;

    // write関数を使用してデータをシリアルポートに書き込む
    // 引数: ファイルディスクリプタ, データ配列の先頭ポインタ, 送信サイズ
    ssize_t bytes_written = write(fd, data, 10);

    if (bytes_written < 0) {
        perror("データの送信に失敗しました");
        return -1;
    } else if (bytes_written < 10) {
        fprintf(stderr, "警告: 予期したバイト数が送信されません（送信: %zd/10）\n", bytes_written);
    }
    printf("送信完了: %zd バイト送信しました\n", bytes_written);
    return bytes_written;
}

// レスポンスを受信する関数
// 引数: ファイルディスクリプタ, 受信バッファ, バッファサイズ
// 戻り値: 受信バイト数
int receive_ddsm_data(int fd, uint8_t *buffer, int size) {
    if (fd < 0 || buffer == NULL) return -1;

    ssize_t bytes_read = read(fd, buffer, size);
    
    if (bytes_read < 0) {
        perror("データの受信に失敗しました");
        return -1;
    } else if (bytes_read == 0) {
        printf("受信データなし\n");
    } else {
        printf("受信完了: %zd バイト受信しました\n", bytes_read);
        printf("受信データ: ");
        for (ssize_t i = 0; i < bytes_read; i++) {
            printf("0x%02X ", buffer[i]);
        }
        printf("\n");
    }
    return bytes_read;
}

int main(int argc, char *argv[]) {
    // RPM引数の取得（指定がなければ500）
    int target_rpm = 500;
    if (argc > 1) {
        char *endptr = NULL;
        long val = strtol(argv[1], &endptr, 10);
        if (endptr == argv[1] || *endptr != '\0') {
            fprintf(stderr, "RPMは整数で指定してください\n");
            return 1;
        }
        target_rpm = (int)val;
    }

    // ポートの初期化
    const char *device = "/dev/ttyACM0";
    int fd = init_serial(device);
    if (fd < 0) {
        fprintf(stderr, "ポートの初期化に失敗しました\n");
        return 1;
    }
    printf("ポートの初期化成功 (RPM=%d)\n\n", target_rpm);

    // 送信コマンドの準備（ID=1, 入力RPM）
    uint8_t command[10];
    create_vel_packet(command, 0x01, target_rpm);

    printf("送信パケット: ");
    for (int i = 0; i < 10; i++) {
        printf("0x%02X ", command[i]);
    }
    printf("\n");

    // データの送信
    printf("データを送信します...\n");
    int result = send_ddsm_data(fd, command);
    if (result < 0) {
        fprintf(stderr, "送信処理に失敗しました\n");
        close(fd);
        return 1;
    }

    // レスポンスを待機（VMIN/VTIMEで自動待機）
    printf("\nデータを受信します...\n");
    uint8_t response[256];
    int received = receive_ddsm_data(fd, response, sizeof(response));

    // 受信データのCRCをチェック
    if (received == 10) {
        uint8_t cal_crc = calc_crc8_maxim(response, 9);
        if (cal_crc == response[9]) {
            printf("✓ CRCチェック成功\n");
        } else {
            printf("✗ CRCチェック失敗（計算: 0x%02X, 受信: 0x%02X）\n", cal_crc, response[9]);
        }
    }

    // ポートを閉じる
    close(fd);
    printf("\nポートをクローズしました\n");
    printf("通信を終了しました\n");

    return (result < 0) ? 1 : 0;
}