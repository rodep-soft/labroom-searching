#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

/*
 シリアルポートを初期化する関数
 成功時はファイルディスクリプタ、失敗時は -1
 */
int init_serial(const char *device) {
    // O_RDWR: 読み書き両用, O_NOCTTY: 制御端末にしない, O_NDELAY: 非ブロックモード
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("シリアルポートを開けませんでした");
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {  // エラーチェック追加
        perror("ターミナル設定の取得に失敗しました");
        close(fd);
        return -1;
    }

    // B115200: ボーレート115200に設定
    cfsetispeed(&options, B115200); //input
    cfsetospeed(&options, B115200); //output

    // --- 制御フラグ (c_cflag) の設定 ---
    options.c_cflag |= (CLOCAL | CREAD); // デバイスとして認識して、受信を可能にする
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
    // 非ブロック読み取りのためにVMIN=0, VTIME=0に設定
    options.c_cc[VMIN] = 0;   // 最小受信バイト数
    options.c_cc[VTIME] = 0;  // タイムアウト（10ms単位）

    // 設定を即座に反映させる
    if (tcsetattr(fd, TCSANOW, &options) != 0) {  // エラーチェック追加
        perror("ターミナル設定の適用に失敗しました");
        close(fd);
        return -1;
    }

    return fd;
}
/*
 DDSMにデータを送信する関数
 引数: ファイルディスクリプタ, 送信データ配列
 戻り値: 送信バイト数、失敗時は -1
 */
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

/*
 DDSMからのレスポンスを受信する関数
 引数: ファイルディスクリプタ, 受信バッファ, バッファサイズ
 戻り値: 受信バイト数
 */
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
    // デバイスパスの取得（引数なしの場合はデフォルト値）
    const char *device = (argc > 1) ? argv[1] : "/dev/ttyACM0";

    printf("使用デバイス: %s\n", device);

    // 1. ポートの初期化
    int fd = init_serial(device);
    if (fd < 0) {
        fprintf(stderr, "ポートの初期化に失敗しました\n");
        return 1;
    }
    printf("ポートの初期化成功\n\n");

    // 2. 送信データの準備 (DDSMのプロトコルに合わせて値を変更してください)
    // 例として10個のデータを配列に格納
    uint8_t command[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};

    // 3. データの送信
    printf("データを送信します...\n");
    int result = send_ddsm_data(fd, command);
    if (result < 0) {
        fprintf(stderr, "送信処理に失敗しました\n");
        close(fd);
        return 1;
    }

    // 4. レスポンスを待機（最大500ms）
    printf("\nレスポンスを待機中...\n");
    usleep(500000);  // 500ms待機

    // 5. データを受信
    uint8_t response[256];
    int received = receive_ddsm_data(fd, response, sizeof(response));

    // 6. ポートを閉じる
    close(fd);
    printf("\nポートをクローズしました\n");
    printf("通信を終了しました\n");

    return (result < 0) ? 1 : 0;
}